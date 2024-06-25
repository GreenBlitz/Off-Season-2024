// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimation.OdometryObservation;
import frc.robot.poseestimation.VisionObservation;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import java.util.NoSuchElementException;
import static frc.robot.RobotContainer.SWERVE;

public class PoseEstimator6328 {

    private static PoseEstimator6328 instance;

    public static PoseEstimator6328 getInstance() {
        if (instance == null) {
            instance = new PoseEstimator6328();
        }
        return instance;
    }

    private Pose2d odometryPose;
    private Pose2d estimatedPose;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
    private final Matrix<N3, N1> qStdDevs;
    private final SwerveDriveKinematics kinematics;
    private SwerveDriveWheelPositions lastWheelPositions;
    private Rotation2d lastGyroAngle;
    private boolean isFirstOdometryUpdate;

    private PoseEstimator6328() {
        odometryPose = new Pose2d();
        estimatedPose = new Pose2d();
        poseBuffer = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
        qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
        lastWheelPositions =
                new SwerveDriveWheelPositions(
                        new SwerveModulePosition[]{
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition()
                        }
                );
        lastGyroAngle = new Rotation2d();
        isFirstOdometryUpdate = true;
        kinematics = SwerveConstants.KINEMATICS;

        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS.get(i, 0), 2));
        }
    }

    public void addOdometryObservation(OdometryObservation observation) {
        setInitialValuesAtStart();

        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.getWheelPositions());
        lastWheelPositions = observation.getWheelPositions();

        boolean isGyroConnected = observation.getGyroAngle() != null;

        if (isGyroConnected) {
            updateDeltaTheta(twist,observation);
        }
        odometryPose = odometryPose.exp(twist);
        poseBuffer.addSample(observation.getTimestamp(), odometryPose);
        estimatedPose = estimatedPose.exp(twist);
    }

    private void setInitialValuesAtStart() {
        if (isFirstOdometryUpdate) {
            lastWheelPositions = SWERVE.getSwerveWheelPositions(0);
            lastGyroAngle = SWERVE.getOdometryYawUpdates()[0];
            isFirstOdometryUpdate = false;
        }
    }

    private Twist2d updateDeltaTheta(Twist2d twist, OdometryObservation observation) {
        lastGyroAngle = observation.getGyroAngle();
        return new Twist2d(twist.dx, twist.dy, observation.getGyroAngle().minus(lastGyroAngle).getRadians());
    }

    public void addVisionObservation(VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS
                    > observation.getTimestamp()) {
                return;
            }
        }
        catch (NoSuchElementException ex) {
            return;
        }
        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(observation.getTimestamp());
        if (sample.isEmpty()) {
            // exit if not there
            return;
        }

        // sample --> odometryPose transform and backwards of that
        Transform2d sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = observation.getStdDevs().get(i, 0) * observation.getStdDevs().get(i, 0);
        }
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            }
            else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }
        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.getVisionPose());
        // scale transform by visionK
        var kTimesTransform = visionK.times(
                VecBuilder.fill(transform.getX(), transform.getY(), transform.getRotation().getRadians())
        );
        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0))
        );

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    /**
     * Reset estimated pose and odometry pose to pose <br>
     * Clear pose buffer
     */
    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        lastGyroAngle = initialPose.getRotation();
        odometryPose = initialPose;
        poseBuffer.clear();
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    @AutoLogOutput(key = "Poses/Robot/OdometryPose")
    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public void setOdometryStandardDeviations(double x, double y, double rotation) {
        Vector<N3> newQStdDevs = VecBuilder.fill(x, y, rotation);
        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(newQStdDevs.get(i, 0), 2));
        }
    }

}

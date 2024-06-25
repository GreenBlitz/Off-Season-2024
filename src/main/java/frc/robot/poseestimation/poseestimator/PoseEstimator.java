package frc.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.poseestimation.OdometryObservation;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;

public class PoseEstimator implements AutoCloseable {

    private final Field2d field;
    private final PoseEstimator6328 swerveDrivePoseEstimator;
    private Pose2d robotPose;

    public PoseEstimator() {
        this.field = new Field2d();
        this.swerveDrivePoseEstimator = PoseEstimator6328.getInstance();
        this.robotPose = PoseEstimatorConstants.DEFAULT_POSE;
        resetPose(robotPose);

        SmartDashboard.putData("Field", field);
        setLoggingPathToPaths();
    }

    private void setLoggingPathToPaths() {
        PathPlannerLogging.setLogActivePathCallback((pose) -> {
            field.getObject("path").setPoses(pose);
            //todo - move to swerve
            Logger.recordOutput(SwerveConstants.SWERVE_LOG_PATH + "Current Path To Follow", pose.toArray(new Pose2d[0]));
        });
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        robotPose = swerveDrivePoseEstimator.getEstimatedPose();

        logCurrentPose();
        field.setRobotPose(getCurrentPose());
    }

    private void logCurrentPose() {
        Logger.recordOutput(PoseEstimatorConstants.POSE_LOG_PATH, getCurrentPose());
    }

    public void resetPose(Pose2d currentPose) {
        SWERVE.setHeading(currentPose.getRotation());
        swerveDrivePoseEstimator.resetPose(currentPose);
    }

    public void resetHeading(Rotation2d targetAngle) {
        resetPose(new Pose2d(getCurrentPose().getTranslation(), targetAngle));
    }

    public Pose2d getCurrentPose() {
        return robotPose;
    }

    public void updatePoseEstimatorOdometry() {
        int odometryUpdates = SWERVE.getOdometryTimeStepQueue().length;
        SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = SWERVE.getSwerveWheelPositions(i);
            gyroRotations[i] = SWERVE.getOdometryYawUpdates()[i];
        }

        POSE_ESTIMATOR.updatePoseEstimatorStates(swerveWheelPositions, gyroRotations, SWERVE.getOdometryTimeStepQueue());
    }

    private void updatePoseEstimatorStates(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++) {
            swerveDrivePoseEstimator.addOdometryObservation(new OdometryObservation(
                    swerveWheelPositions[i],
                    gyroRotations[i],
                    timestamps[i]
            ));
        }
    }

}

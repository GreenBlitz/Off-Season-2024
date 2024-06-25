package frc.robot.poseestimation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;

public class OdometryObservation {

    private SwerveDriveWheelPositions wheelPositions;

    private Rotation2d gyroAngle;

    private double timestamp;

    public OdometryObservation(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {
        this.wheelPositions = wheelPositions;
        this.gyroAngle = gyroAngle;
        this.timestamp = timestamp;
    }

    public SwerveDriveWheelPositions getWheelPositions() {
        return wheelPositions;
    }

    public void setWheelPositions(SwerveDriveWheelPositions wheelPositions) {
        this.wheelPositions = wheelPositions;
    }

    public Rotation2d getGyroAngle() {
        return gyroAngle;
    }

    public void setGyroAngle(Rotation2d gyroAngle) {
        this.gyroAngle = gyroAngle;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(double timestamp) {
        this.timestamp = timestamp;
    }

}

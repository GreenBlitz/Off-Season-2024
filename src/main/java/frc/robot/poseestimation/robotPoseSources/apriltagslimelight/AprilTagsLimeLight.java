package frc.robot.poseestimation.robotPoseSources.apriltagslimelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.poseestimation.robotPoseSources.IPoseSource;
import frc.robot.poseestimation.robotPoseSources.PoseSourceInputsAutoLogged;

public class AprilTagsLimeLight implements IPoseSource {

    private String hostname;
    private NetworkTableEntry robotPoseEntry, idEntry, tagPoseEntry;
    public AprilTagsLimeLight(String limeLightHostname){
        this.hostname = limeLightHostname;
        String robotPoseQuery =  "botpose_wpiblue";
        robotPoseEntry = NetworkTableInstance.getDefault().getTable(hostname).getEntry(robotPoseQuery);
        tagPoseEntry = NetworkTableInstance.getDefault().getTable(hostname).getEntry("targetpose_cameraspace");
        idEntry = NetworkTableInstance.getDefault().getTable(hostname).getEntry("tid");
    }

    private boolean isTagPresent(){
        return (int) idEntry.getInteger(-1) != -1;
    }
    private Pose3d getEstimatedPositionFromPoseArray (double[] poseArray){
        return new Pose3d(
                new Translation3d(
                        poseArray[AprilTagsLimeLightConstants.LIMELIGHT_ARRAY_VALUES.X_AXIS.value],
                        poseArray[AprilTagsLimeLightConstants.LIMELIGHT_ARRAY_VALUES.Y_AXIS.value],
                        poseArray[AprilTagsLimeLightConstants.LIMELIGHT_ARRAY_VALUES.Z_AXIS.value]
                ),
                new Rotation3d(
                        poseArray[AprilTagsLimeLightConstants.LIMELIGHT_ARRAY_VALUES.ROLL_ANGLE.value],
                        poseArray[AprilTagsLimeLightConstants.LIMELIGHT_ARRAY_VALUES.PITCH_ANGLE.value],
                        poseArray[AprilTagsLimeLightConstants.LIMELIGHT_ARRAY_VALUES.YAW_ANGLE.value]
                )
        );
    }
    private double getDistance (Pose3d pose1, Pose3d pose2){
        return Math.sqrt(
                Math.pow(pose1.getX() - pose2.getX(),2) +
                Math.pow(pose1.getY() - pose2.getY(),2) +
                Math.pow(pose1.getZ() - pose2.getZ(),2)
        );
    }
    private double getDistanceFromTag(Pose3d robotPose) {
        double[] tagPoseArray = tagPoseEntry.getDoubleArray(new double[AprilTagsLimeLightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
        Pose3d tagPose = getEstimatedPositionFromPoseArray(tagPoseArray);
        return getDistance(robotPose, tagPose);
    }

    @Override
    public void updateInputs(PoseSourceInputsAutoLogged inputs) {
        inputs.hasResult = isTagPresent();
        if(inputs.hasResult){
            double[] poseArray = robotPoseEntry.getDoubleArray(new double[AprilTagsLimeLightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
            inputs.resultLatency = poseArray[AprilTagsLimeLightConstants.LIMELIGHT_ARRAY_VALUES.TOTAL_LATENCY.value] / 1000;
            inputs.robotPoseEstimation = getEstimatedPositionFromPoseArray(poseArray);
            inputs.getDistanceFromTag = getDistanceFromTag(inputs.robotPoseEstimation);
        }
    }

}

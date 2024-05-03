package frc.robot.poseestimation.robotPoseSources.apriltagslimelight;

public class AprilTagsLimeLightConstants {
    public static final int LIMELIGHT_ENTRY_ARRAY_LENGTH = 7;


    public static enum LIMELIGHT_ARRAY_VALUES {
        X_AXIS(0),
        Y_AXIS(1),
        Z_AXIS(2),
        ROLL_ANGLE(3),
        YAW_ANGLE(4),
        PITCH_ANGLE(5),
        TOTAL_LATENCY(6);

        public final int value;
        LIMELIGHT_ARRAY_VALUES(int value){
            this.value = value;
        }
    }

}

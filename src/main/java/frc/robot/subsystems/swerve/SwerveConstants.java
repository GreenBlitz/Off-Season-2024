package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.MathConstants;

public class SwerveConstants {

    public static final String SWERVE_LOG_PATH = "Swerve/";
    public static final String SWERVE_STATE_LOG_PATH = SWERVE_LOG_PATH + "Current State/";
    public static final String SWERVE_VELOCITY_LOG_PATH = SWERVE_LOG_PATH + "Velocity/";


    public static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(1);
    public static final Rotation2d ROTATION_VELOCITY_TOLERANCE = Rotation2d.fromRadians(0.05);
    public static final double TRANSLATION_TOLERANCE_METERS = 0.05;
    public static final double TRANSLATION_VELOCITY_TOLERANCE = 0.05;


    public static final double DRIVE_NEUTRAL_DEADBAND = 0.2;
    public static final double ROTATION_NEUTRAL_DEADBAND = 0.2;


    public static final double MAX_SPEED_METERS_PER_SECOND = 5.5;
    public static final Rotation2d MAX_ROTATIONAL_SPEED_PER_SECOND = Rotation2d.fromRadians(10);
    public static final double SLOW_DRIVE_MODE_FACTOR = 0.5;


    private static final double MAX_ROTATION_VELOCITY = 540;
    public static final double MAX_ROTATION_ACCELERATION = 360;
    public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ROTATION_VELOCITY,
            MAX_ROTATION_ACCELERATION
    );


    public static final double MODULE_X_DISTANCE_FROM_CENTER = 0.27833;
    public static final double MODULE_Y_DISTANCE_FROM_CENTER = 0.34733;

    public static final Translation2d FRONT_LEFT_TRANSLATION2D = new Translation2d(
            MODULE_X_DISTANCE_FROM_CENTER,
            MODULE_Y_DISTANCE_FROM_CENTER
    );
    public static final Translation2d FRONT_RIGHT_TRANSLATION2D = new Translation2d(
            MODULE_X_DISTANCE_FROM_CENTER,
            -MODULE_Y_DISTANCE_FROM_CENTER
    );
    public static final Translation2d BACK_LEFT_TRANSLATION2D = new Translation2d(
            -MODULE_X_DISTANCE_FROM_CENTER,
            MODULE_Y_DISTANCE_FROM_CENTER
    );
    public static final Translation2d BACK_RIGHT_TRANSLATION2D = new Translation2d(
            -MODULE_X_DISTANCE_FROM_CENTER,
            -MODULE_Y_DISTANCE_FROM_CENTER
    );

    public static final Translation2d[] LOCATIONS = {
            FRONT_LEFT_TRANSLATION2D,
            FRONT_RIGHT_TRANSLATION2D,
            BACK_LEFT_TRANSLATION2D,
            BACK_RIGHT_TRANSLATION2D
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);


    public static final PIDConstants PROFILED_ROTATION_PID_DEGREES_CONSTANTS = new PIDConstants(6, 0, 0);
    public static final ProfiledPIDController PROFILED_ROTATION_PID_DEGREES_CONTROLLER = new ProfiledPIDController(
            PROFILED_ROTATION_PID_DEGREES_CONSTANTS.kP,
            PROFILED_ROTATION_PID_DEGREES_CONSTANTS.kI,
            PROFILED_ROTATION_PID_DEGREES_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );

    static {
        PROFILED_ROTATION_PID_DEGREES_CONTROLLER.enableContinuousInput(
                -MathConstants.HALF_CIRCLE.getDegrees(),
                MathConstants.HALF_CIRCLE.getDegrees()
        );
    }

    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0);
    public static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    );


    public static final PIDConstants AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0);
    public static final PIDConstants AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(3, 0, 0);
    public static final double DRIVE_RADIUS_METERS = Math.hypot(MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER);
    public static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true);
    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS,
            MAX_SPEED_METERS_PER_SECOND,
            DRIVE_RADIUS_METERS,
            REPLANNING_CONFIG
    );

    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

    public static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;

}

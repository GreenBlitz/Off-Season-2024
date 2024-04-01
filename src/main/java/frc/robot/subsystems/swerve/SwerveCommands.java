package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.commands.InitExecuteCommand;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.RobotContainer.SWERVE;

public class SwerveCommands {

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed open mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(false),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    public static Command getRotateToAngleCommand(Rotation2d targetAngle) {
        return new InstantCommand(SWERVE::resetRotationController)
                .andThen(new RunCommand(() -> SWERVE.rotateToAngle(targetAngle)))
                        .until(() -> SWERVE.isAtAngle(targetAngle));
    }

    private static Command getPIDToPoseCommand(AlliancePose2d targetPose) {
        return new InstantCommand(SWERVE::resetRotationController)
                .andThen(new RunCommand(() -> SWERVE.pidToPose(targetPose.toMirroredAlliancePose()))
                        .until(() -> SWERVE.isAtPosition(targetPose.toMirroredAlliancePose())));
    }

    private static Command getPathfindToPoseCommand(AlliancePose2d targetPose, PathConstraints pathConstraints) {
        final Pose2d targetMirroredAlliancePose = targetPose.toMirroredAlliancePose();
        final Pose2d currentBluePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
        if (currentBluePose.getTranslation().getDistance(targetMirroredAlliancePose.getTranslation()) < 0.35)
            return createOnTheFlyPathCommand(targetMirroredAlliancePose, pathConstraints);
        return AutoBuilder.pathfindToPose(targetMirroredAlliancePose, pathConstraints);
    }

    public static Command getDriveToPoseCommand(Supplier<AlliancePose2d> targetPose, PathConstraints constraints) {
        return new DeferredCommand(() -> getCurrentDriveToPoseCommand(targetPose.get(), constraints), Set.of(SWERVE));
    }

    private static Command getCurrentDriveToPoseCommand(AlliancePose2d targetPose, PathConstraints constraints) {
        Logger.recordOutput("target roto pose", targetPose.toBlueAlliancePose());
        return new SequentialCommandGroup(
                new InstantCommand(() -> SWERVE.initializeDrive(true)),
                getPathfindToPoseCommand(targetPose, constraints)
                ,getPIDToPoseCommand(targetPose)
        );
    }

    private static Command createOnTheFlyPathCommand(Pose2d targetPose, PathConstraints constraints) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose(),
                targetPose
        );

        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                new GoalEndState(0, targetPose.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }
}

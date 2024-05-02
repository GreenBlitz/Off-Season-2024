package frc.robot.commands;

import frc.robot.subsystems.romidrivetrain.ROMIDriveTrain;
import frc.robot.subsystems.romidrivetrain.ROMIDriveTrainConstants;
import frc.utils.GBCommand;
import frc.utils.joysticks.SmartJoystick;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ROMIController extends GBCommand {
    SmartJoystick joystick;
    ROMIDriveTrain romi;

    public ROMIController (SmartJoystick joystick, ROMIDriveTrain romiDriveTrain){
        this.joystick = joystick;
        this.romi = romiDriveTrain;
    }

    @Override
    public void execute() {
        romi.arcadeDrive(
                joystick.getAxisValue(SmartJoystick.Axis.LEFT_Y) * ROMIDriveTrainConstants.MAX_VELOCITY.in(MetersPerSecond),
                joystick.getAxisValue(SmartJoystick.Axis.RIGHT_X) * ROMIDriveTrainConstants.MAX_ROTATIONAL_SPEED.in(RotationsPerSecond)
        );
    }

}

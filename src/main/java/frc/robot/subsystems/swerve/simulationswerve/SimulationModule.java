package frc.robot.subsystems.swerve.simulationswerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.ModuleConstants;
import frc.robot.subsystems.swerve.ModuleUtils;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class SimulationModule implements IModule {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    private final ModuleUtils.ModuleName moduleName;

    private final PositionVoltage steerPositionRequest = new PositionVoltage(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);

    public SimulationModule(ModuleUtils.ModuleName moduleName){
        this.moduleName = moduleName;
        this.steerMotor = new SimpleMotorSimulation(
                SimulationModuleConstants.STEER_MOTOR_GEARBOX,
                ModuleConstants.STEER_GEAR_RATIO,
                SimulationModuleConstants.STEER_MOMENT_OF_INERTIA
        );
        this.driveMotor = new SimpleMotorSimulation(
                SimulationModuleConstants.DRIVE_MOTOR_GEARBOX,
                ModuleConstants.DRIVE_GEAR_RATIO,
                SimulationModuleConstants.DRIVE_MOMENT_OF_INERTIA
        );

        configureDriveMotor();
        configureSteerMotor();
    }


    private void configureDriveMotor() {
        driveMotor.applyConfiguration(new TalonFXConfiguration());
    }

    private void configureSteerMotor() {
        steerMotor.applyConfiguration(SimulationModuleConstants.STEER_MOTOR_CONFIG);
    }

    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                ModuleConstants.WHEEL_DIAMETER_METERS,
                steerMotor.getVelocityRevolutionsPerSecond(),
                0,
                ModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "driveVoltage", driveMotor.getVoltage());
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }
    
    @Override
    public void resetByEncoder() {
    
    }
    
    @Override
    public void stop() {
        driveMotor.stop();
        steerMotor.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "driveVoltage", driveMotor.getVoltage());
    }

    @Override
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = Conversions.revolutionsToDegrees(steerMotor.getPositionRevolutions());
        inputs.odometryUpdatesSteerAngleDegrees = new double[]{inputs.steerAngleDegrees};
        inputs.steerVoltage = steerMotor.getVoltage();

        inputs.driveDistanceMeters = Conversions.revolutionsToDistance(driveMotor.getPositionRevolutions(), ModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.odometryUpdatesDriveDistanceMeters = new double[]{inputs.driveDistanceMeters};
        inputs.driveVelocityMetersPerSecond = Conversions.revolutionsToDistance(driveMotor.getVelocityRevolutionsPerSecond(), ModuleConstants.WHEEL_DIAMETER_METERS);
        inputs.driveCurrent = driveMotor.getCurrent();
        inputs.driveVoltage = driveMotor.getVoltage();
    }
}
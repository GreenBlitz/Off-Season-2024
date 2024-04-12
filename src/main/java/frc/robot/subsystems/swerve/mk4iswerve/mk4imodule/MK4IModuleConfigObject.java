package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.devicewrappers.GBTalonFXPro;

class MK4IModuleConfigObject {

    private final MK4IModuleRecords.MK4IModuleMotors moduleMotors;
    private final GBTalonFXPro steerMotor, driveMotor;
    private final CANcoder steerEncoder;
    private final MK4IModuleRecords.MK4IModuleSignals moduleSignals;

    protected MK4IModuleConfigObject(
            String busChain,
            int steerMotorId,
            boolean isSteerMotorInverted,
            int driveMotorId,
            boolean isDriveMotorInverted,
            int steerEncoderId) {
        this.steerEncoder = new CANcoder(steerEncoderId, busChain);
        this.moduleMotors = new MK4IModuleRecords.MK4IModuleMotors(
                new GBTalonFXPro(driveMotorId, busChain), new GBTalonFXPro(steerMotorId, busChain));
        this.steerMotor = moduleMotors.steerMotor();
        this.driveMotor = moduleMotors.driveMotor();
        this.moduleSignals = new MK4IModuleRecords.MK4IModuleSignals(
                steerEncoder.getAbsolutePosition(),
                steerEncoder.getVelocity(),
                steerEncoder.getSupplyVoltage(),
                driveMotor.getPosition(),
                driveMotor.getVelocity(),
                driveMotor.getAcceleration(),
                driveMotor.getMotorVoltage(),
                driveMotor.getStatorCurrent(),
                steerMotor.getPosition(),
                steerMotor.getVelocity(),
                steerMotor.getAcceleration(),
                steerMotor.getMotorVoltage());

        configEncoder();
        optimizeBusAndSignalOfEncoder();

        configDriveMotor();
        driveMotor.setInverted(isDriveMotorInverted);
        optimizeBusAndSignalOfDriveMotor();

        configSteerMotor();
        steerMotor.setInverted(isSteerMotorInverted);
        optimizeBusAndSignalOfSteerMotor();
    }

    private void configEncoder() {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        steerEncoder.getConfigurator().refresh(magnetSensorConfigs);
        MK4IModuleConstants.ENCODER_CONFIG.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
        steerEncoder.getConfigurator().apply(MK4IModuleConstants.ENCODER_CONFIG);
    }

    private void optimizeBusAndSignalOfEncoder() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ, moduleSignals.steerEncoderAbsolutePositionSignal());
        BaseStatusSignal.setUpdateFrequencyForAll(
                100, moduleSignals.steerEncoderVelocitySignal(), moduleSignals.steerEncoderVoltageSignal());

        steerEncoder.optimizeBusUtilization();
    }

    private void configDriveMotor() {
        driveMotor.applyConfiguration(MK4IModuleConstants.DRIVE_MOTOR_CONFIG);
    }

    private void optimizeBusAndSignalOfDriveMotor() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                moduleSignals.drivePositionSignal(),
                moduleSignals.driveVelocitySignal(),
                moduleSignals.driveAccelerationSignal());
        BaseStatusSignal.setUpdateFrequencyForAll(
                100, moduleSignals.driveVoltageSignal(), moduleSignals.driveStatorCurrentSignal());

        driveMotor.optimizeBusUtilization();
    }

    private void configSteerMotor() {
        TalonFXConfiguration configuration = MK4IModuleConstants.STEER_MOTOR_CONFIG;
        configuration.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();
        steerMotor.applyConfiguration(configuration);
    }

    private void optimizeBusAndSignalOfSteerMotor() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                moduleSignals.steerPositionSignal(),
                moduleSignals.steerVelocitySignal(),
                moduleSignals.steerAccelerationSignal());
        BaseStatusSignal.setUpdateFrequencyForAll(20, moduleSignals.steerVoltageSignal());

        steerMotor.optimizeBusUtilization();
    }

    public CANcoder getSteerEncoder() {
        return steerEncoder;
    }

    public MK4IModuleRecords.MK4IModuleMotors getMotors() {
        return moduleMotors;
    }

    public MK4IModuleRecords.MK4IModuleSignals getModuleSignals() {
        return moduleSignals;
    }
}

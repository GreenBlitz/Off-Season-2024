package frc.robot.subsystems.Gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon2GyroConfigObject {
    private Pigeon2 gyro;

    public StatusSignal<Double> YAW_SIGNAL, PITCH_SIGNAL, X_ACCELERATION_SIGNAL, Y_ACCELERATION_SIGNAL, Z_ACCELERATION_SIGNAL;

    public Pigeon2GyroConfigObject(int id, String busChain){
        gyro = new Pigeon2(id, busChain);
        configGyro();
        optimizeBusAndSignalOfGyro();
    }

    public Pigeon2 getGyro() {
        return gyro;
    }

    public void configGyro(){
        gyro.getConfigurator().apply(GyroConstants.PIGEON_2_CONFIGURATION);
    }

    private void optimizeBusAndSignalOfGyro() {
        YAW_SIGNAL = gyro.getYaw();
        PITCH_SIGNAL = gyro.getPitch();
        X_ACCELERATION_SIGNAL = gyro.getAccelerationX();
        Y_ACCELERATION_SIGNAL = gyro.getAccelerationY();
        Z_ACCELERATION_SIGNAL = gyro.getAccelerationZ();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                X_ACCELERATION_SIGNAL,
                Y_ACCELERATION_SIGNAL,
                Z_ACCELERATION_SIGNAL
        );

        PITCH_SIGNAL.setUpdateFrequency(100);
        YAW_SIGNAL.setUpdateFrequency(250);//PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ

        gyro.optimizeBusUtilization();
    }
}

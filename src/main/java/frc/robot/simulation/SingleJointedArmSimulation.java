package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.utils.roborioutils.RoborioUtils;

public class SingleJointedArmSimulation extends MotorSimulation implements Mechanism2dUser {

    private final SingleJointedArmSim armSimulation;

    private final MechanismLigament2d armLigament2d;

    public SingleJointedArmSimulation(DCMotor gearbox, double gearRatio, double armLengthMeters, double armMassKilograms,
            Rotation2d minimumAngle, Rotation2d maximumAngle, Rotation2d startingAngle, boolean simulateGravity) {
        this.armSimulation = new SingleJointedArmSim(
                gearbox,
                gearRatio,
                SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKilograms),
                armLengthMeters,
                minimumAngle.getRadians(),
                maximumAngle.getRadians(),
                simulateGravity,
                startingAngle.getRadians()
        );
        armLigament2d = new MechanismLigament2d("Arm", armLengthMeters, startingAngle.getDegrees());
    }

    @Override
    public double getCurrent() {
        return armSimulation.getCurrentDrawAmps();
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRadians(armSimulation.getAngleRads());
    }

    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(armSimulation.getVelocityRadPerSec());
    }

    @Override
    protected void setInputVoltage(double voltage) {
        armSimulation.setInputVoltage(voltage);
    }

    @Override
    protected void updateMotor() {
        armSimulation.update(RoborioUtils.getCurrentRoborioCycleTime());
        armLigament2d.setAngle(getPosition());
    }

    @Override
    public MechanismLigament2d getLigament() {
        return armLigament2d;
    }

    @Override
    public MechanismLigament2d append(Mechanism2dUser mechanism2dUser) {
        return append(mechanism2dUser.getLigament());
    }

    @Override
    public MechanismLigament2d append(MechanismLigament2d ligament) {
        return armLigament2d.append(ligament);
    }

}

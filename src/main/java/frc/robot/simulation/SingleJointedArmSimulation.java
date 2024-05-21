package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.roborioutils.RoborioUtils;

public class SingleJointedArmSimulation extends MotorSimulation {

    private final SingleJointedArmSim armSimulation;

    private final Mechanism2d mechanism2d;

    private final MechanismRoot2d root2d;

    private final MechanismLigament2d arm2d;

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
        mechanism2d = new Mechanism2d(2 * armLengthMeters, armLengthMeters);
        root2d = mechanism2d.getRoot("Pivot", armLengthMeters, 0);
        arm2d = root2d.append(new MechanismLigament2d("Arm", armLengthMeters, startingAngle.getDegrees()));
        SmartDashboard.putData("ArmMechanism2d", mechanism2d);
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
        arm2d.setAngle(getPosition());
    }

}

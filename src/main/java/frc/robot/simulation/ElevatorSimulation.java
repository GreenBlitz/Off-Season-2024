package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.utils.Conversions;
import frc.utils.roborioutils.RoborioUtils;

public class ElevatorSimulation extends MotorSimulation implements Mechanism2dUser {

    private final ElevatorSim elevatorSimulation;

    private final double diameterMeters;

    private final MechanismLigament2d elevatorLigament2d;

    public ElevatorSimulation(DCMotor gearbox, double gearRatio, double carriageMassKilograms, double drumRadiusMeters,
            double minimumHeightMeters, double maximumHeightMeters, double startingHeightMeters, boolean simulateGravity) {
        this.diameterMeters = 2 * drumRadiusMeters;
        this.elevatorSimulation = new ElevatorSim(
                gearbox,
                gearRatio,
                carriageMassKilograms,
                drumRadiusMeters,
                minimumHeightMeters,
                maximumHeightMeters,
                simulateGravity,
                startingHeightMeters
        );
        elevatorLigament2d = new MechanismLigament2d("Elevator", startingHeightMeters, 90);
    }

    @Override
    public double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    /**
     * Returns in Rotation2D the position of the drum
     *
     * @return the position
     */
    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(
                Conversions.distanceToRevolutions(
                        getPositionMeters(),
                        diameterMeters
                )
        );
    }

    public double getPositionMeters() {
        return elevatorSimulation.getPositionMeters();
    }

    /**
     * Returns the velocity in Rotation2D of the drum
     *
     * @return the velocity
     */
    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(
                Conversions.distanceToRevolutions(
                        getVelocityMetersPerSecond(),
                        diameterMeters
                )
        );
    }

    public double getVelocityMetersPerSecond() {
        return elevatorSimulation.getVelocityMetersPerSecond();
    }

    @Override
    protected void setInputVoltage(double voltage) {
        elevatorSimulation.setInputVoltage(voltage);
    }

    @Override
    protected void updateMotor() {
        elevatorSimulation.update(RoborioUtils.getCurrentRoborioCycleTime());
        elevatorLigament2d.setLength(getPositionMeters());
    }

    @Override
    public MechanismLigament2d getLigament() {
        return elevatorLigament2d;
    }

    @Override
    public MechanismLigament2d append(Mechanism2dUser mechanism2dUser) {
        return append(mechanism2dUser.getLigament());
    }

    @Override
    public MechanismLigament2d append(MechanismLigament2d ligament) {
        return elevatorLigament2d.append(ligament);
    }

}

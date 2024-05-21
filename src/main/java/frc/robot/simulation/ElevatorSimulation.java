package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.Conversions;
import frc.utils.roborioutils.RoborioUtils;

public class ElevatorSimulation extends MotorSimulation {

    private final ElevatorSim elevatorSimulation;

    private final double diameterMeters;

    private final Mechanism2d mechanism2d;

    private final MechanismRoot2d root2d;

    private final MechanismLigament2d elevator2d;

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
        mechanism2d = new Mechanism2d(maximumHeightMeters, maximumHeightMeters);
        root2d = mechanism2d.getRoot("ElevatorBase", maximumHeightMeters/2, 0);
        elevator2d = root2d.append(new MechanismLigament2d("Elevator", startingHeightMeters, 90));
        SmartDashboard.putData("ElevatorMechanism2d", mechanism2d);
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
        elevator2d.setLength(getPositionMeters());
    }

}

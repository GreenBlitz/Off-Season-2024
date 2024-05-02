package frc.robot.subsystems.romidrivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class ROMIDriveTrainConstants {
    public static final double COUNTS_PER_REVOLUTION = 1440.0;
    public static final Measure<Distance> WHEEL_DIAMETER = Millimeters.of(70);
    public static final Measure<Velocity<Distance>> MAX_VELOCITY = MetersPerSecond.of(1);
    public static final Measure<Velocity<Angle>> MAX_ROTATIONAL_SPEED = RotationsPerSecond.of(1);
}

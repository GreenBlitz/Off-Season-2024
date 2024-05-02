package frc.robot.subsystems.romidrivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

public class ROMIDriveTrainConstants {
    public static final double COUNTS_PER_REVOLUTION = 1440.0;
    public static final Measure<Distance> WHEEL_DIAMETER = Millimeters.of(70);

    public static void main(String[] args) {
        System.out.println(WHEEL_DIAMETER.in(Inches));
    }
}

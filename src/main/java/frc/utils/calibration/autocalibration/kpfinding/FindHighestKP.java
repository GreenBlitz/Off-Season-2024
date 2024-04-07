package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.utils.GBSubsystem;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindHighestKP extends SequentialCommandGroup {

    /**
     * @author Yoav Herman
     * This function is finding for you what is the best (biggest) kP value to use on your system. <p>
     *
     * IMPORTANT:
     * @apiNote You can choose what units to use or which control mode you calibrate. BUT Make sure you use the SAME UNITS EVERYWHERE!!!
     * @apiNote This function must get RELATIVE values and NOT ABSOLUTE. For EXAMPLE robot angle should NOT be from (-180,180) but RELATIVE!!! <P>
     *
     * RECOMMEND:
     * @apiNote The system should be in one of the positions of "valuesToRunFor" when you begin the run!!!
     *
     * @param isSetControlNeedToRunPeriodic -> is the setControl function run on the motor or on the rio
     * @param tolerance -> wanted tolerance
     * @param timeoutForActionSeconds -> max time for action
     * @param errorToKpValueFactor -> how many kP to add or minus for one error
     * @param multiPFactor -> by how much increase kP until there is Oscillation
     * @param valuesToRunFor -> two points of the system to jump between
     * @param currentValueSupplier -> supplier to the current position of the system
     * @param currentKpValueSupplier -> supplier to current kP value
     * @param setControl -> function to control the system
     * @param setKp -> function to set new kP
     * @param isAtPose -> function to check is system at position
     * @param doOnEnd -> function to do on end (probably stand in place)
     */

    public FindHighestKP(
            GBSubsystem subsystem,
            boolean isSetControlNeedToRunPeriodic,
            double tolerance, double timeoutForActionSeconds, double errorToKpValueFactor, double multiPFactor,
            Pair<Double, Double> valuesToRunFor,
            DoubleSupplier currentValueSupplier, DoubleSupplier currentKpValueSupplier,
            Consumer<Double> setControl, Consumer<Double> setKp,
            Predicate<Double> isAtPose,
            Runnable doOnEnd
    ) {
        super(
                new FindKP(
                        subsystem,
                        isSetControlNeedToRunPeriodic,
                        tolerance, timeoutForActionSeconds, errorToKpValueFactor,
                        valuesToRunFor,
                        currentValueSupplier, currentKpValueSupplier,
                        setControl, setKp,
                        isAtPose,
                        doOnEnd
                ),
                new FindHighestKPBeforeOscillate(
                        subsystem,
                        isSetControlNeedToRunPeriodic,
                        tolerance, timeoutForActionSeconds, multiPFactor,
                        valuesToRunFor,
                        currentValueSupplier, currentKpValueSupplier,
                        setControl, setKp,
                        isAtPose,
                        doOnEnd
                )
        );
    }

}

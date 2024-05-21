package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public interface MechanismUser2d {

    MechanismLigament2d getLigament();

    MechanismLigament2d append(MechanismUser2d mechanismUser2d);

    MechanismLigament2d append(MechanismLigament2d ligament);
}

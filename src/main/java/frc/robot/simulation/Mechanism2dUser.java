package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public interface Mechanism2dUser {

    MechanismLigament2d getLigament();

    MechanismLigament2d append(Mechanism2dUser mechanism2dUser);

    MechanismLigament2d append(MechanismLigament2d ligament);
}

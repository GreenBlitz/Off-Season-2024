package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MechanismInstance2d {

    private final Mechanism2d mechanism2d;

    private final MechanismRoot2d root;

    private MechanismLigament2d lastLigament = null;

    public MechanismInstance2d(String name, double mechanismWidth, double mechanismHeight, double rootX, double rootY, MechanismUser2d... users) {
        mechanism2d = new Mechanism2d(mechanismWidth, mechanismHeight);
        root = mechanism2d.getRoot(name+"Root", rootX, rootY);
        for (MechanismUser2d user : users) {
            if (lastLigament == null)
                lastLigament = root.append(user.getLigament());
            else
                lastLigament = lastLigament.append(user.getLigament());
        }
        SmartDashboard.putData(name, mechanism2d);
    }
}

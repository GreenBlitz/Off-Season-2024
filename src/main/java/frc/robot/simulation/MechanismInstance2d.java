package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MechanismInstance2d {

    private final Mechanism2d mechanism2d;

    private final MechanismRoot2d root;

    private MechanismLigament2d lastLigament = null;

    public static MechanismLigament2d[] convertToLigaments(MechanismUser2d... users) {
        MechanismLigament2d[] ligaments = new MechanismLigament2d[users.length];
        for (int i = 0; i < users.length; i++) {
            ligaments[i] = users[i].getLigament();
        }
        return ligaments;
    }

    public MechanismInstance2d(String name, double mechanismWidth, double mechanismHeight, double rootX, double rootY, MechanismUser2d... users) {
        this(name, mechanismWidth, mechanismHeight, rootX, rootY, convertToLigaments(users));
    }

    public MechanismInstance2d(String name, double mechanismWidth, double mechanismHeight, double rootX, double rootY, MechanismLigament2d... ligaments) {
        mechanism2d = new Mechanism2d(mechanismWidth, mechanismHeight);
        root = mechanism2d.getRoot(name + "Root", rootX, rootY);
        append(ligaments);
        SmartDashboard.putData(name, mechanism2d);
    }

    public void append(MechanismUser2d... users) {
        append(convertToLigaments(users));
    }

    public void append(MechanismLigament2d... ligaments) {
        for (MechanismLigament2d ligament : ligaments) {
            if (lastLigament == null) {
                lastLigament = root.append(ligament);
            }
            else {
                lastLigament = lastLigament.append(ligament);
            }
        }
    }

}

package frc.robot.poseestimation.robotPoseSources;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PoseSourceInputs {
    public boolean hasResult;
    public Pose3d robotPoseEstimation;
    public double getDistanceFromTag;
    public double resultLatency;
}

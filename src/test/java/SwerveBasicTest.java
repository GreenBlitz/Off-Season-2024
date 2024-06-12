import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveBasicTest {

    Swerve swerve;

    @BeforeEach
    void setup() {
        this.swerve = RobotContainer.SWERVE;
        swerve.stop();
    }

    @Test
    void shouldBeIdle() {
        swerve.stop();
        assertEquals(
                0,
                swerve.getSelfRelativeVelocity().vxMetersPerSecond + swerve.getSelfRelativeVelocity().vyMetersPerSecond
                        + swerve.getSelfRelativeVelocity().omegaRadiansPerSecond
        );
    }

}

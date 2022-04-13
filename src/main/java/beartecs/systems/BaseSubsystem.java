package beartecs.systems;

import beartecs.logger.RobotLogger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class BaseSubsystem extends SubsystemBase {
  public enum RobotMode {
    DISABLED, AUTO, TELEOP
  }

  protected final RobotLogger m_logger = RobotContainer.getLogger();

  public boolean isSim() {
    return Robot.isSimulation();
  }

  public NetworkTable getNetworkTable() {
    return NetworkTableInstance.getDefault().getTable("SmartDashboard");
  }

  public ShuffleboardTab addTab(String tabName) {
    // Switch between tabs for merging or allowing independent tabs
    return Shuffleboard.getTab("Window");
    // return Shuffleboard.getTab(tabName);
  }

  public RobotMode getRobotMode() {
    if (DriverStation.isAutonomousEnabled())
      return RobotMode.AUTO;
    if (DriverStation.isTeleopEnabled())
      return RobotMode.TELEOP;

    return RobotMode.DISABLED;
  }
}

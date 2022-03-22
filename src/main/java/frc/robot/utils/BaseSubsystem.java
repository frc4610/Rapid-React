package frc.robot.utils;

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

  public void onLEDCallback(int reservedStatus) {
    RobotContainer.getLEDSubsystem().setStatus(isOkay(), reservedStatus);
  }

  public boolean isOkay() {
    return true;
  }

  public boolean IsSim() {
    return Robot.isSimulation();
  }

  // TODO: use this to put items on shuffleboard with the ability to add tabs or layouts
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

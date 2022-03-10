package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public RobotMode getRobotMode() {
    if (DriverStation.isAutonomousEnabled())
      return RobotMode.AUTO;
    if (DriverStation.isTeleopEnabled())
      return RobotMode.TELEOP;

    return RobotMode.DISABLED;
  }
}

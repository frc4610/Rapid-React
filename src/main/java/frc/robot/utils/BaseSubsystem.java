package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class BaseSubsystem extends SubsystemBase {

  public void onLEDCallback(int reservedStatus) {
    RobotContainer.getLEDSubsystem().setStatus(isOkay(), reservedStatus);
  }

  public boolean isOkay() {
    return true;
  }
}

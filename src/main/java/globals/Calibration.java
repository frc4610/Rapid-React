package globals;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Calibration {
  private static final ShuffleboardTab m_tab = Shuffleboard.getTab("Calibration");

  public static final NetworkTableEntry CALIBRATION_MODE = m_tab.add("Calibration Mode", false).getEntry();
  public static final NetworkTableEntry ZERO_WHEELS = m_tab.add("Zero Wheels", false).getEntry();
}

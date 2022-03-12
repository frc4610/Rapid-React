package frc.robot.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingPattern implements AddressableLEDPattern {
  private AddressableLEDPattern m_onPattern;
  private AddressableLEDPattern m_offPattern;
  private double m_interval;
  private boolean on = true;
  private double lastChange;

  /**
   * 
   * @param onColor color for when the blink is on.
   * @param inteval time in seconds between changes.
   */
  public BlinkingPattern(Color onColor, double interval) {
    super();
    m_onPattern = new SolidColorPattern(onColor);
    m_offPattern = new SolidColorPattern(Color.kBlack);
    m_interval = interval;
  }

  @Override
  public void setLEDs(AddressableLEDBuffer buffer) {
    double timestamp = Timer.getFPGATimestamp();
    if (timestamp - lastChange > m_interval) {
      on = !on;
      lastChange = timestamp;
    }
    if (on) {
      m_onPattern.setLEDs(buffer);
    } else {
      m_offPattern.setLEDs(buffer);
    }

  }

  public boolean isAnimated() {
    return true;
  }
}

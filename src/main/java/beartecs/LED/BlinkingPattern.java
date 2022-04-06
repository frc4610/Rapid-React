package beartecs.LED;

import beartecs.math.MathUtils;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingPattern implements AddressableLEDPattern {
  private IntensityPattern m_onPattern;
  private double m_interval;
  private boolean on = true;
  private double m_lastChange;

  /**
   * 
   * @param onColor color for when the blink is on.
   * @param inteval time in seconds between changes.
   */
  public BlinkingPattern(Color onColor, double interval) {
    super();
    m_onPattern = new IntensityPattern(onColor, 1.0);
    m_interval = interval;
  }

  @Override
  public void setLEDs(AddressableLEDBuffer buffer) {
    double timestamp = Timer.getFPGATimestamp();
    if (timestamp - m_lastChange > m_interval) {
      on = !on;
      m_lastChange = timestamp;
    }
    double interp = MathUtils.clamp((timestamp - m_lastChange) / m_interval, 0.0, 1.0);
    m_onPattern.setIntensity(interp);
    m_onPattern.setLEDs(buffer);
  }

  public boolean isAnimated() {
    return true;
  }
}

package globals.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import globals.utils.math.MathUtils;

public class TimerPattern implements AddressableLEDPattern {
  private Color m_activeColor;
  private double m_timeSeconds;
  private double m_startTime;

  public TimerPattern(Color active, double timeSeconds) {
    m_activeColor = active;
    m_timeSeconds = timeSeconds;
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void setLEDs(AddressableLEDBuffer buffer) {
    double timestamp = Timer.getFPGATimestamp();
    double interp = MathUtils.clamp((timestamp - m_startTime) / m_timeSeconds, 0.0, 1.0);
    if (interp > 0.0) {
      for (int index = 0; index < buffer.getLength(); index++) {
        if (index < MathUtils.lerp(buffer.getLength(), 0, interp))
          buffer.setLED(index, m_activeColor);
        else {
          buffer.setLED(index, Color.kBlack);
        }
      }
    }
  }

  @Override
  public boolean isAnimated() {
    return true;
  }
}

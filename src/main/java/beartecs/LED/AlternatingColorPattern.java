package beartecs.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class AlternatingColorPattern implements AddressableLEDPattern {
  private Color[] m_colors;
  private double m_interval = 0.1;
  private double m_lastChange = 0.0;
  private boolean on = false;

  public AlternatingColorPattern(Color[] colors) {
    super();
    this.m_colors = colors;
  }

  public AlternatingColorPattern(Color[] colors, double interval) {
    super();
    this.m_colors = colors;
    m_interval = interval;
  }

  @Override
  public void setLEDs(AddressableLEDBuffer buffer) {
    double timestamp = Timer.getFPGATimestamp();
    if (timestamp - m_lastChange > m_interval) {
      on = !on;
      m_lastChange = timestamp;
    }
    for (int index = 0; index < buffer.getLength() / 2; index++) {
      buffer.setLED(index, m_colors[(on ? index + 1 : index) % m_colors.length]);
      buffer.setLED(buffer.getLength() - index - 1, m_colors[(on ? index : index + 1) % m_colors.length]);
    }
  }
}
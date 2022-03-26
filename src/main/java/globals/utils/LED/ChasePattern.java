package globals.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class ChasePattern implements AddressableLEDPattern {
  private Color[] m_Colors;
  private int m_SegmentWidth;
  private int m_offset;

  public ChasePattern(Color[] colors, int segmentWidth) {
    super();
    m_Colors = colors;
    m_SegmentWidth = segmentWidth;
  }

  @Override
  public void setLEDs(AddressableLEDBuffer buffer) {
    int numberOfColors = m_Colors.length;
    int effectiveIndex;
    int colorIndex;
    int bufferLength = buffer.getLength();// / 2;

    for (int index = 0; index < bufferLength; index++) {
      effectiveIndex = (index + m_offset) % bufferLength;
      colorIndex = (index / m_SegmentWidth) % numberOfColors;
      buffer.setLED(effectiveIndex, m_Colors[colorIndex]);
      // buffer.setLED(buffer.getLength() - effectiveIndex - 1, m_Colors[colorIndex]);
    }

    m_offset = (m_offset + 1) % bufferLength;
  }

  public boolean isAnimated() {
    return true;
  }
}

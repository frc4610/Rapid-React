package globals.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RainbowPattern implements AddressableLEDPattern {
  private int m_firstHue = 0;

  public RainbowPattern() {
    super();

  }

  @Override
  public void setLEDs(AddressableLEDBuffer buffer) {
    int currentHue;
    for (int index = 0; index < buffer.getLength() / 2; index++) {
      currentHue = (m_firstHue + (index * 180 / buffer.getLength())) % 180;
      buffer.setHSV(index, currentHue, 255, 255);
      buffer.setHSV(buffer.getLength() - index - 1, currentHue, 255, 255);
    }

    m_firstHue = (m_firstHue + 3) % 180;
  }

  public boolean isAnimated() {
    return true;
  }
}

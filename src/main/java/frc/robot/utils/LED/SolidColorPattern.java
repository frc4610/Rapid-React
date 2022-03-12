package frc.robot.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class SolidColorPattern implements AddressableLEDPattern {
  private Color m_color;

  public SolidColorPattern(Color aColor) {
    super();
    this.m_color = aColor;
  }

  @Override
  public void setLEDs(AddressableLEDBuffer buffer) {

    for (int index = 0; index < buffer.getLength() / 2; index++) {
      buffer.setLED(index, m_color);
      buffer.setLED(buffer.getLength() - index - 1, m_color);
    }

  }

}

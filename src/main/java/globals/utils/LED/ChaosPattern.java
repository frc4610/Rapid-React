package globals.utils.LED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class ChaosPattern implements AddressableLEDPattern {
  private boolean m_firstTime = true;

  public ChaosPattern() {
    super();

  }

  @Override
  public void setLEDs(AddressableLEDBuffer buffer) {
    if (m_firstTime) {
      for (int index = 0; index < buffer.getLength() / 2; index++) {
        buffer.setLED(index, new Color(Math.random(), Math.random(), Math.random()));
        buffer.setLED(buffer.getLength() - index - 1, new Color(Math.random(), Math.random(), Math.random()));
      }
      m_firstTime = false;
    }
    for (int index = 0; index < buffer.getLength() / 2; index++) {
      buffer.setLED(index, randomColorShift(buffer.getLED(index)));
      buffer.setLED(buffer.getLength() - index - 1, randomColorShift(buffer.getLED(index)));
    }

  }

  private Color randomColorShift(Color aColor) {
    return new Color(randomShift(aColor.red), randomShift(aColor.green), randomShift(aColor.blue));
  }

  private double randomShift(double value) {
    double sign = Math.random() >= 0.5 ? 1.0 : -1.0;
    double amount = Math.random() / 10;
    return MathUtil.clamp(value + sign * amount, 0, 1);
  }

  public boolean isAnimated() {
    return true;
  }
}

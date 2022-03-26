package globals.utils.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface AddressableLEDPattern {
  public void setLEDs(AddressableLEDBuffer buffer);

  default boolean isAnimated() {
    return false;
  }
}
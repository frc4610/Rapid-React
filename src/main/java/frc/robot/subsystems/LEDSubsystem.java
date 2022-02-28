package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_ledStrip;
  private final AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue = 0;

  public static int LED_STRIP_COUNT = 60;
  public static int LED_COUNT = 68;

  public LEDSubsystem() {
    m_ledStrip = new AddressableLED(Ids.LED_STRIP);
    m_ledBuffer = new AddressableLEDBuffer(LED_STRIP_COUNT);
  }

  public void setLEDStripColor(int r, int g, int b) {
    for (var idx = 0; idx < m_ledBuffer.getLength(); idx++) {
      setLEDColor(r, g, b, idx);
    }
  }

  public void setLEDColor(int r, int g, int b, int idx) {
    m_ledBuffer.setRGB(idx, 255, 0, 0);
  }

  public void setLEDStripRainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  @Override
  public void periodic() {
    m_ledStrip.setData(m_ledBuffer);
  }
}
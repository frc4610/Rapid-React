package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

// I would use typedef but java doesnt have it
class LED {
  public boolean isEnabled = false;
  private int m_r;
  private int m_g;
  private int m_b;

  LED() {
    setColor(0, 0, 0);
    // isEnabled = false;
  }

  LED(int r, int g, int b) {
    setColor(r, g, b);
    // isEnabled = false;
  }

  LED(int r, int g, int b, boolean enabled) {
    setColor(r, g, b);
    isEnabled = enabled;
  }

  public int getRed() {
    return m_r;
  }

  public int getGreen() {
    return m_g;
  }

  public int getBlue() {
    return m_b;
  }

  public void setColor(int r, int g, int b) {
    if (m_r != r || m_g != g || m_b != b) {
      m_r = r;
      m_g = g;
      m_b = b;
      isEnabled = true;
    }
  }
}

class LEDSegment {
  private final int m_offset, m_total;
  private List<Pair<Integer, LED>> m_leds = new ArrayList<>();

  LEDSegment(final int offsetIdx, final int totalCount) {
    m_offset = offsetIdx;
    m_total = totalCount;
    for (int i = 0; i < totalCount; i++) {
      m_leds.add(Pair.of(offsetIdx + i, new LED()));
    }
  }

  public void setIndex(final int r, final int g, final int b, final int idx) {
    for (Pair<Integer, LED> pair : m_leds) {
      if (pair.getFirst() - m_offset == idx) {
        pair.getSecond().setColor(r, g, b);
        break;
      }
    }
  }

  public void setAll(final int r, final int g, final int b) {
    for (Pair<Integer, LED> pair : m_leds) {
      pair.getSecond().setColor(r, g, b);
    }
  }

  public void updateLEDs(final CANdle controller) {
    for (Pair<Integer, LED> pair : m_leds) {
      if (pair.getSecond().isEnabled) {
        if (controller.setLEDs(pair.getSecond().getRed(), pair.getSecond().getGreen(), pair.getSecond().getBlue(), 0,
            pair.getFirst(), 1) != ErrorCode.OK) {
          // pair.getSecond().isEnabled = false;
        }
      }
    }
  }
}

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_ledStrip;
  private final AddressableLEDBuffer m_ledBuffer;

  private final CANdle m_ledController;
  private final CANdleConfiguration m_ledControllerConfig;
  private final CANdleFaults m_faults;
  private final LEDSegment m_statusSegment;

  private int m_rainbowFirstPixelHue = 0;

  public static int LED_STRIP_COUNT = 60;

  public LEDSubsystem() {
    m_ledStrip = new AddressableLED(Ids.PWM_LED_STRIP);
    m_ledBuffer = new AddressableLEDBuffer(LED_STRIP_COUNT);
    m_ledStrip.setLength(LED_STRIP_COUNT);
    m_ledStrip.setData(m_ledBuffer);
    m_ledStrip.start();

    m_ledController = new CANdle(Ids.LED_CANDLE);
    m_ledControllerConfig = new CANdleConfiguration();
    m_faults = new CANdleFaults();

    m_ledControllerConfig.stripType = LEDStripType.GRB; // led strip type
    m_ledControllerConfig.brightnessScalar = 0.5; // dim the LEDs to half brightness
    m_ledControllerConfig.disableWhenLOS = true; // when losing connection turn off
    m_ledControllerConfig.statusLedOffWhenActive = true; // Removes orange tint
    m_ledController.configAllSettings(m_ledControllerConfig);

    m_statusSegment = new LEDSegment(0, 8);
  }

  public void setAll(int r, int g, int b) {
    for (var idx = 0; idx < m_ledBuffer.getLength(); idx++) {
      setLEDColor(r, g, b, idx);
    }
  }

  public void setLEDColor(int r, int g, int b, int idx) {
    m_ledBuffer.setRGB(idx, r, g, b);
  }

  public void setStatusDefault() {
    m_statusSegment.setAll(255, 255, 255);
  }

  public void setStatus(final int r, final int g, final int b, final int idx) {
    m_statusSegment.setIndex(r, g, b, idx);
  }

  public void setStatus(final boolean enabled, boolean vital, final int idx) {
    if (enabled)
      m_statusSegment.setIndex(0, 255, 0, idx);
    else if (vital)
      m_statusSegment.setIndex(255, 0, 0, idx);
  }

  public void setStatus(final boolean enabled, final int idx) {
    if (enabled)
      m_statusSegment.setIndex(0, 255, 0, idx);
    else
      m_statusSegment.setIndex(255, 0, 0, idx);
  }

  public void setLEDStripRainbow() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void setAllianceColors() {
    int red = 0;
    int blue = 0;

    if (DriverStation.getAlliance() != Alliance.Blue) {
      red = 255;
      blue = 0;
    } else {
      blue = 255;
      red = 0;
    }
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, 0, blue);
    }
  }

  @Override
  public void periodic() {
    m_ledStrip.setData(m_ledBuffer);
    m_statusSegment.updateLEDs(m_ledController);
  }

  public ErrorCode getLastError() {
    return m_ledController.getLastError();
  }

  public ErrorCode getFaults() {
    return m_ledController.getFaults(m_faults);
  }
}
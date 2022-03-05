package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.utils.MathUtils;

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
    for (Pair<Integer, LED> pair : m_leds) { // TODO: make index an array instead of Map but java doesnt expose idx
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
          pair.getSecond().isEnabled = false;
        }
      }
    }
  }
}

public class LEDSubsystem extends SubsystemBase {
  private final CANdle m_ledController;
  private final CANdleConfiguration m_ledControllerConfig;
  private final CANdleFaults m_faults;
  private final LEDSegment m_statusSegment;

  private int colorLengthLeft = 30;
  private int colorLengthRight = 30;
  private Color colorLeft = new Color(255, 0, 0);
  private Color colorRight = new Color(255, 0, 0);

  public LEDSubsystem() {
    m_ledController = new CANdle(Ids.LED_CANDLE);
    m_ledControllerConfig = new CANdleConfiguration();
    m_faults = new CANdleFaults();

    m_ledControllerConfig.stripType = LEDStripType.GRB; // led strip type
    m_ledControllerConfig.brightnessScalar = 0.5; // dim the LEDs to half brightness
    m_ledControllerConfig.disableWhenLOS = true; // when losing connection turn off
    m_ledController.configAllSettings(m_ledControllerConfig);

    m_statusSegment = new LEDSegment(0, 8);
  }

  public ErrorCode getLastError() {
    return m_ledController.getLastError();
  }

  public ErrorCode getFaults() {
    return m_ledController.getFaults(m_faults);
  }

  public void setStatusDefault() {
    m_statusSegment.setAll(255, 255, 255);
  }

  public void setStatus(final int r, final int g, final int b, final int idx) {
    m_statusSegment.setIndex(r, g, b, idx);
  }

  public void setStatus(final boolean enabled, final int idx) {
    if (enabled)
      m_statusSegment.setIndex(0, 255, 0, idx);
    else
      m_statusSegment.setIndex(255, 0, 0, idx);
  }

  public void setAll(int r, int g, int b) {
    colorLengthLeft = 30;
    colorLengthRight = 30;
    colorLeft = new Color(r, g, b);
    colorRight = new Color(r, g, b);
  }

  public void setLeftColor(int r, int g, int b) {
    colorLengthLeft = 30;
    colorLeft = new Color(r, g, b);
  }

  public void setRightColor(int r, int g, int b) {
    colorLengthRight = 30;
    colorRight = new Color(r, g, b);
  }

  public void setLeftColorLerped(int r, int g, int b, double t) {
    colorLengthLeft = MathUtils.lerp(0, 30, t);
    colorLeft = new Color(r, g, b);
  }

  public void setRightColorLerped(int r, int g, int b, double t) {
    colorLengthRight = MathUtils.lerp(0, 30, t);
    colorRight = new Color(r, g, b);
  }

  @Override
  public void periodic() {

    // Phoenix does something similar in there multi animation branch
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/c733d1c9d8ed89691fbe8c5c05c4e7bac8fe9efb/Java%20General/CANdle%20MultiAnimation/src/main/java/frc/robot/subsystems/CANdleSystem.java#L221
    m_ledController.setLEDs((int) colorLeft.red, (int) colorLeft.green, (int) colorLeft.blue, 0, 8, colorLengthLeft);
    m_ledController.setLEDs((int) colorRight.red, (int) colorRight.green, (int) colorRight.blue, 0, 30,
        colorLengthRight);
    m_statusSegment.updateLEDs(m_ledController);
  }
}
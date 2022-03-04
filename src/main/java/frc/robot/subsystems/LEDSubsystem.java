package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.Pair;
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
    isEnabled = false;
  }

  LED(int r, int g, int b) {
    setColor(r, g, b);
    isEnabled = false;
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
      if (pair.getFirst() == idx) {
        pair.getSecond().setColor(r, g, b);
        break;
      }
    }
  }

  // percent [0-1]
  public void setPercent(final int r, final int g, final int b, final double percent) {
    int total = MathUtils.lerp(0, m_total, percent);
    for (Pair<Integer, LED> pair : m_leds) {
      if (total > pair.getFirst()) {
        pair.getSecond().setColor(r, g, b);
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
        controller.setLEDs(pair.getSecond().getRed(), pair.getSecond().getGreen(), pair.getSecond().getBlue(), 0,
            m_offset + pair.getFirst(), 1);
        pair.getSecond().isEnabled = false;
      }
    }
  }
}

public class LEDSubsystem extends SubsystemBase {
  private final CANdle m_ledController;
  private final CANdleConfiguration m_ledControllerConfig;
  private final CANdleFaults m_faults;
  private final LEDSegment m_statusSegment, m_firstSegment, m_secondSegment;

  public LEDSubsystem() {
    m_ledController = new CANdle(Ids.LED_CANDLE);
    m_ledControllerConfig = new CANdleConfiguration();
    m_faults = new CANdleFaults();

    m_ledControllerConfig.stripType = LEDStripType.GRB; // led strip type
    m_ledControllerConfig.brightnessScalar = 0.5; // dim the LEDs to half brightness
    m_ledControllerConfig.disableWhenLOS = true; // when losing connection turn off
    m_ledController.configAllSettings(m_ledControllerConfig);

    m_statusSegment = new LEDSegment(0, 8);
    m_firstSegment = new LEDSegment(8, 30);
    m_secondSegment = new LEDSegment(30, 30);
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
    m_firstSegment.setAll(r, g, b);
    m_secondSegment.setAll(r, g, b);
  }

  public void setLeftColor(int r, int g, int b) {
    m_firstSegment.setAll(r, g, b);
  }

  public void setRightColor(int r, int g, int b) {
    m_secondSegment.setAll(r, g, b);
  }

  public void setLeftColorLerped(int r, int g, int b, double t) {
    m_firstSegment.setPercent(r, g, b, t);
  }

  public void setRightColorLerped(int r, int g, int b, double t) {
    m_secondSegment.setPercent(r, g, b, t);
  }

  @Override
  public void periodic() {

    // Phoenix does something similar in there multi animation branch
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/c733d1c9d8ed89691fbe8c5c05c4e7bac8fe9efb/Java%20General/CANdle%20MultiAnimation/src/main/java/frc/robot/subsystems/CANdleSystem.java#L221
    m_statusSegment.setAll(200, 200, 200);
    m_firstSegment.updateLEDs(m_ledController);
    m_secondSegment.updateLEDs(m_ledController);
    m_statusSegment.updateLEDs(m_ledController);
  }
}
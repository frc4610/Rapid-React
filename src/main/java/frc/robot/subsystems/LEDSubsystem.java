package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

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
    m_r = r;
    m_g = g;
    m_b = b;
  }
}

public class LEDSubsystem extends SubsystemBase {
  private final CANdle m_ledController;
  private final CANdleConfiguration m_ledControllerConfig;
  private final CANdleFaults m_faults;

  public static int LED_STRIP_COUNT = 60;
  public static int LED_COUNT = 68;

  private static List<Pair<Integer, LED>> m_ledMap = new ArrayList<>();

  public LEDSubsystem() {
    m_ledController = new CANdle(Ids.LED_CANDLE);
    m_ledControllerConfig = new CANdleConfiguration();
    m_faults = new CANdleFaults();

    m_ledControllerConfig.stripType = LEDStripType.GRB; // led strip type
    m_ledControllerConfig.brightnessScalar = 0.5; // dim the LEDs to half brightness
    m_ledControllerConfig.disableWhenLOS = true; // when losing connection turn off
    m_ledController.configAllSettings(m_ledControllerConfig);

    for (int i = 0; i < LED_COUNT; i++) {
      m_ledMap.add(Pair.of(i, new LED(0, 0, 0)));
    }
  }

  public ErrorCode setAnimation(Animation anim) {
    return m_ledController.animate(anim);
  }

  public ErrorCode getLastError() {
    return m_ledController.getLastError();
  }

  public ErrorCode getFaults() {
    return m_ledController.getFaults(m_faults);
  }

  public void setStatusLEDColor(int r, int g, int b, int idx) {
    setLEDColor(r, g, b, idx);
  }

  public void setLEDStripColor(int r, int g, int b) {
    for (int idx = 8; idx < 68; idx++) {
      setLEDColor(r, g, b, idx);
    }
  }

  public void setLEDColor(int r, int g, int b, int idx) {
    m_ledMap.set(idx, Pair.of(idx, new LED(r, g, b, true)));
  }

  @Override
  public void periodic() {
    // Phoenix does something similar in there multi animation branch
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/c733d1c9d8ed89691fbe8c5c05c4e7bac8fe9efb/Java%20General/CANdle%20MultiAnimation/src/main/java/frc/robot/subsystems/CANdleSystem.java#L221
    for (Pair<Integer, LED> pair : m_ledMap) {
      if (pair.getSecond().isEnabled) {
        m_ledController.setLEDs(pair.getSecond().getRed(), pair.getSecond().getGreen(), pair.getSecond().getBlue(), 0,
            pair.getFirst(), 1);
        pair.getSecond().isEnabled = false;
      }
    }
  }
}
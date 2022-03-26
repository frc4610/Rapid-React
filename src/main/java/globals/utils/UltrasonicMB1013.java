package globals.utils;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import globals.utils.math.MathUtils;

// https://www.maxbotix.com/firstrobotics

//300mm to 5000mm, a 10Hz

public class UltrasonicMB1013 {
  private final DigitalOutput m_trigger;
  private final AnalogInput m_sensor;
  private double m_lastMsUpdateTime;
  public static double refreshRate = 100.0;

  public UltrasonicMB1013(int port) {
    m_trigger = new DigitalOutput(port);
    m_sensor = new AnalogInput(port);
    setStatus(true);
  }

  public void setStatus(boolean value) {
    m_trigger.set(value);
    m_lastMsUpdateTime = RobotContainer.getMsClock();
  }

  private double getRawRange() {
    return m_sensor.getValue() * RobotContainer.get5VScalar();
  }

  public double getRangeInch() {
    return MathUtils.clamp(getRawRange() * 0.0492 - Ultrasonic.LENGTH_FROM_SIDE, 12.0, 190.0);
  }

  public boolean isWithinRange() {
    return MathUtils.withinRange(getRawRange() * 0.0492, 12.0, 190.0);
  }

  public double getLastMsUpdateTime() {
    return m_lastMsUpdateTime;
  }
}
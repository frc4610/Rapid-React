package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.RobotContainer;

// https://www.maxbotix.com/firstrobotics

//300mm to 5000mm, a 10Hz

public class UltrasonicMB1013 {
  private final DigitalOutput m_trigger;
  private final AnalogInput m_sensor;

  public UltrasonicMB1013(int port) {
    m_trigger = new DigitalOutput(port);
    m_sensor = new AnalogInput(port);
    setStatus(true);
  }

  public void setStatus(boolean value) {
    m_trigger.set(value);
  }

  public double getRangeMeters() {
    return MathUtils.clamp(m_sensor.getValue() * RobotContainer.get5VScalar(), 0.3, 5.0); // 30cm - 500cm
  }

  public double getRangeInch() {
    return getRangeMeters() * 0.0492;
  }

  public double getRangeCm() {
    return getRangeMeters() * 0.125;
  }
}
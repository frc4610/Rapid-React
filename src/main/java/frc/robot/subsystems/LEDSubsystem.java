package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle m_ledController;
  private final CANdleConfiguration m_ledControllerConfig;
  private final CANdleFaults m_faults;

  public static int LED_STRIP_COUNT = 60;
  public static int LED_COUNT = 68;

  public LEDSubsystem() {
    m_ledController = new CANdle(0);
    m_ledControllerConfig = new CANdleConfiguration();
    m_faults = new CANdleFaults();

    m_ledControllerConfig.stripType = LEDStripType.GRB; // led strip type
    m_ledControllerConfig.brightnessScalar = 0.5; // dim the LEDs to half brightness
    m_ledControllerConfig.disableWhenLOS = true; // when losing connection turn off
    m_ledController.configAllSettings(m_ledControllerConfig);

    setStatusLEDColor(0, 255, 0, 1);
    setLEDStripColor(255, 255, 255); // set the CANdle LEDs to white

    setAnimation(new RainbowAnimation(1, 0.5, LED_STRIP_COUNT)); // brightness, speed, # of leds
  }

  public ErrorCode setAnimation(Animation anim) {
    return m_ledController.animate(anim);
  }

  public ErrorCode setStatusLEDColor(int r, int g, int b, int idx) {
    return m_ledController.setLEDs(r, g, b, 0, idx, idx);
  }

  // FIXME: Java has no default params?
  public ErrorCode setLEDStripColor(int r, int g, int b) {
    return setLEDStripColor(r, g, b, 0);
  }

  public ErrorCode setLEDStripColor(int r, int g, int b, int white) {
    return m_ledController.setLEDs(r, g, b, white, LED_COUNT - LED_STRIP_COUNT, LED_COUNT);
  }

  public ErrorCode getLastError() {
    return m_ledController.getLastError();
  }

  public ErrorCode getFaults() {
    return m_ledController.getFaults(m_faults);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getLastError() != ErrorCode.OK) {
      DriverStation.reportWarning("LEDSubsystem " + getLastError().name(), false);
    }
    if (m_faults.hasAnyFault()) {
      DriverStation.reportWarning("LEDSubsystem fault bitwise " + Long.toString(m_faults.toBitfield()), false);
    }
  }

}
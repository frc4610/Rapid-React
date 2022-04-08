package frc.robot.subsystems;

import beartecs.UltrasonicMB1013;
import beartecs.Constants.*;
import beartecs.math.MathUtils;
import beartecs.template.BaseSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class UltrasonicSubsystem extends BaseSubsystem {
  private final ShuffleboardTab m_ultrasonicTab;
  private final ShuffleboardLayout m_ultrasonicLayout;
  private final UltrasonicMB1013 m_ultrasonicLeft, m_ultrasonicRight;
  private final LEDSubsystem m_ledSubsystem;
  private boolean m_isEnabled, m_isAligned;

  public UltrasonicSubsystem(LEDSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
    m_ultrasonicTab = addTab("Ultrasonic");
    m_ultrasonicLeft = new UltrasonicMB1013(Ids.LEFT_ULTRASONIC);
    m_ultrasonicRight = new UltrasonicMB1013(Ids.RIGHT_ULTRASONIC);
    EnableSensors();

    m_ultrasonicLayout = m_ultrasonicTab.getLayout("Data", BuiltInLayouts.kGrid)
        .withSize(2, 1)
        .withPosition(5, 0);
    m_ultrasonicLayout.addNumber("Rotation", () -> getUltrasonicRotation().getDegrees())
        .withPosition(1, 0)
        .withSize(1, 1);
    m_ultrasonicLayout.addNumber("Distance", () -> getUltrasonicDistance())
        .withPosition(2, 0)
        .withSize(1, 1);
  }

  public void EnableSensors() {
    m_isEnabled = true;
    m_ultrasonicLeft.setStatus(true);
    m_ultrasonicRight.setStatus(true);
  }

  public void DisableSensors() {
    m_isEnabled = false;
    m_ultrasonicLeft.setStatus(false);
    m_ultrasonicRight.setStatus(false);
  }

  // atan(Delta/Width)
  public Rotation2d getUltrasonicRotation() {
    return new Rotation2d(
        Math.atan2(m_ultrasonicLeft.getRangeInch() - m_ultrasonicRight.getRangeInch(), Ultrasonic.WIDTH_INCH));
  }

  public double getUltrasonicDistance() {
    return Math.min(m_ultrasonicLeft.getRangeInch(), m_ultrasonicRight.getRangeInch());
  }

  public boolean isUltrasonicAligned() {
    return m_isAligned;
  }

  @Override
  public void periodic() {
    m_isAligned = MathUtils.withinRange(getUltrasonicRotation().getDegrees(), -Ultrasonic.ANGULAR_THRESHOLD,
        Ultrasonic.ANGULAR_THRESHOLD)
        && MathUtils.withinRange(getUltrasonicDistance(), Ultrasonic.MIN_DISTANCE, Ultrasonic.MAX_DISTANCE);
    if (m_isEnabled && getRobotMode() != RobotMode.AUTO) {
      if (isUltrasonicAligned()) {
        m_ledSubsystem.setPattern(LEDSubsystem.m_greenAlternating);
      } else {
        m_ledSubsystem.setAllianceColors();
      }
    }
  }
}
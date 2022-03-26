package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.*;
import globals.utils.BaseSubsystem;
import globals.utils.UltrasonicMB1013;
import globals.utils.math.MathUtils;

public class UltrasonicSubsystem extends BaseSubsystem {
  private final ShuffleboardTab m_ultrasonicTab;
  private final ShuffleboardLayout m_ultrasonicLayout;
  private final UltrasonicMB1013 m_ultrasonicLeft, m_ultrasonicRight;
  private final LEDSubsystem m_ledSubsystem;
  private boolean m_isEnabled;

  public UltrasonicSubsystem(LEDSubsystem ledsubsystem) {
    m_ledSubsystem = ledsubsystem;
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

  @Override
  public boolean isOkay() {
    return true;
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

  @Override
  public void periodic() {
    if (m_isEnabled && getRobotMode() != RobotMode.AUTO) {
      double minDistance = getUltrasonicDistance();
      if (MathUtils.withinRange(getUltrasonicRotation().getDegrees(), -Ultrasonic.ANGULAR_THRESHOLD,
          Ultrasonic.ANGULAR_THRESHOLD)
          && MathUtils.withinRange(minDistance, Ultrasonic.MIN_DISTANCE, Ultrasonic.MAX_DISTANCE)) {
        m_ledSubsystem.setPattern(LEDSubsystem.m_greenAlternating);
      } else {
        m_ledSubsystem.setAllianceColors();
      }
    }
  }
}
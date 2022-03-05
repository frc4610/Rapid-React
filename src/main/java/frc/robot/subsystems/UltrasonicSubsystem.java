package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.*;
import frc.robot.utils.BaseSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.ThreadPool;
import frc.robot.utils.UltrasonicMB1013;

public class UltrasonicSubsystem extends BaseSubsystem {
  private final ShuffleboardTab m_ultrasonicTab;
  private final ShuffleboardLayout m_ultrasonicLayout;
  private final UltrasonicMB1013 m_ultrasonicLeft, m_ultrasonicRight;
  private final LEDSubsystem m_ledSubsystem;

  public UltrasonicSubsystem(LEDSubsystem ledsubsystem) {
    m_ledSubsystem = ledsubsystem;
    m_ultrasonicTab = Shuffleboard.getTab("Ultrasonic");
    m_ultrasonicLeft = new UltrasonicMB1013(Ids.LEFT_ULTRASONIC);
    m_ultrasonicRight = new UltrasonicMB1013(Ids.RIGHT_ULTRASONIC);
    m_ultrasonicLeft.setStatus(true);
    m_ultrasonicRight.setStatus(true);

    m_ultrasonicLayout = m_ultrasonicTab.getLayout("Data", BuiltInLayouts.kGrid)
        .withSize(3, 1)
        .withPosition(5, 0);
    m_ultrasonicLayout.addNumber("One Range", () -> m_ultrasonicLeft.getRangeInch())
        .withPosition(1, 0)
        .withSize(1, 1);
    m_ultrasonicLayout.addNumber("Two Range", () -> m_ultrasonicRight.getRangeInch())
        .withPosition(2, 0)
        .withSize(1, 1);
    m_ultrasonicLayout.addNumber("Rotation", () -> getUltrasonicRotation().getDegrees())
        .withPosition(3, 0)
        .withSize(1, 1);
    m_ultrasonicLayout.addNumber("Distance", () -> getUltrasonicDistance())
        .withPosition(4, 0)
        .withSize(1, 1);
  }

  @Override
  public boolean isOkay() {
    return true;
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
    double minDistance = getUltrasonicDistance();

    if (MathUtils.withinRange(getUltrasonicRotation().getDegrees(), -Ultrasonic.ANGULAR_THRESHOLD,
        Ultrasonic.ANGULAR_THRESHOLD)
        && MathUtils.withinRange(minDistance, Ultrasonic.MIN_DISTANCE, Ultrasonic.MAX_DISTANCE)) {
      m_ledSubsystem.setAll(0, 255, 0);
    } else {
      m_ledSubsystem.setAll(0, 0, 255);
    }
  }
}
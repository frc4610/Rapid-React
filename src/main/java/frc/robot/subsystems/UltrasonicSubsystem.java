package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.utils.MathUtils;
import frc.robot.utils.ThreadPool;
import frc.robot.utils.UltrasonicMB1013;

public class UltrasonicSubsystem extends SubsystemBase {
  private final ShuffleboardTab m_ultrasonicTab;
  private final ShuffleboardLayout m_ultrasonicLayout;
  private final UltrasonicMB1013 m_ultrasonicLeft, m_ultrasonicRight;
  private final LEDSubsystem m_ledSubsystem;

  public UltrasonicSubsystem(LEDSubsystem ledsubsystem) {
    m_ledSubsystem = ledsubsystem;
    m_ultrasonicTab = Shuffleboard.getTab("Ultrasonic");
    m_ultrasonicLeft = new UltrasonicMB1013(Ids.LEFT_ULTRASONIC);
    m_ultrasonicRight = new UltrasonicMB1013(Ids.RIGHT_ULTRASONIC);
    ThreadPool.threadPool.execute(() -> {
      try {
        while (true) {
          // synchronized should fix deadlocking if it happens
          synchronized (m_ultrasonicLeft) {
            m_ultrasonicLeft.setStatus(false);
          }
          synchronized (m_ultrasonicRight) {
            m_ultrasonicRight.setStatus(true);
          }
          Thread.sleep(Double.doubleToLongBits(UltrasonicMB1013.refreshRate));
          synchronized (m_ultrasonicLeft) {
            m_ultrasonicLeft.setStatus(true);
          }
          synchronized (m_ultrasonicRight) {
            m_ultrasonicRight.setStatus(false);
          }
          Thread.sleep(Double.doubleToLongBits(UltrasonicMB1013.refreshRate));
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    });

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

  public boolean isOkay() {
    // FIXME: check if ultrasonic is plugged in
    return true;
  }

  // atan(Delta/Width)
  public Rotation2d getUltrasonicRotation() {
    return new Rotation2d(Math.atan(m_ultrasonicLeft.getRangeInch() - m_ultrasonicRight.getRangeInch() /
        Ultrasonic.WIDTH_INCH));
  }

  public double getUltrasonicDistance() {
    return Math.min(m_ultrasonicLeft.getRangeInch(), m_ultrasonicRight.getRangeInch());
  }

  @Override
  public void periodic() {
    double minDistance = getUltrasonicDistance();
    boolean aligned = MathUtils.withinRange(getUltrasonicRotation().getDegrees(), -Ultrasonic.ANGULAR_THRESHOLD,
        Ultrasonic.ANGULAR_THRESHOLD);
    boolean isCloseEnough = MathUtils.withinRange(minDistance, Ultrasonic.MIN_DISTANCE, Ultrasonic.MAX_DISTANCE);

    if (minDistance < Ultrasonic.LED_START_RANGE) {
      m_ledSubsystem.setRightColorLerped(0, 0, 255,
          MathUtils.clamp(m_ultrasonicRight.getRangeInch(), Ultrasonic.MAX_DISTANCE, Ultrasonic.LED_START_RANGE)
              / Ultrasonic.LED_START_RANGE);

      m_ledSubsystem.setLeftColorLerped(0, 0, 255,
          MathUtils.clamp(m_ultrasonicLeft.getRangeInch(), Ultrasonic.MAX_DISTANCE, Ultrasonic.LED_START_RANGE)
              / Ultrasonic.LED_START_RANGE);
    }

    if (!m_ultrasonicLeft.isWithinRange()) {
      m_ledSubsystem.setLeftColor(255, 0, 0);
    }
    if (!m_ultrasonicRight.isWithinRange()) {
      m_ledSubsystem.setRightColor(255, 0, 0);
    }

    if (aligned && isCloseEnough) {
      m_ledSubsystem.setAll(0, 255, 0);
    }
  }
}
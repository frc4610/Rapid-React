package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.utils.ThreadPool;
import frc.robot.utils.UltrasonicMB1013;

public class UltrasonicSubsystem extends SubsystemBase {
  private final ShuffleboardTab m_ultrasonicTab;
  private final ShuffleboardLayout m_ultrasonicLayout;
  private final UltrasonicMB1013 m_ultrasonicOne, m_ultrasonicTwo;

  public UltrasonicSubsystem() {
    // Ultrasonic
    m_ultrasonicTab = Shuffleboard.getTab("Ultrasonic");
    // FIXME: Atomic or Mutex lock ultrasonic
    m_ultrasonicOne = new UltrasonicMB1013(Ids.LEFT_ULTRASONIC);
    m_ultrasonicTwo = new UltrasonicMB1013(Ids.RIGHT_ULTRASONIC);
    ThreadPool.threadPool.execute(() -> {
      try {
        while (true) {
          // synchronized should fix deadlocking if it happens
          synchronized (m_ultrasonicOne) {
            m_ultrasonicOne.setStatus(false);
          }
          synchronized (m_ultrasonicTwo) {
            m_ultrasonicTwo.setStatus(true);
          }
          Thread.sleep(Double.doubleToLongBits(UltrasonicMB1013.refreshRate));
          synchronized (m_ultrasonicOne) {
            m_ultrasonicOne.setStatus(true);
          }
          synchronized (m_ultrasonicTwo) {
            m_ultrasonicTwo.setStatus(false);
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
    m_ultrasonicLayout.addNumber("One Range", () -> m_ultrasonicOne.getRangeInch())
        .withPosition(1, 0)
        .withSize(1, 1);
    m_ultrasonicLayout.addNumber("Two Range", () -> m_ultrasonicTwo.getRangeInch())
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
    return new Rotation2d(Math.atan(m_ultrasonicOne.getRangeInch() - m_ultrasonicTwo.getRangeInch() /
        Ultrasonic.WIDTH_INCH));
  }

  public double getUltrasonicDistance() {
    return Math.min(m_ultrasonicOne.getRangeInch(), m_ultrasonicTwo.getRangeInch());
  }
}
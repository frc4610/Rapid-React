package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.UltrasonicMB1013;

// not sure how mutexs work in java/ threads
class UltrasonicThread implements Runnable {
  private Thread m_thread;
  private String m_threadName;
  private final UltrasonicMB1013 m_ultrasoncisOne;
  private final UltrasonicMB1013 m_ultrasoncisTwo;

  UltrasonicThread(String name, UltrasonicMB1013 one, UltrasonicMB1013 two) {
    m_threadName = name;
    m_ultrasoncisOne = one;
    m_ultrasoncisTwo = two;
    System.out.println("Creating " + m_threadName);
  }

  public void run() {
    System.out.println("Running " + m_threadName);
    try {
      while (true) {
        m_ultrasoncisOne.setStatus(false);
        m_ultrasoncisTwo.setStatus(true);
        Thread.sleep(Double.doubleToLongBits(UltrasonicMB1013.refreshRate));
        m_ultrasoncisOne.setStatus(true);
        m_ultrasoncisTwo.setStatus(false);
        Thread.sleep(Double.doubleToLongBits(UltrasonicMB1013.refreshRate));
      }
    } catch (InterruptedException e) {
      System.out.println("Thread " + m_threadName + " interrupted.");
    }
    System.out.println("Thread " + m_threadName + " exiting.");
  }

  public void start() {
    System.out.println("Starting " + m_threadName);
    if (m_thread == null) {
      m_thread = new Thread(this, m_threadName);
      m_thread.start();
    }
  }
}

public class UltrasonicSubsystem extends SubsystemBase {
  private final ShuffleboardTab m_ultrasonicTab;
  private final ShuffleboardLayout m_ultrasonicLayout;
  private final UltrasonicThread m_ultrasonicThread;
  private final UltrasonicMB1013 m_ultrasoncisOne;
  private final UltrasonicMB1013 m_ultrasoncisTwo;

  public UltrasonicSubsystem() {
    // Ultrasonic
    m_ultrasonicTab = Shuffleboard.getTab("Ultrasonic");
    m_ultrasoncisOne = new UltrasonicMB1013(0);
    m_ultrasoncisTwo = new UltrasonicMB1013(1);
    m_ultrasonicThread = new UltrasonicThread("Ultrasonic", m_ultrasoncisOne, m_ultrasoncisTwo);
    m_ultrasonicThread.start();

    m_ultrasonicLayout = m_ultrasonicTab.getLayout("Data", BuiltInLayouts.kGrid)
        .withSize(3, 1)
        .withPosition(5, 0);
    m_ultrasonicLayout.addNumber("One Range", () -> m_ultrasoncisOne.getRangeInch())
        .withPosition(1, 0)
        .withSize(1, 1);
    m_ultrasonicLayout.addNumber("Two Range", () -> m_ultrasoncisTwo.getRangeInch())
        .withPosition(2, 0)
        .withSize(1, 1);
    m_ultrasonicLayout.addNumber("Rotation", () -> getUltrasonicRotation().getDegrees())
        .withPosition(3, 0)
        .withSize(1, 1);
    m_ultrasonicLayout.addNumber("Distance", () -> getUltrasonicDistance())
        .withPosition(4, 0)
        .withSize(1, 1);
  }

  // atan(Delta/Width)
  public Rotation2d getUltrasonicRotation() {
    return new Rotation2d(Math.atan(m_ultrasoncisOne.getRangeInch() - m_ultrasoncisTwo.getRangeInch() /
        Constants.Ultrasonic.WIDTH_INCH));
  }

  public double getUltrasonicDistance() {
    return Math.min(m_ultrasoncisOne.getRangeInch(), m_ultrasoncisTwo.getRangeInch());
  }
}
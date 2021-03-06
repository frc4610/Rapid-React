// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import beartecs.Constants;
import beartecs.LED.TimerPattern;
import beartecs.logger.RobotLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer = new RobotContainer();
  private final RobotLogger m_logger = RobotContainer.getLogger();
  private boolean m_hadLowBattery = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_logger.logInfo("robotInit()");
    m_robotContainer.onRobotInit();
    CameraServer.startAutomaticCapture();
    onModeInit();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    final CANStatus canBus = RobotController.getCANStatus();
    if (canBus.percentBusUtilization > 80.0) {
      m_logger.logWarning("CANBus{" +
          "percentBusUtilization=" + canBus.percentBusUtilization +
          ", busOffCount=" + canBus.busOffCount +
          ", txFullCount=" + canBus.txFullCount +
          ", receiveErrorCount=" + canBus.receiveErrorCount +
          ", transmitErrorCount=" + canBus.transmitErrorCount +
          '}');
    }
    if (!RobotContainer.checkRoboRIO()) {
      m_logger.logWarning("RoboRIO fault detected!");
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_logger.logInfo("disabledInit()");
    RobotContainer.getDrivetrain().zeroGyro();
    onModeInit();
    m_logger.flush();
  }

  @Override
  public void disabledPeriodic() {
    if (RobotController.getInputVoltage() < Constants.CHECK_VOLTAGE) {
      RobotContainer.getLEDSubsystem().setPattern(LEDSubsystem.m_blinkingYellow);
      m_hadLowBattery = true;
    } else if (!m_hadLowBattery) {
      //RobotContainer.getLEDSubsystem().setPattern(LEDSubsystem.m_scannerRedPattern);
      RobotContainer.getLEDSubsystem().setPattern(LEDSubsystem.m_rainbowPattern);
    }
    RobotContainer.getDrivetrain().drive(0, 0, 0); // Allows Seeding Talon Encoders with CANCoders
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_logger.logInfo("autonomousInit()");
    m_autonomousCommand = RobotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    double autoTime = DriverStation.isFMSAttached() ? DriverStation.getMatchTime() : 15;
    if (DriverStation.getAlliance() == Alliance.Blue) {
      LEDSubsystem.m_timerPattern = new TimerPattern(Color.kBlue, autoTime);
    } else {
      LEDSubsystem.m_timerPattern = new TimerPattern(Color.kRed, autoTime);
    }
    onModeInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    RobotContainer.getLEDSubsystem().setPattern(LEDSubsystem.m_timerPattern);
  }

  @Override
  public void autonomousExit() {
    //Reset the odometry rotation as the robot leaves autonomous before teleop
    RobotContainer.getDrivetrain().resetPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  @Override
  public void teleopInit() {
    m_logger.logInfo("teleopInit()");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.setDefaultTeleopCommand();
    onModeInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    m_logger.logInfo("testInit()");
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public void onModeInit() {
    LiveWindow.setEnabled(false); // Disable Live Window we don't need that data being sent
    LiveWindow.disableAllTelemetry();
  }
}

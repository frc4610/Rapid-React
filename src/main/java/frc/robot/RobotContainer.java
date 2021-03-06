// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Scanner;
import java.util.logging.Level;

import beartecs.Constants;
import beartecs.logger.RobotLogger;
import beartecs.swerve.sim.PoseTelemetry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Autonomous.AutoActionCmd;
import frc.robot.commands.Teleop.UserControllerCmd;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;

public class RobotContainer {
  public final static File deployDirectory = Filesystem.getDeployDirectory();
  public final static PoseTelemetry telemetry = new PoseTelemetry();
  public String branch = "null";
  private final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final static LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final static UltrasonicSubsystem m_ultrasonicSubsystem = new UltrasonicSubsystem(m_ledSubsystem);
  private final static AutonomousSubsystem m_autonomousSubsystem = new AutonomousSubsystem(m_drivetrainSubsystem,
      m_intakeSubsystem);
  private static RobotLogger m_logger;

  public RobotContainer() {
    try {
      FileReader gitFile = new FileReader(new File(deployDirectory, "git.txt"));
      Scanner gitReader = new Scanner(gitFile);
      if (gitReader.hasNextLine())
        branch = gitReader.nextLine();
      gitReader.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
  }

  public void onRobotInit() {
    SmartDashboard.putString("Branch:", branch);
    SmartDashboard.putData("Field", PoseTelemetry.field);
    m_ultrasonicSubsystem.EnableSensors();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private static void configureDriveButtons() {
    new Button(Controls::getResetGyroButton)
        .whenPressed(() -> {
          m_drivetrainSubsystem.zeroGyro();
        });
  }

  public static void setDefaultTeleopCommand() {
    m_drivetrainSubsystem.zeroGyro();
    m_drivetrainSubsystem.setDefaultCommand(new UserControllerCmd(m_drivetrainSubsystem));

    configureDriveButtons();
  }

  public final static DrivetrainSubsystem getDrivetrain() {
    return m_drivetrainSubsystem;
  }

  public static AutoActionCmd getAutonomousCommand() {
    return m_autonomousSubsystem.getAutoCmd().get();
  }

  public static LEDSubsystem getLEDSubsystem() {
    return m_ledSubsystem;
  }

  public final static UltrasonicSubsystem getUltrasonicSubsystem() {
    return m_ultrasonicSubsystem;
  }

  /**
   * returns scalar of 5V bus
   * 0-1
   */
  public static double get5VScalar() {
    return 5 / RobotController.getVoltage5V();
  }

  /**
   * returns clock in ms
   */
  public static double getMsClock() {
    return RobotController.getFPGATime() / 1000.0;
  }

  /**
   * returns true if everything is working
   */
  public static boolean checkRoboRIO() {
    if (!RobotController.getEnabled5V())
      return false;
    if (!RobotController.getEnabled3V3())
      return false;
    if (!RobotController.getEnabled6V())
      return false;
    return true;
  }

  private static void initLogger(RobotLogger log) {
    try {
      log.init(RobotContainer.class);
      log.setLevel(Level.INFO);

      log.cleanLogs(Constants.LOG_EXPIRATION_IN_HRS);
      log.logInfo("Logger initialized");
    } catch (IOException error) {
      log.logError("Failed to init logger!");
      throw new RuntimeException(error);
    }
  }

  public static RobotLogger getLogger() {
    if (m_logger == null) {
      m_logger = new RobotLogger();
      initLogger(m_logger);
    }
    return m_logger;
  }
}

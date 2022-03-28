// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import beartecs.CAN.CANDevice;
import beartecs.controller.XboxControllerExtended;
import beartecs.json.JsonReader;
import beartecs.math.MathUtils;
import beartecs.swerve.sim.PoseTelemetry;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final static XboxControllerExtended m_driverController = new XboxControllerExtended(0);
  private final static XboxControllerExtended m_operatorController = new XboxControllerExtended(1);
  private final static LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_operatorController);
  private final static UltrasonicSubsystem m_ultrasonicSubsystem = new UltrasonicSubsystem(m_ledSubsystem);
  private final static AutonomousSubsystem m_autonomousSubsystem = new AutonomousSubsystem(m_drivetrainSubsystem,
      m_intakeSubsystem);
  private static List<CANDevice> m_canDevices = new ArrayList<CANDevice>();

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

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private static void configureDriveButtons() {
    new Button(m_driverController::getBackButton)
        .whenPressed(() -> {
          reset();
        });
    new Button(m_driverController::getRightBumper)
        .whenPressed(() -> {
          m_drivetrainSubsystem.setSpeedModifier(0.5);
          m_driverController.setLeftVibration(0.1);
          m_driverController.setRightVibration(0.1);
        }).whenReleased(() -> {
          m_drivetrainSubsystem.setSpeedModifier(1.0);
          m_driverController.setLeftVibration(0.0);
          m_driverController.setRightVibration(0.0);
        });
  }

  public static void updateSubsystemStatus() {
    m_ledSubsystem.setStatus(checkRoboRIO(), 0);
    m_ledSubsystem.setStatus(!m_canDevices.isEmpty(), 1);
    m_ultrasonicSubsystem.onLEDCallback(2);
    m_intakeSubsystem.onLEDCallback(3);
    m_ledSubsystem.setStatus(m_intakeSubsystem.getArmState() == m_intakeSubsystem.getVerifiedArmState(), 4);
    m_ledSubsystem.setStatus(DriverStation.isAutonomousEnabled(), 5);
  }

  public static void requestCANBusData() {
    JSONObject canBusData = JsonReader.getJsonDataFromURL(Constants.RIO_IP + "/?action=getdevices");
    if (canBusData.isEmpty())
      return;
    JSONArray deviceArray = (JSONArray) canBusData.get("DeviceArray");
    if (deviceArray.isEmpty())
      return;

    for (Object elem : deviceArray) {
      m_canDevices.add(new CANDevice((JSONObject) elem));
    }
  }

  public final static DrivetrainSubsystem getDrivetrain() {
    return m_drivetrainSubsystem;
  }

  public static void setDefaultTeleopCommand() {
    m_drivetrainSubsystem.zeroGyro();
    m_drivetrainSubsystem.setDefaultCommand(new UserControllerCmd(m_drivetrainSubsystem));

    configureDriveButtons();
  }

  public static AutoActionCmd getAutonomousCommand() {
    return m_autonomousSubsystem.getAutoCmd().get();
  }

  public static LEDSubsystem getLEDSubsystem() {
    return m_ledSubsystem;
  }

  /**
   * Resets Gyro
   */
  public static void reset() {
    m_drivetrainSubsystem.zeroGyro();
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

  public static double getDriveForwardAxis() {
    return MathUtils.modifyAxis(m_driverController.getLeftY(), Constants.Controller.XBOX_DEADBAND);
  }

  public static double getDriveStrafeAxis() {
    return MathUtils.modifyAxis(m_driverController.getLeftX(), Constants.Controller.XBOX_DEADBAND);
  }

  public static double getDriveRotationAxis() {
    return MathUtils.modifyAxis(m_driverController.getRightX(), Constants.Controller.XBOX_DEADBAND);
  }

  public static Rotation2d getDrivePOV() {
    return Rotation2d.fromDegrees(m_driverController.getPOV());
  }
}

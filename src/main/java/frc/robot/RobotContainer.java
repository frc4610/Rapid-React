// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveContinuous;
import frc.robot.commands.UserControllerCommand;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.CAN.CANDevice;
import frc.robot.utils.Controller.XboxControllerExtended;
import frc.robot.utils.json.JsonReader;

public class RobotContainer {
  // @SuppressWarnings("unused") // to remove warnings
  private final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final static XboxControllerExtended m_driverController = new XboxControllerExtended(0);
  private final static XboxControllerExtended m_operatorController = new XboxControllerExtended(1);
  private final static LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  private final static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
      Constants.USE_ONE_CONTROLLER ? m_driverController : m_operatorController);
  private final static AutonomousSubsystem m_autonomousSubsystem = new AutonomousSubsystem(m_drivetrainSubsystem);
  private final static UltrasonicSubsystem m_ultrasonicSubsystem = new UltrasonicSubsystem(m_ledSubsystem);
  private static List<CANDevice> m_canDevices = new ArrayList<CANDevice>();

  public final static Field2d dashboardField = new Field2d();

  public RobotContainer() {
    Shuffleboard.getTab(Constants.VERSION);
    SmartDashboard.putData("Field", dashboardField);

    m_drivetrainSubsystem.setDefaultCommand(new UserControllerCommand(
        m_drivetrainSubsystem,
        () -> -MathUtils.modifyAxis(m_driverController.getLeftY(), Constants.Controller.XBOX_DEADBAND)
            * Constants.Motor.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -MathUtils.modifyAxis(m_driverController.getLeftX(), Constants.Controller.XBOX_DEADBAND)
            * Constants.Motor.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -MathUtils.modifyAxis(m_driverController.getRightX(), Constants.Controller.XBOX_DEADBAND)
            * Constants.Motor.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Configure the button bindings
    configureDriveButtons();
    // configureLEDButtons();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureDriveButtons() {
    new Button(m_driverController::getBackButton)
        .whenPressed(() -> {
          reset();
        });
    new Button(m_driverController::getAButton)
        .whileHeld(() -> { // As class says don't go nathan mode
          m_drivetrainSubsystem.limitPower();
          m_driverController.setLeftVibration(0.1);
          m_driverController.setRightVibration(0.1);
        });

    new Button(m_driverController::getLeftBumper)
        .whileHeld(new DriveContinuous(m_drivetrainSubsystem, true));
    new Button(m_driverController::getRightBumper)
        .whileHeld(new DriveContinuous(m_drivetrainSubsystem, false));
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutonomousCommand(m_drivetrainSubsystem, m_autonomousSubsystem);
  }

  public static LEDSubsystem getLEDSubsystem() {
    return m_ledSubsystem;
  }

  /**
   * Resets Gyro
   */
  public void reset() {
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
}

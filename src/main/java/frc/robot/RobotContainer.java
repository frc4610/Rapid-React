// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.commands.UserControllerCommand;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.Controller.XboxControllerExtended;

public class RobotContainer {
  // FIXME: Tell the vm to not ignore these unused classes
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final XboxControllerExtended m_controller = new XboxControllerExtended(0);
  // private final VisionSubsysten m_visionSubsystem =
  // new VisionSubsysten(m_drivetrainSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(m_controller);
  private final AutonomousSubsystem m_autonomousSubsystem = new AutonomousSubsystem(m_drivetrainSubsystem);
  private final UltrasonicSubsystem m_ultrasonicSubsystem = new UltrasonicSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  // TODO: Make into a field telometry class with predicted poses
  public final static Field2d dashboardField = new Field2d();

  public RobotContainer() {
    Shuffleboard.getTab(Constants.VERSION);
    SmartDashboard.putData("Field", dashboardField);

    m_drivetrainSubsystem.setDefaultCommand(new UserControllerCommand(
        m_drivetrainSubsystem,
        () -> -MathUtils.modifyAxis(m_controller.getLeftY(), Constants.Controller.XBOX_DEADBAND)
            * Constants.Motor.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -MathUtils.modifyAxis(m_controller.getLeftX(), Constants.Controller.XBOX_DEADBAND)
            * Constants.Motor.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -MathUtils.modifyAxis(m_controller.getRightX(), Constants.Controller.XBOX_DEADBAND)
            * Constants.Motor.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Configure the button bindings
    configureDriveButtons();
    configureLEDButtons();

    DriverStation.silenceJoystickConnectionWarning(true);

    m_ledSubsystem.setStatus(() -> checkRoboRIO(), 0);
    m_ledSubsystem.setStatus(() -> m_intakeSubsystem.isOkay(), 1);
    m_ledSubsystem.setStatus(() -> m_ultrasonicSubsystem.isOkay(), 2);
    m_ledSubsystem.setStatus(() -> m_ledSubsystem.isOkay(), 3);
  }

  private void configureDriveButtons() {
    new Button(m_controller::getAButton)
        .whileHeld(() -> { // As class says don't go nathan mode
          m_drivetrainSubsystem.limitPower();
          m_controller.setLeftVibration(0.1);
          m_controller.setRightVibration(0.1);
        });
    new Button(m_controller::getDPadLeft)
        .whenPressed(new RotateToAngleCommand(m_drivetrainSubsystem, () -> Math.toRadians(-90)));
    new Button(m_controller::getDPadRight)
        .whenPressed(new RotateToAngleCommand(m_drivetrainSubsystem, () -> Math.toRadians(90)));
  }

  private void configureLEDButtons() {
    new Button(m_controller::getXButton)
        .whileHeld(() -> {
          m_ledSubsystem.setAll(MathUtils.random.nextInt(255), MathUtils.random.nextInt(255),
              MathUtils.random.nextInt(255));
        });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutonomousCommand(m_drivetrainSubsystem, m_autonomousSubsystem);
  }

  /**
   * Resets Gyro
   */
  public void reset() {
    m_drivetrainSubsystem.zeroGyro();
    m_ledSubsystem.setAll(255, 0, 0);
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

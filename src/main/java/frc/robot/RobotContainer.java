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
import frc.robot.commands.AimAtTargetCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.UserControllerCommand;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.subsystems.VisionSubsysten;
import frc.robot.utils.MathUtils;
import frc.robot.utils.Controller.XboxControllerExtended;

public class RobotContainer {
  // FIXME: Tell the vm to not ignore these unused classes
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsysten m_visionSubsystem = new VisionSubsysten(m_drivetrainSubsystem);
  private final AutonomousSubsystem m_autonomousSubsystem = new AutonomousSubsystem(m_drivetrainSubsystem);
  private final XboxControllerExtended m_controller = new XboxControllerExtended(0);
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

    if (!checkRoboRIO()) {
      DriverStation.reportWarning("Robot not properly enabled", false);
    }
  }

  private void configureDriveButtons() {
    // Start clears command and sets to default
    new Button(m_controller::getStartButton)
        .whenPressed(() -> {
          if (!m_drivetrainSubsystem.getCurrentCommand().equals(m_drivetrainSubsystem.getDefaultCommand())) {
            m_drivetrainSubsystem.getCurrentCommand().end(true);
          }
        });

    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
        .whenPressed(m_drivetrainSubsystem::zeroGyro);

    new Button(m_controller::getBButton)
        .whileHeld(new AimAtTargetCommand(m_drivetrainSubsystem, m_visionSubsystem));

    new Button(m_controller::getAButton)
        .whileHeld(new AutonomousCommand(m_drivetrainSubsystem, m_autonomousSubsystem));
  }

  private void configureLEDButtons() {
    new Button(m_controller::getXButton)
        .whileHeld(() -> {
          m_ledSubsystem.setLEDStripColor(MathUtils.random.nextInt(255), MathUtils.random.nextInt(255),
              MathUtils.random.nextInt(255));
        });
    new Button(m_controller::getBButton)
        .whileHeld(() -> {
          m_ledSubsystem.setStatusLEDColor(MathUtils.random.nextInt(255), MathUtils.random.nextInt(255),
              MathUtils.random.nextInt(255), 2);
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
    m_ledSubsystem.setLEDStripColor(255, 0, 0);
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

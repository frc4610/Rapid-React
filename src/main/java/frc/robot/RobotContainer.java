// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.UserControllerCommand;
import frc.robot.commands.RotationDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.MathController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController m_controller = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new UserControllerCommand(
            m_drivetrainSubsystem,
            () -> -MathController.modifyAxis(m_controller.getLeftY(), Constants.Controller.Y_AXIS_DEADBAND)
              * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -MathController.modifyAxis(m_controller.getLeftX(), Constants.Controller.X_AXIS_DEADBAND) 
              * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -MathController.modifyAxis(m_controller.getRightX(), Constants.Controller.Z_AXIS_DEADBAND) 
              * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope); // No requirements because we don't need to interrupt anything

    new Button(m_controller::getAButton)
            .whenPressed(m_drivetrainSubsystem::setTargetSideways); // No requirements because we don't need to interrupt anything

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // 1. Create trajectory settings
   /* TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND_SQRT)
              .setKinematics(m_drivetrainSubsystem.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
                  new Translation2d(1, 0),
                  new Translation2d(1, -1)),
          new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
          trajectoryConfig);


    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          trajectory,
          m_drivetrainSubsystem::getPose,
          m_drivetrainSubsystem::getKinematics,
          xController,
          yController,
          thetaController,
          m_drivetrainSubsystem::setModuleStates,
          m_drivetrainSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
          new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
          swerveControllerCommand,
          new InstantCommand(() -> m_drivetrainSubsystem.stopModules()));*/

    return new InstantCommand();
  }
}

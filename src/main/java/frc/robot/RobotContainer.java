// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.UserControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.utils.XboxControllerExtended;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxControllerExtended m_controller = new XboxControllerExtended(0);

  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(new UserControllerCommand(
            m_drivetrainSubsystem,
            () -> -MathUtils.modifyAxis(m_controller.getLeftY(), Constants.Controller.Y_AXIS_DEADBAND)
              * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -MathUtils.modifyAxis(m_controller.getLeftX(), Constants.Controller.X_AXIS_DEADBAND) 
              * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -MathUtils.modifyAxis(m_controller.getRightX(), Constants.Controller.Z_AXIS_DEADBAND) 
              * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Start clears command and sets to default
    new Button(m_controller::getStartButton)
         .whenPressed(()->{ 
            if(!m_drivetrainSubsystem.getCurrentCommand().equals( m_drivetrainSubsystem.getDefaultCommand())) {
              m_drivetrainSubsystem.getCurrentCommand().end(true);
            }
          });

    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope); // No requirements because we don't need to interrupt anything

    new Button(m_controller::getDPadRight)
            .whenPressed(()->{ 
              m_drivetrainSubsystem.setTargetHeading(Rotation2d.fromDegrees(90), false);
            });

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

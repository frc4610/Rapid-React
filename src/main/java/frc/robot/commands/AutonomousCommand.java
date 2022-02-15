package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

  public static enum Paths {
    PATH_ONE,
    PATH_TWO
  }

  private static PIDController m_xPIDController = new PIDController(Constants.Autonomous.PX_CONTROLLER, 0, 0);
  private static PIDController m_yPIDController = new PIDController(Constants.Autonomous.PX_CONTROLLER, 0, 0);
  private static ProfiledPIDController m_thetaPIDController = new ProfiledPIDController(
      Constants.Autonomous.PTHETA_CONTROLLER, 0, 0, Constants.Autonomous.PTHETA_CONTROLLER_CONSTRAINTS);

  private Paths m_path;
  private TrajectoryConfig m_config;

  public AutonomousCommand(DrivetrainSubsystem subsystem, Paths path) {
    m_path = path;
    m_thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_config = new TrajectoryConfig(
        Constants.Autonomous.MAX_VELOCITY_METERS_PER_SECOND,
        Constants.Autonomous.MAX_ACCELERATION_METERS_PER_SECOND)
            .setKinematics(subsystem.getKinematics());

    // TLDR: Use json data
    Trajectory trajectory = getTrajectory();

    // ------------------------Making Of driving
    // Commands----------------------------------//
    SwerveControllerCommand driveCommandController = new SwerveControllerCommand(
        trajectory,
        subsystem::getPose,
        subsystem.getKinematics(),
        m_xPIDController,
        m_yPIDController,
        m_thetaPIDController,
        subsystem::setModuleStates,
        subsystem);

    // ---------------------The Actual Command List That will Run-----------------//

    addCommands(
        new InstantCommand(() -> subsystem.resetPose(trajectory.getInitialPose())),
        driveCommandController);
  }

  public Trajectory getTrajectory() {
    // Inline switches?
    return TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        m_config);
  }
}
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

  private static PIDController m_xPIDController = new PIDController(Constants.Autonomous.PX_CONTROLLER, 0, 0);
  private static PIDController m_yPIDController = new PIDController(Constants.Autonomous.PX_CONTROLLER, 0, 0);
  private static ProfiledPIDController m_thetaPIDController = new ProfiledPIDController(
      Constants.Autonomous.PTHETA_CONTROLLER, 0, 0, Constants.Autonomous.PTHETA_CONTROLLER_CONSTRAINTS);

  public AutonomousCommand(DrivetrainSubsystem driveSubsystem, AutonomousSubsystem autoSubsystem) {
    m_thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);

    Trajectory trajectory = autoSubsystem.getTrajectory();

    SwerveControllerCommand driveCommandController = new SwerveControllerCommand(
        trajectory,
        driveSubsystem::getPose,
        driveSubsystem.getKinematics(),
        m_xPIDController,
        m_yPIDController,
        m_thetaPIDController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    // ---------------------The Actual Command List That will Run-----------------//

    addCommands(
        new InstantCommand(() -> driveSubsystem.resetPose(trajectory.getInitialPose())),
        driveCommandController);
  }
}
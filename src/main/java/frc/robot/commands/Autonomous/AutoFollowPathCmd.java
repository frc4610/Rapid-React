package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import beartecs.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoFollowPathCmd extends SequentialCommandGroup {
  private final PathPlannerTrajectory m_trajectory;
  private DrivetrainSubsystem m_drivetrainSubsystem;

  public AutoFollowPathCmd(String trajectoryPath, AutonomousSubsystem auto, DrivetrainSubsystem drive) {
    m_trajectory = PathPlanner.loadPath(trajectoryPath, Motor.MAX_VELOCITY_MPS, Motor.MAX_VELOCITY_MPS);
    if (m_trajectory == null)
      return;
    auto.showCurrentTrajectory(m_trajectory);
    m_drivetrainSubsystem = drive;
    addCommands(setup(), createTrajectoryFollowerCommand());
  }

  private PPSwerveControllerCommand createTrajectoryFollowerCommand() {
    PIDController xyController = Auto.PID_XY.getPidController();
    ProfiledPIDController thetaController = Auto.PID_THETA.getProfiledPidController();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new PPSwerveControllerCommand(
        m_trajectory,
        () -> m_drivetrainSubsystem.getPose(),
        m_drivetrainSubsystem.getKinematics(),
        xyController,
        xyController,
        thetaController,
        m_drivetrainSubsystem::setModuleStates,
        m_drivetrainSubsystem);
  }

  private InstantCommand setup() {
    return new InstantCommand(() -> {
      m_drivetrainSubsystem.resetPose(m_trajectory.getInitialPose());
    });
  }
}

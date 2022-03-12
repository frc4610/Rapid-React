package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoHoloFolowCmd extends CommandBase {

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final PIDController m_pidXYController = Auto.PID_XY_HDC.getPIDController();
  private final ProfiledPIDController m_pidRotController = Auto.PID_THETA.getProfiledPidController();
  private final HolonomicDriveController m_driveController = new HolonomicDriveController(
      m_pidXYController, m_pidXYController,
      m_pidRotController);
  private final Pose2d m_finalPose;
  private final double m_linearVelocity;

  public AutoHoloFolowCmd(DrivetrainSubsystem drivetrainSubsystem, Pose2d desiredPosition, double linearVelocity) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_finalPose = desiredPosition;
    m_linearVelocity = linearVelocity;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    ChassisSpeeds adjustedVelocities = m_driveController.calculate(
        m_drivetrainSubsystem.getPose(),
        m_finalPose,
        m_linearVelocity,
        m_finalPose.getRotation());

    m_drivetrainSubsystem.setModuleStates(adjustedVelocities);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveController.atReference();
  }

}
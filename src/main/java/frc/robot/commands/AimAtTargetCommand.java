package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsysten;

public class AimAtTargetCommand extends CommandBase {

  private final PIDController m_PIDController;
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final VisionSubsysten m_visionSubsystem;

  public AimAtTargetCommand(DrivetrainSubsystem driveSubsystem, VisionSubsysten visionSubsysten) {
    m_drivetrainSubsystem = driveSubsystem;
    m_visionSubsystem = visionSubsysten;
    m_PIDController = new PIDController(0.2, 0.0, 0.1); // from getFalcon500SteerFactory
    m_PIDController.setIntegratorRange(-Constants.Autonomous.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        Constants.Autonomous.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    m_PIDController.setTolerance(Math.toRadians(5.0));
    addRequirements(m_drivetrainSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_visionSubsystem.hasTarget())
      return;

    double rotation = m_PIDController.calculate(m_visionSubsystem.getRotationToTarget().getRadians(), 0);

    rotation *= Constants.Autonomous.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    m_drivetrainSubsystem.drive(0, 0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_visionSubsystem.isOnTarget();
  }

}
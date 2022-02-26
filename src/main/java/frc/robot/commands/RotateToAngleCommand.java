package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateToAngleCommand extends CommandBase {

  private final PIDController m_PIDController;
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final DoubleSupplier m_targetRadianAngle;

  public RotateToAngleCommand(DrivetrainSubsystem driveSubsystem, DoubleSupplier targetRadianAngle) {
    m_drivetrainSubsystem = driveSubsystem;
    m_targetRadianAngle = targetRadianAngle;
    m_PIDController = new PIDController(0.2, 0.0, 0.1); // from getFalcon500SteerFactory
    m_PIDController.setIntegratorRange(-Autonomous.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        Autonomous.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    m_PIDController.setTolerance(Math.toRadians(5.0));
    addRequirements(m_drivetrainSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rotation = m_PIDController.calculate(m_targetRadianAngle.getAsDouble(), 0);

    rotation *= Autonomous.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    m_drivetrainSubsystem.drive(0, 0, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_PIDController.atSetpoint();
  }
}
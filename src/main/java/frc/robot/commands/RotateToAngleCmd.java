package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateToAngleCmd extends CommandBase {

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final double m_angleRadian;

<<<<<<< Updated upstream
  private ProfiledPIDController m_rotationController = new ProfiledPIDController(Auto.PID_TURN.P, 0.0, 0.0,
      Auto.THETA_CONSTRAINTS);
=======
  private ProfiledPIDController m_rotationController = Auto.PID_THETA.getProfiledPidController();
>>>>>>> Stashed changes

  public RotateToAngleCmd(DrivetrainSubsystem drivetrain, double angleDegree) {
    m_drivetrainSubsystem = drivetrain;
    m_angleRadian = Math.toRadians(angleDegree);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationController.setTolerance(0.1, 0.1); // about 0.1 radians = 6 degrees, 6 deg/sec
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationController.reset(m_drivetrainSubsystem.getGyroRotation().getRadians());
    m_rotationController.setGoal(m_angleRadian);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationOutput = m_rotationController
        .calculate(m_drivetrainSubsystem.getGyroRotation().getRadians(), m_angleRadian);
    if (Math.abs(rotationOutput) < 0.05) {
      rotationOutput = 0.0;
    }

    m_drivetrainSubsystem.drive(RobotContainer.getDriveForwardAxis(), RobotContainer.getDriveStrafeAxis(),
        rotationOutput);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(RobotContainer.getDriveForwardAxis(), RobotContainer.getDriveStrafeAxis(), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotationController.atGoal();
  }

}
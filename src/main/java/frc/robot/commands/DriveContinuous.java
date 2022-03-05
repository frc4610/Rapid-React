package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveContinuous extends CommandBase {

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final boolean m_inverted;

  public DriveContinuous(DrivetrainSubsystem drivetrain, boolean inverted) {
    m_drivetrainSubsystem = drivetrain;
    m_inverted = inverted;
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(m_inverted ? -Autonomous.DRIVE_POWER : Autonomous.DRIVE_POWER, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
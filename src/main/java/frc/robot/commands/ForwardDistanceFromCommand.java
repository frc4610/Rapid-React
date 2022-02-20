package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;

public class ForwardDistanceFromCommand extends CommandBase {
  private final PIDController m_PIDController;
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final UltrasonicSubsystem m_ultrasonicSubsystem;
  // median filter to discard outliers; filters over 5 samples
  private final MedianFilter m_filter = new MedianFilter(5);

  public ForwardDistanceFromCommand(DrivetrainSubsystem drivetrain, UltrasonicSubsystem ultrasonicSubsysten,
      double wantedDistanceInch) {
    m_drivetrainSubsystem = drivetrain;
    m_ultrasonicSubsystem = ultrasonicSubsysten;
    m_PIDController = new PIDController(7.0, 0.018, 1.5); // get from getFalcon500SteerFactory
    m_PIDController.setSetpoint(wantedDistanceInch);
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_PIDController.calculate(m_filter.calculate(m_ultrasonicSubsystem.getUltrasonicDistance()));
    m_drivetrainSubsystem.drive(output, 0, 0, false);
  }
}

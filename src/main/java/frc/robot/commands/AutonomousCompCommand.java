package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousCompCommand extends SequentialCommandGroup {
  public AutonomousCompCommand(DrivetrainSubsystem driveSubsystem, AutonomousSubsystem autoSubsystem,
      IntakeSubsystem intakeSystem) {

    addCommands(
        new InstantCommand(() -> intakeSystem.autonomousIntakeEnable()),
        new WaitCommand(1),
        new InstantCommand(() -> intakeSystem.autonomousIntakeDisable()),
        new WaitCommand(0.69),
        new DriveContinuous(driveSubsystem, true),
        new WaitCommand(2),
        new InstantCommand(() -> driveSubsystem.stopModules()));
  }
}
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// Autonomous builder to handle Firing, and getting balls
public class AutoActionCmd extends SequentialCommandGroup {
  private AutonomousSubsystem m_autonomousSubystem;
  private DrivetrainSubsystem m_drivetrainSubystem;
  private IntakeSubsystem m_intakeSubystem;

  public AutoActionCmd(AutonomousSubsystem autonomousSubystem, DrivetrainSubsystem drivetrainSubsystem,
      IntakeSubsystem intakeSubystem) {
    m_autonomousSubystem = autonomousSubystem;
    m_drivetrainSubystem = drivetrainSubsystem;
    m_intakeSubystem = intakeSubystem;
    addRequirements(drivetrainSubsystem);
  }

  public AutoActionCmd executeCommand(CommandBase cmd) {
    addCommands(cmd);
    return this;
  }

  public AutoActionCmd executePause(double delayBeforeStart) {
    addCommands(new WaitCommand(delayBeforeStart));
    return this;
  }

  public AutoActionCmd executeIntakeFire() {
    addCommands(
        new InstantCommand(() -> m_intakeSubystem.autonomousIntakeState(true))
            .andThen(new WaitCommand(IntakeSubsystem.INTAKE_OUT_TIME))
            .andThen(new InstantCommand(() -> m_intakeSubystem.autonomousIntakeDisable())));
    return this;
  }

  public AutoActionCmd executeArmPosition(boolean state) {
    addCommands(
        new InstantCommand(() -> {
          if (state) {
            m_intakeSubystem.autonomousIntakeDisable();
            m_intakeSubystem.autonomousArmUp();
          } else {
            m_intakeSubystem.autonomousIntakeState(false);
            m_intakeSubystem.autonomousArmDown();
          }
        }).andThen(new WaitUntilCommand(m_intakeSubystem::hasFinishedTransition)));
    return this;
  }

  public AutoActionCmd executeIntakeArmUp() {
    addCommands(
        new InstantCommand(() -> m_intakeSubystem.autonomousIntakeDisable())
            .andThen(new InstantCommand(() -> m_intakeSubystem.autonomousArmUp())));
    return this;
  }

  public AutoActionCmd executeDrivePath(String pathPlanner) {
    addCommands(new AutoFollowPathCmd(pathPlanner, m_autonomousSubystem, m_drivetrainSubystem));
    return this;
  }

  public AutoActionCmd executeDrivePath(String pathPlanner, double delayBeforeStart) {
    addCommands(new WaitCommand(delayBeforeStart)
        .andThen(new AutoFollowPathCmd(pathPlanner, m_autonomousSubystem, m_drivetrainSubystem)));
    return this;
  }

  public AutoActionCmd executeAction(SubsystemAction action) {
    addCommands(actionToCommand(action));
    return this;
  }

  public AutoActionCmd executeAction(SubsystemAction action, double delayBeforeStart) {
    addCommands(new WaitCommand(delayBeforeStart)
        .andThen(actionToCommand(action)));
    return this;
  }

  public AutoActionCmd executeParallel(String pathPlanner, SubsystemAction action) {
    addCommands(
        new AutoFollowPathCmd(pathPlanner, m_autonomousSubystem, m_drivetrainSubystem)
            .alongWith(actionToCommand(action)));
    return this;
  }

  public AutoActionCmd executeParallel(String pathPlanner, SubsystemAction action, double delayBeforeStart) {
    addCommands(new WaitCommand(delayBeforeStart)
        .andThen(
            new AutoFollowPathCmd(pathPlanner, m_autonomousSubystem, m_drivetrainSubystem)
                .alongWith(actionToCommand(action))));
    return this;
  }

  public AutoActionCmd complete() {
    addCommands(actionToCommand((drivetrain, intake) -> {
      drivetrain.drive(0, 0, 0);
      intake.autonomousIntakeDisable();
      drivetrain.zeroGyro();
    }));
    return this;
  }

  private InstantCommand actionToCommand(SubsystemAction action) {
    return new InstantCommand(() -> action.doAction(m_drivetrainSubystem, m_intakeSubystem));
  }

  public interface SubsystemAction {
    void doAction(DrivetrainSubsystem drive, IntakeSubsystem intake);
  }

  @Override
  public void schedule() {
    super.schedule();
  }
}
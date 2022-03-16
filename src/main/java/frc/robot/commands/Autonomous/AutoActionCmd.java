package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  public AutoActionCmd executePause(double delayBeforeStart) {
    addCommands(new WaitCommand(delayBeforeStart));
    return this;
  }

  public AutoActionCmd executeIntakeFire() {
    addCommands(
        new InstantCommand(() -> m_intakeSubystem.autonomousIntakeFireEnable())
            .andThen(new WaitCommand(1))
            .andThen(new InstantCommand(() -> m_intakeSubystem.autonomousIntakeFireDisable())));
    return this;
  }

  public AutoActionCmd executeIntakeEnable(){
    addCommands(
        new InstantCommand(() -> m_intakeSubystem.autonomousIntakeEnable())
            .andThen(new InstantCommand(() -> m_intakeSubystem.autonomousArmDown())));
    return this;
  }

  public AutoActionCmd executeIntakeDisable(){
    addCommands(
      new InstantCommand(() -> m_intakeSubystem.autonomousIntakeFireDisable())
          .andThen(new InstantCommand(() -> m_intakeSubystem.autonomousArmUp())));
    return this;
  }

  public AutoActionCmd executeFeildReset(){
    addCommands(
      new InstantCommand(() -> m_drivetrainSubystem.autonomousFeildReset()));
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
      intake.autonomousIntakeFireDisable();
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
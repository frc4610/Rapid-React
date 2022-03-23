package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.commands.Autonomous.AutoActionCmd;
import frc.robot.utils.BaseSubsystem;

public class AutonomousSubsystem extends BaseSubsystem {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final SendableChooser<AutoActionCmd> m_autoChooser = new SendableChooser<AutoActionCmd>();

  public AutonomousSubsystem(DrivetrainSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    m_drivetrainSubsystem = driveSubsystem;
    m_intakeSubsystem = intakeSubsystem;

    loadAutoActionCmds();
    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  public void loadAutoActionCmds() {

    m_autoChooser.setDefaultOption("1m Forward", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeDrivePath("1m Forward") // went 87 cm
        .complete());
    m_autoChooser.addOption("1 Ball Auto", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeIntakeFire()
        .executeAction((drivetrain, intake) -> drivetrain.drive(-Auto.DRIVE_POWER, 0, 0, false))
        .executePause(1.69)
        .complete());
    m_autoChooser.addOption("Path", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeIntakeFire()
        .executeDrivePath("New Path")
        .executePause(1.69)
        .complete());
  }

  public Optional<AutoActionCmd> getAutoCmd() {
    return Optional.ofNullable(m_autoChooser.getSelected());
  }

  public void showCurrentTrajectory(Trajectory trajectory) {
    RobotContainer.telemetry.setTrajectory(trajectory);
  }
}
package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.PathPlanner;

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

    m_autoChooser.setDefaultOption("1 Ball Auto", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeIntakeFire()
        .executeAction((drivetrain, intake) -> drivetrain.drive(-Auto.DRIVE_POWER, 0, 0, false))
        .executePause(1.69)
        .complete());
    m_autoChooser.addOption("Path", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeIntakeFire()
        .executeDrivePath("New Path")
        .executePause(1.69)
        .complete());
    m_autoChooser.addOption("4BallAutoLeft", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
    .executeIntakeEnable()
    .executeDrivePath("4BallAutoFirstStage")
    .executeIntakeDisable()
    .executeDrivePath("4BallAutoSecondStage")
    .executeIntakeFire()
    .executePause(1.69)
    .executeIntakeEnable()
    .executeDrivePath("4BallAutoThirdStage")
    .executeDrivePath("4BallAutoFourthStage")
    .executeIntakeDisable()
    .executeDrivePath("4BallAutoFithStage")
    .executeIntakeFire()
    .executePause(1.69)
    .executeDrivePath("4BallAutoSixthStage")
    .executeFeildReset()
    .complete());

    m_autoChooser.addOption("4BallAutoRight",  new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
    .executeIntakeEnable()
    .executeDrivePath("4BallAutoFirstStageRight")
    .executeIntakeDisable()
    .executeDrivePath("4BallAutoSecondStageRight")
    .executeIntakeFire()
    .executePause(1.69)
    .executeIntakeEnable()
    .executeDrivePath("4BallAutoThirdStage")
    .executeIntakeDisable()
    .executeDrivePath("4BallAutoFourthStage")
    .executeIntakeFire()
    .executePause(1.69)
    .executeDrivePath("4BallAutoFithStage")
    .executeFeildReset()
    .complete());

    m_autoChooser.addOption("HangarDump", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
    .executeIntakeEnable()
    .executeDrivePath("HangarDump1")
    .executeIntakeDisable()
    .executeDrivePath("HangarDump2")
    .executeIntakeFire()
    .executePause(1.69)
    .executeIntakeEnable()
    .executeDrivePath("HangarDump3")
    .executeIntakeDisable()
    .executePause(1.69)
    .executeIntakeFire()
    .complete()
    );
  }

  public Optional<AutoActionCmd> getAutoCmd() {
    return Optional.ofNullable(m_autoChooser.getSelected());
  }

  public void showCurrentTrajectory(Trajectory trajectory) {
    RobotContainer.dashboardField.getObject("Trajectory").setTrajectory(trajectory);
  }
}
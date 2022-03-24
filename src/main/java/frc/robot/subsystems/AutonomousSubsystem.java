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
    m_autoChooser.addOption("4BallAutoLeft", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeIntakeArmDown()
        .executeDrivePath("4BallAutoFirstStage")
        .executeIntakeArmUp()
        .executeDrivePath("4BallAutoSecondStage")
        .executeIntakeFire()
        .executePause(1.69)
        .executeIntakeArmDown()
        .executeDrivePath("4BallAutoThirdStage")
        .executeDrivePath("4BallAutoFourthStage")
        .executeIntakeArmUp()
        .executeDrivePath("4BallAutoFithStage")
        .executeIntakeFire()
        .executePause(1.69)
        .executeDrivePath("4BallAutoSixthStage")
        .complete());

    m_autoChooser.addOption("4BallAutoRight", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeIntakeArmDown()
        .executeDrivePath("4BallAutoFirstStageRight")
        .executeIntakeArmUp()
        .executeDrivePath("4BallAutoSecondStageRight")
        .executeIntakeFire()
        .executePause(1.69)
        .executeIntakeArmDown()
        .executeDrivePath("4BallAutoThirdStage")
        .executeIntakeArmUp()
        .executeDrivePath("4BallAutoFourthStage")
        .executeIntakeFire()
        .executePause(1.69)
        .executeDrivePath("4BallAutoFithStage")
        .complete());

    m_autoChooser.addOption("HangarDump", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeIntakeArmDown()
        .executeDrivePath("HangarDump1")
        .executeIntakeArmUp()
        .executeDrivePath("HangarDump2")
        .executeIntakeFire()
        .executePause(1.69)
        .executeIntakeArmDown()
        .executeDrivePath("HangarDump3")
        .executeIntakeArmUp()
        .executePause(1.69)
        .executeIntakeFire()
        .executePause(1)
        .executeDrivePath("HangarDump4")
        .complete());

    m_autoChooser.addOption("2BallLeft", new AutoActionCmd(this, m_drivetrainSubsystem, m_intakeSubsystem)
        .executeIntakeArmDown()
        .executeDrivePath("2BallLeft1")
        .executeIntakeArmUp()
        .executeDrivePath("2BallLeft2")
        .executeIntakeFire()
        .executePause(1.69)
        .executeDrivePath("2BallLeft3")
        .complete());
  }

  public Optional<AutoActionCmd> getAutoCmd() {
    return Optional.ofNullable(m_autoChooser.getSelected());
  }

  public void showCurrentTrajectory(Trajectory trajectory) {
    RobotContainer.telemetry.setTrajectory(trajectory);
  }
}
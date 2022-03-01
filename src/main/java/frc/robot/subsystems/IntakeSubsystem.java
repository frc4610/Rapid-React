package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ids;
import frc.robot.utils.Controller.XboxControllerExtended;
import oblog.annotations.Config;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_intake = new WPI_TalonFX(Ids.INTAKE);
  private final WPI_TalonFX m_arm = new WPI_TalonFX(Ids.PIVOT);
  private final XboxControllerExtended m_controller;

  // FIXME: Config these values

  @Config(tabName = "intakeSubsystem")
  private final double m_armHoldUpPower = 0.08;
  @Config(tabName = "intakeSubsystem")
  private final double m_armHoldDownPower = 0.13;
  @Config(tabName = "intakeSubsystem")
  private final double m_armTravelPower = 0.5;

  @Config(tabName = "intakeSubsystem")
  private final double m_armTimeUp = 0.5;
  @Config(tabName = "intakeSubsystem")
  private final double m_armTimeDown = 0.35;

  @Config(tabName = "intakeSubsystem")
  private final double m_intakePower = 0.4;

  private boolean m_armUp = true;
  private double m_lastBurstTime = 0;

  public IntakeSubsystem(XboxControllerExtended controller) {
    m_controller = controller;
    m_arm.setInverted(false);
    m_arm.setNeutralMode(NeutralMode.Brake); // Force Motor in brake mode
    m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // Talon FX encoder
  }

  public double getArmEncoderValue() {
    // gets encoder value in degrees
    return m_arm.getSelectedSensorPosition() * (360 / 4096);
  }

  @Override
  public void periodic() {
    // Intake controls
    if (m_controller.getLeftBumper()) {
      m_intake.set(ControlMode.PercentOutput, m_intakePower);
    } else if (m_controller.getRightBumper()) {
      m_intake.set(ControlMode.PercentOutput, -m_intakePower);
    } else {
      m_intake.set(ControlMode.PercentOutput, 0);
    }

    // Arm Controls
    if (m_armUp) {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeUp) {
        m_arm.set(m_armTravelPower);
      } else {
        m_arm.set(m_armHoldUpPower);
      }
    } else {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeDown) {
        m_arm.set(-m_armTravelPower);
      } else {
        m_arm.set(-m_armHoldDownPower);
      }
    }

    if (m_controller.getRightTriggerAxis() > 0 && !m_armUp) {
      m_lastBurstTime = Timer.getFPGATimestamp();
      m_armUp = true;
    } else if (m_armUp) {
      m_lastBurstTime = Timer.getFPGATimestamp();
      m_armUp = false;
    }
  }
}
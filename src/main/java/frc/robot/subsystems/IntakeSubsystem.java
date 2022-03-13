package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.*;
import frc.robot.utils.BaseSubsystem;
import frc.robot.utils.controller.XboxControllerExtended;
import frc.robot.utils.math.MathUtils;

public class IntakeSubsystem extends BaseSubsystem {

  private final WPI_TalonFX m_intake = new WPI_TalonFX(Ids.INTAKE);
  private final WPI_TalonFX m_arm = new WPI_TalonFX(Ids.ARM);
  private final XboxControllerExtended m_controller;
  DigitalInput m_topLimitSwitch = new DigitalInput(3);
  DigitalInput m_bottomLimitSwitch = new DigitalInput(2);

  private final double m_armTimeUp = 0.83;
  private final double m_armTimeDown = 0.4;

  // 38991 when up
  // 1555 when down

  // true == up false == down
  private boolean m_armState = true;
  private boolean m_verifiedArmState = true;
  private double m_lastBurstTime = 0;
  private boolean m_autoControl = false;
  private ShuffleboardTab m_intakeTab;

  public IntakeSubsystem(XboxControllerExtended controller) {
    m_controller = controller;
    m_arm.setInverted(false);
    m_arm.setNeutralMode(NeutralMode.Brake); // Force Motor in brake mode
    m_arm.setSelectedSensorPosition(Arm.ABS_UP_POSITION);

    m_armState = updateArmState();
    m_intakeTab = Shuffleboard.getTab("IntakeSubsystem");
    m_intakeTab.addBoolean("Arm State", () -> m_armState);
    m_intakeTab.addBoolean("Verified Arm State", () -> m_verifiedArmState);
    m_intakeTab.addNumber("Arm Selected Position", () -> m_arm.getSelectedSensorPosition());
  }

  @Override
  public boolean isOkay() {
    return true;
  }

  public boolean getArmState() {
    return m_armState;
  }

  public boolean getVerifiedArmState() {
    return m_verifiedArmState;
  }

  public boolean updateArmState() {
    if (m_controller.getStartButton()) {
      m_arm.setSelectedSensorPosition(Arm.ABS_UP_POSITION);
    }
    if (m_arm.getSelectedSensorPosition() > Arm.UP_POSITION) {
      m_verifiedArmState = true;
    } else if (m_arm.getSelectedSensorPosition() > Arm.DOWN_POSITION) {
      m_verifiedArmState = false;
    }
    return m_verifiedArmState;
  }

  public void updateIntake() {
    if (m_controller.getLeftTriggerAxis() > 0) {
      m_intake.set(ControlMode.PercentOutput,
          m_controller.getBackButton() ? 1
              : -MathUtils.clamp(m_controller.getLeftTriggerAxis(), 0.0,
                  Intake.POWER_OUT.getDouble(0.8)));
    } else if (!m_verifiedArmState && m_controller.getRightTriggerAxis() > 0) {
      m_intake.set(ControlMode.PercentOutput, MathUtils.clamp(m_controller.getRightTriggerAxis(), 0.0,
          Intake.POWER_IN.getDouble(0.45)));
    } else {
      m_intake.set(ControlMode.PercentOutput, 0);
    }
  }

  public void autonomousIntakeFireEnable() {
    m_autoControl = true;
    m_intake.set(ControlMode.PercentOutput, -0.35);
  }

  public void autonomousIntakeFireDisable() {
    m_autoControl = false;
    m_intake.set(ControlMode.PercentOutput, 0);
  }

  public void updateArm() {
    final boolean rightBumper = m_controller.getRightBumper();
    final boolean rightTriggerAxis = m_controller.getRightTriggerAxis() > 0 || rightBumper;

    if (m_armState) {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeUp) {
        m_arm.set(Arm.TRAVEL_UP_POWER.getDouble(Arm.DEFAULT_TRAVEL_UP_POWER));
      } else if (m_topLimitSwitch.get() == false) {
        m_arm.set(Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE));
      } else {
        m_arm.set(0);
      }
    } else {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeDown) {
        m_arm.set(-Arm.TRAVEL_DOWN_POWER.getDouble(Arm.DEFAULT_TRAVEL_DOWN_POWER));
      } else if (m_bottomLimitSwitch.get() == false) {
        m_arm.set(-Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE));
      }
      else if (m_bottomLimitSwitch.get() == true){
        m_arm.set(0);
      }
    }

    if (rightTriggerAxis && m_armState) {
      if (m_verifiedArmState)
        m_lastBurstTime = Timer.getFPGATimestamp();
      m_armState = false;
    } else if (!rightTriggerAxis && !m_armState) {
      if (!m_verifiedArmState) // Fixes spamming right trigger causing motor stall
        m_lastBurstTime = Timer.getFPGATimestamp();
      m_armState = true;
    }
  }

  @Override
  public void periodic() {
    updateArmState();
    if (!m_autoControl) {
      updateIntake();
      updateArm();
    }
  }
}
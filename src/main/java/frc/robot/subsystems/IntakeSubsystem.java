package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.*;
import frc.robot.utils.BaseSubsystem;
import frc.robot.utils.controller.XboxControllerExtended;
import frc.robot.utils.math.MathUtils;

public class IntakeSubsystem extends BaseSubsystem {

  private final WPI_TalonFX m_intake = new WPI_TalonFX(Ids.INTAKE);
  private final WPI_TalonFX m_arm = new WPI_TalonFX(Ids.ARM);
  private final XboxControllerExtended m_controller;
  private final DigitalInput m_topLimitSwitch = new DigitalInput(Ids.DIO_TOP_LIMITSWTICH); // currently broken
  private final DigitalInput m_bottomLimitSwitch = new DigitalInput(Ids.DIO_BOTTOM_LIMITSWTICH);

  private final double m_armTimeUp = 0.83;
  private final double m_armTimeDown = 0.4;
  // 38991 when up
  // 1555 when down

  // true == up false == down
  private boolean m_requestedArmState = true;
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

    m_requestedArmState = m_armState = updateArmState();
    m_intakeTab = addTab("IntakeSubsystem");
    m_intakeTab.addBoolean("Requested Arm State", () -> m_requestedArmState);
    m_intakeTab.addBoolean("Arm State", () -> m_armState);
    m_intakeTab.addBoolean("Verified Arm State", () -> m_verifiedArmState);
    m_intakeTab.addNumber("Arm Selected Position", () -> m_arm.getSelectedSensorPosition());
  }

  @Override
  public boolean isOkay() {
    return true;
  }

  public boolean getRequestedArmState() {
    return m_requestedArmState;
  }

  public boolean getArmState() {
    return m_armState;
  }

  public boolean getVerifiedArmState() {
    return m_verifiedArmState;
  }

  public boolean isAtRequestedPosition() {
    return m_requestedArmState == m_verifiedArmState;
  }

  public boolean updateArmState() {
    if (m_controller.getStartButton()) {
      m_arm.setSelectedSensorPosition(Arm.ABS_UP_POSITION);
    } else if (m_arm.getSelectedSensorPosition() < -1) {
      m_arm.setSelectedSensorPosition(0); // stop from going negative
    }

    // the arm has reached the ideal arm position
    if (m_arm.getSelectedSensorPosition() > Arm.UP_POSITION) {
      m_verifiedArmState = true; // is at top
    } else if (m_bottomLimitSwitch.get() || m_arm.getSelectedSensorPosition() < Arm.DOWN_POSITION) {
      m_verifiedArmState = false; // is at bottom
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
    m_intake.set(ControlMode.PercentOutput, -0.6);
  }

  public void autonomousIntakeFireDisable() {
    m_autoControl = false;
    m_intake.set(ControlMode.PercentOutput, 0);
  }

  public void autonomousIntakeEnable() {
    m_autoControl = true;
    m_intake.set(ControlMode.PercentOutput, 0.35); // spline based off time and velocity // Mr Gibson said it's how bots are effective
  }

  public void autonomousArmDown() {
    m_autoControl = true;
    m_requestedArmState = false;
  }

  public void autonomousArmUp() {
    m_autoControl = false;
    m_requestedArmState = true;
  }

  public void updateArm() {
    final boolean rightBumper = m_controller.getRightBumper();
    final boolean rightTriggerAxis = getRobotMode() == RobotMode.AUTO ? m_requestedArmState
        : m_controller.getRightTriggerAxis() > 0 || rightBumper; // if we want to control the arm using request then set it

    if (m_armState) {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeUp) {
        m_arm.set(Arm.TRAVEL_UP_POWER.getDouble(Arm.DEFAULT_TRAVEL_UP_POWER)); // from the bottom up power
      } else if (!m_verifiedArmState) {
        m_arm.set(Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE)); // not in up position apply power till we get there
      } else {
        m_arm.set(0); // stop the power // we don't need to supply power to it as it "locks" itself in a pivot
      }
    } else {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeDown) {
        m_arm.set(-Arm.TRAVEL_DOWN_POWER.getDouble(Arm.DEFAULT_TRAVEL_DOWN_POWER)); // from the top down power
      } else if (!m_bottomLimitSwitch.get()) { // not hit the limit switch use distance power
        m_arm.set(-Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE));
      } else if (m_bottomLimitSwitch.get()) { // hit limit switch no longer want to use distance but hold down
        m_arm.set(-Arm.HOLD_DOWN_POWER.getDouble(Arm.DEFAULT_HOLD_DOWN_POWER));
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
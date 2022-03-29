package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.Constants.*;
import beartecs.controller.XboxControllerExtended;
import beartecs.math.MathUtils;
import beartecs.template.PositionSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IntakeSubsystem extends PositionSubsystem {
  private final WPI_TalonFX m_intake = new WPI_TalonFX(Ids.INTAKE);
  private final XboxControllerExtended m_controller;
  private final DigitalInput m_topLimitSwitch = new DigitalInput(Ids.DIO_TOP_LIMITSWTICH); // currently broken
  private final DigitalInput m_bottomLimitSwitch = new DigitalInput(Ids.DIO_BOTTOM_LIMITSWTICH);

  public static final double INTAKE_OUT_TIME = 1;
  private final double m_armTimeUp = 0.85;
  private final double m_armTimeDown = 0.45;

  private boolean m_armState = true;
  private boolean m_verifiedArmState = true;
  private boolean m_armPositionChanged = false;
  private double m_lastArmPosition = 0;
  private double m_lastBurstTime = 0;
  private double m_autoIntakeSpeed = 0;
  private ShuffleboardTab m_intakeTab;

  public IntakeSubsystem(XboxControllerExtended controller) {
    super(Ids.ARM);
    m_controller = controller;
    m_armState = updateArmState();
    m_intakeTab = addTab("IntakeSubsystem");

    // TODO: Group
    m_intakeTab.addBoolean("Top Switch", () -> getTopLimitSwitch());
    m_intakeTab.addBoolean("Bottom Switch", () -> getBottomLimitSwitch());
    // TODO: Group
    m_intakeTab.addBoolean("Arm State", () -> m_armState);
    m_intakeTab.addBoolean("Verified Arm State", () -> m_verifiedArmState);
    m_intakeTab.addNumber("Arm Selected Position", () -> this.getEncoderPosition());
    m_intakeTab.addBoolean("Arm Changed", () -> m_armPositionChanged);
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

  public boolean hasFinishedTransition() {
    return m_armState == m_verifiedArmState;
  }

  public boolean getBottomLimitSwitch() {
    return !m_bottomLimitSwitch.get();
  }

  public boolean getTopLimitSwitch() {
    return !m_topLimitSwitch.get();
  }

  public boolean shouldGoUp() {
    return m_controller.getRightTriggerAxis() > 0 || m_controller.getRightBumper();
  }

  public boolean shouldGoDown() {
    return !shouldGoUp() || m_controller.getLeftBumper();
  }

  public double getIntakePower() {
    if (getRobotMode() == RobotMode.AUTO) {
      return m_autoIntakeSpeed;
    } else if (m_controller.getLeftTriggerAxis() > 0) {
      return -MathUtils.clamp(m_controller.getBackButton() ? 1
          : m_controller.getLeftTriggerAxis(), 0.0,
          Intake.POWER_OUT.getDouble(0.8));
    } else if (!m_verifiedArmState && m_controller.getRightTriggerAxis() > 0) {
      return MathUtils.clamp(
          m_controller.getRightTriggerAxis(),
          0.0,
          Intake.POWER_IN.getDouble(0.45));
    } else {
      return 0.0;
    }
  }

  public boolean updateArmState() {
    if (m_controller.getStartButton()) {
      this.setEncoder(Arm.ABS_UP_POSITION);
    } else if (this.getEncoderPosition() < -1) {
      this.resetEncoder(); // stop from going negative
    }

    // Arm encoder position
    if (m_lastArmPosition != this.getEncoderPosition()) {
      m_armPositionChanged = true;
      m_lastArmPosition = this.getEncoderPosition();
    } else {
      m_armPositionChanged = false;
    }
    // the arm has reached the ideal arm position
    if (getTopLimitSwitch() || this.getEncoderPosition() > Arm.UP_POSITION) {
      m_verifiedArmState = true; // is at top
    } else if (getBottomLimitSwitch() || this.getEncoderPosition() < Arm.DOWN_POSITION) {
      m_verifiedArmState = false; // is at bottom
    }
    return m_verifiedArmState;
  }

  public void updateIntake() {
    m_intake.set(ControlMode.PercentOutput, getIntakePower());
  }

  // [true == fire] [false == intake]
  public void autonomousIntakeState(boolean state) {
    if (state) {
      m_autoIntakeSpeed = -0.6;
    } else if (!state) {
      m_autoIntakeSpeed = 0.35;
    }
  }

  public void autonomousIntakeDisable() {
    m_autoIntakeSpeed = 0;
  }

  public void autonomousArmDown() {
    m_armState = false;
  }

  public void autonomousArmUp() {
    m_armState = true;
  }

  public void updateArm() {
    final boolean rightBumper = m_controller.getRightBumper();
    final boolean leftBumper = m_controller.getLeftBumper();

    if (m_armState) {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeUp) {
        this.setManualOutput(Arm.TRAVEL_UP_POWER.getDouble(Arm.DEFAULT_TRAVEL_UP_POWER)); // from the bottom up power
      } else if (!m_verifiedArmState || leftBumper) {
        this.setManualOutput(Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE)); // not in up position apply power till we get there
      } else {
        this.stop(); // stop the power // we don't need to supply power to it as it "locks" itself in a pivot
      }
    } else {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeDown) {
        this.setManualOutput(-Arm.TRAVEL_DOWN_POWER.getDouble(Arm.DEFAULT_TRAVEL_DOWN_POWER)); // from the top down power
      } else if (m_verifiedArmState || rightBumper) { // not hit the limit switch use distance power
        this.setManualOutput(-Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE));
      }
    }

    if (shouldGoUp() && m_armState) {
      if (m_verifiedArmState)
        m_lastBurstTime = Timer.getFPGATimestamp();
      m_armState = false;
    } else if (shouldGoDown() && !m_armState) {
      if (!m_verifiedArmState) // Fixes spamming right trigger causing motor stall
        m_lastBurstTime = Timer.getFPGATimestamp();
      m_armState = true;
    }
  }

  @Override
  public void periodic() {
    updateArmState();
    updateIntake();
    updateArm();
  }
}
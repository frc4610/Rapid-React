package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.template.*;
import beartecs.controller.XboxControllerExtended;
import beartecs.math.MathUtils;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import beartecs.Constants.*;

public class IntakeSubsystem extends BaseSubsystem {

  private final WPI_TalonFX m_intake = new WPI_TalonFX(Ids.INTAKE.deviceNumber, Ids.INTAKE.canBus);
  private final WPI_TalonFX m_arm = new WPI_TalonFX(Ids.ARM.deviceNumber, Ids.ARM.canBus);
  private final XboxControllerExtended m_controller;
  private final DigitalInput m_topLimitSwitch = new DigitalInput(Ids.DIO_TOP_LIMITSWTICH); // currently broken
  private final DigitalInput m_bottomLimitSwitch = new DigitalInput(Ids.DIO_BOTTOM_LIMITSWTICH);

  public static final double INTAKE_OUT_TIME = 1;
  private final double m_armTimeUp = 0.85;
  private final double m_armTimeDown = 0.45;
  // 38991 when up
  // 1555 when down

  private boolean m_armState = true;
  private boolean m_verifiedArmState = true;
  private boolean m_armPositionChanged = false;
  private double m_lastArmPosition = 0;
  private double m_lastBurstTime = 0;
  private double m_autoIntakeSpeed = 0;
  private double m_encoderHoldPosition = 0;
  private boolean m_lastTopLimitSwitch = false;
  private boolean m_lastBottomLimitSwitch = false;
  private ProfiledPIDController m_armPidController = Arm.ARM_PID.getProfiledPidController();

  public IntakeSubsystem(XboxControllerExtended controller) {
    m_controller = controller;

    m_arm.setInverted(false);
    m_arm.setNeutralMode(NeutralMode.Brake); // Force Motor in brake mode
    m_arm.enableVoltageCompensation(true);
    m_arm.setSelectedSensorPosition(Arm.ABS_UP_POSITION);
    m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_arm.configMotionCruiseVelocity(700, 0);
    m_arm.configMotionAcceleration(5000, 0);

    m_armState = updateArmState();
    ShuffleboardTab tab = addTab("IntakeSubsystem");
    ShuffleboardLayout layout = tab.getLayout("Intake ", BuiltInLayouts.kGrid)
        .withSize(2, 1)
        .withPosition(1, 1);
    layout.addBoolean("Top Switch", () -> getTopLimitSwitch());
    layout.addBoolean("Bottom Switch", () -> getBottomLimitSwitch());
    layout.addBoolean("Arm State", () -> m_armState);
    layout.addBoolean("Verified Arm State", () -> m_verifiedArmState);
    layout.addNumber("Arm Position", () -> m_arm.getSelectedSensorPosition());
    layout.addNumber("Relative Position", () -> getRelativeJointPosition());
    layout.addBoolean("Arm Changed", () -> m_armPositionChanged);
  }

  public double getRelativeJointPosition() {
    return m_arm.getSelectedSensorPosition() * Arm.GEAR_RATIO;
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
      m_arm.setSelectedSensorPosition(Arm.ABS_UP_POSITION);
    } else if (m_arm.getSelectedSensorPosition() < -1) {
      m_arm.setSelectedSensorPosition(0); // stop from going negative
    }

    // Arm encoder position
    if (m_lastArmPosition != m_arm.getSelectedSensorPosition()) {
      m_armPositionChanged = true;
      m_lastArmPosition = m_arm.getSelectedSensorPosition();
    } else {
      m_armPositionChanged = false;
    }
    if (getTopLimitSwitch() != m_lastTopLimitSwitch) {
      if (getTopLimitSwitch())
        m_encoderHoldPosition = m_arm.getSelectedSensorPosition();
      m_lastTopLimitSwitch = getTopLimitSwitch();

    } else if (getBottomLimitSwitch() != m_lastBottomLimitSwitch) {
      if (getBottomLimitSwitch())
        m_encoderHoldPosition = m_arm.getSelectedSensorPosition();
      m_lastBottomLimitSwitch = getBottomLimitSwitch();
    }

    // the arm has reached the ideal arm position
    if (getTopLimitSwitch() || m_arm.getSelectedSensorPosition() > Arm.UP_POSITION) {
      m_verifiedArmState = true; // is at top
    } else if (getBottomLimitSwitch() || m_arm.getSelectedSensorPosition() < Arm.DOWN_POSITION) {
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
        m_arm.set(Arm.TRAVEL_UP_POWER.getDouble(Arm.DEFAULT_TRAVEL_UP_POWER)); // from the bottom up power
      } else if (!m_verifiedArmState || leftBumper) {
        m_arm.set(Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE)); // not in up position apply power till we get there
      } else {
        m_arm.set(0); // stop the power // we don't need to supply power to it as it "locks" itself in a pivot
      }
    } else {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeDown) {
        m_arm.set(-Arm.TRAVEL_DOWN_POWER.getDouble(Arm.DEFAULT_TRAVEL_DOWN_POWER)); // from the top down power
      } else if (m_verifiedArmState || rightBumper) { // not hit the limit switch use distance power
        m_arm.set(-Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE));
      } else if (getBottomLimitSwitch()) { // Hold while on limit switch
        //m_arm.set(-m_armPidController.calculate(m_arm.getSelectedSensorPosition(), m_encoderHoldPosition)); // Configure offset from inital switch
        /*if (m_arm.getSelectedSensorPosition() > m_encoderHoldPosition) {
          m_arm.set(-Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE));
        } else {
          m_arm.set(0);
        }*/
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
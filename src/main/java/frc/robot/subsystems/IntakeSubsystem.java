package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.math.MathUtils;
import beartecs.math.MotorUtils;
import beartecs.systems.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import beartecs.Constants.*;
import beartecs.configs.GearRatioConfig;

public class IntakeSubsystem extends BaseSubsystem {

  private final WPI_TalonFX m_intake = new WPI_TalonFX(Ids.INTAKE.deviceNumber, Ids.INTAKE.canBus);
  private final WPI_TalonFX m_arm = new WPI_TalonFX(Ids.ARM.deviceNumber, Ids.ARM.canBus);
  private final DigitalInput m_topLimitSwitch = new DigitalInput(Ids.DIO_TOP_LIMITSWTICH); // currently broken
  private final DigitalInput m_bottomLimitSwitch = new DigitalInput(Ids.DIO_BOTTOM_LIMITSWTICH);

  public static final double INTAKE_OUT_TIME = 1;
  private final double m_armTimeUp = 0.85;
  private final double m_armTimeDown = 0.45;

  private boolean m_armState = true;
  private boolean m_verifiedArmState = true;
  private double m_lastBurstTime = 0;
  private double m_autoIntakeSpeed = 0;
  private boolean m_velocityIntake = false;
  private ProfiledPIDController m_armPidController = Arm.ARM_PID.getProfiledPidController();

  private final GearRatioConfig m_gearRatioConfig = new GearRatioConfig(
      MotorUtils.TALON_TICK_RESOLUTION,
      MotorUtils.TALON_MAX_RPM,
      Arm.GEAR_RATIO);

  public IntakeSubsystem() {
    m_arm.setInverted(false);
    m_arm.setNeutralMode(NeutralMode.Brake); // Force Motor in brake mode
    m_arm.enableVoltageCompensation(true);
    m_arm.setSelectedSensorPosition(Arm.ABS_UP_POSITION);
    m_arm.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_arm.configMotionCruiseVelocity(700, 0);
    m_arm.configMotionAcceleration(5000, 0);

    m_armState = updateArmState();
    ShuffleboardTab tab = addTab("IntakeSubsystem");
    ShuffleboardLayout layout = tab.getLayout("Intake", BuiltInLayouts.kGrid)
        .withSize(2, 1)
        .withPosition(1, 1);
    layout.addBoolean("Top Switch", () -> getTopLimitSwitch());
    layout.addBoolean("Bottom Switch", () -> getBottomLimitSwitch());
    layout.addBoolean("Arm State", () -> m_armState);
    layout.addBoolean("Verified Arm State", () -> m_verifiedArmState);
    layout.addNumber("Arm Position", () -> m_arm.getSelectedSensorPosition());
    layout.addNumber("Joint Rotation", () -> getJointRotation().getDegrees());
  }

  public Rotation2d getJointRotation() {
    return Rotation2d.fromDegrees(m_gearRatioConfig.toDegrees(m_arm.getSelectedSensorPosition()));
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

  public double getIntakePower() {
    if (getRobotMode() == RobotMode.AUTO) {
      return m_autoIntakeSpeed;
    } else if (Controls.getIntakeFireSpeed() > 0) {
      return -MathUtils.clamp(Controls.getIntakeFireSpeed(), 0.0,
          Intake.POWER_OUT.getDouble(Intake.DEFAULT_POWER_OUT));
    } else if (!m_verifiedArmState && Controls.getIntakeSpeed() > 0) {
      return MathUtils.clamp(
          Controls.getIntakeSpeed(),
          0.0,
          Intake.POWER_IN.getDouble(Intake.DEFAULT_POWER_IN));
    } else {
      return 0.0;
    }
  }

  public double getIntakeVelocity() {
    return RobotContainer.getDrivetrain().getAccelerationMeters()
        * RobotContainer.getDrivetrain().getAccelerationMeters();
  }

  public boolean updateArmState() {
    if (Controls.getIntakeResetEncoderButton()) {
      m_arm.setSelectedSensorPosition(Arm.ABS_UP_POSITION);
    } else if (m_arm.getSelectedSensorPosition() < 0) {
      m_arm.setSelectedSensorPosition(0); // stop from going negative
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
    if (!m_velocityIntake) {
      m_intake.set(ControlMode.PercentOutput, getIntakePower());
    } else {
      m_intake.set(ControlMode.Velocity, getIntakeVelocity());
    }
  }

  public void updateArm() {

    if (m_armState) {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeUp) {
        m_arm.set(Arm.TRAVEL_UP_POWER.getDouble(Arm.DEFAULT_TRAVEL_UP_POWER)); // from the bottom up power
      } else if (!m_verifiedArmState || Controls.getIntakeForceUp()) {
        m_arm.set(Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE)); // not in up position apply power till we get there
      } else {
        m_arm.set(0); // stop the power // we don't need to supply power to it as it "locks" itself in a pivot
      }
    } else {
      if (Timer.getFPGATimestamp() - m_lastBurstTime < m_armTimeDown) {
        m_arm.set(-Arm.TRAVEL_DOWN_POWER.getDouble(Arm.DEFAULT_TRAVEL_DOWN_POWER)); // from the top down power
      } else if (m_verifiedArmState || Controls.getIntakeForceDown()) { // not hit the limit switch use distance power
        m_arm.set(-Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE));
      } else if (getBottomLimitSwitch() || getJointRotation().getDegrees() < 12) {
        m_arm.set(MathUtils.clamp(
            m_armPidController.calculate(getJointRotation().getDegrees(), 0),
            -Arm.TRAVEL_DIFFERENCE.getDouble(Arm.DEFAULT_TRAVEL_DISTANCE),
            0.0)); // use this to hold down arm without stalling
      }
    }

    if (Controls.getIntakeRequestedUp() && m_armState) {
      if (m_verifiedArmState)
        m_lastBurstTime = Timer.getFPGATimestamp();
      m_armState = false;
    } else if (Controls.getIntakeRequestedDown() && !m_armState) {
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
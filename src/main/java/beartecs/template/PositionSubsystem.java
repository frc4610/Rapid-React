package beartecs.template;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.RunCommand;

public abstract class PositionSubsystem extends BaseSubsystem {

  public WPI_TalonFX m_motor;

  public PositionSubsystem(int talonCanId) {
    m_motor = new WPI_TalonFX(talonCanId);
    setDefaultCommand(new RunCommand(() -> this.stop(), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setManualOutput(double speed) {
    m_motor.set(speed);
  }

  public void setMagicMotionPosition(double position) {
    m_motor.set(ControlMode.MotionMagic, position);
  }

  public void setVoltageOutput(double voltage) {
    m_motor.setVoltage(voltage);
  }

  public void stop() {
    m_motor.set(0);
  }

  // Return Radians per sec velocity
  public double getCharacterizationVelocity() {
    return (getRotationPerSec() * 2 * Math.PI);
  }

  // Return Rotations per sec velocity
  public double getRotationPerSec() {
    return (m_motor.getSelectedSensorVelocity() / 2048) * 10;
  }

  //Return RPM velocity
  public double getRPM() {
    return getRotationPerSec() * 60;
  }

  //Get Position
  public double getEncoderPosition() {
    return m_motor.getSelectedSensorPosition();
  }

  //Reset Encoder
  public void resetEncoder() {
    setEncoder(0);
  }

  public void configFeedbackDevice(FeedbackDevice device) {
    m_motor.configSelectedFeedbackSensor(device, 0, 10);
  }

  public void enableSoftLimits(boolean fwdEnable, boolean revEnable) {
    m_motor.configForwardSoftLimitEnable(fwdEnable);
    m_motor.configReverseSoftLimitEnable(revEnable);
  }

  //Set Encoder
  public void setEncoder(double position) {
    m_motor.setSelectedSensorPosition(position);
  }

  public double getCurrent() {
    return m_motor.getSupplyCurrent();
  }

  //Set Brake Mode
  public void setBrakeMode(boolean mode) {
    m_motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
  }
}

package beartecs.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.configs.GearRatioConfig;
import beartecs.configs.LimitConfig;
import beartecs.configs.ProfiledPidConfig;
import beartecs.math.MathUtils;
import beartecs.math.MotorUtils;
import beartecs.swerve.Gyroscope;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class TurretSystem {
  private final WPI_TalonFX m_turretMotor;
  private final Gyroscope m_gyroscope;
  private final Limelight m_vision;
  private final double m_targetHeight;
  private final double m_maxAngle; // Lime 180
  private final double m_minAngle; // Lime -180
  private final GearRatioConfig m_gearRatio;
  private final double m_startAngle;
  private final double m_tickTolerance;
  private double m_errorOffset;
  private int m_errorCounter;
  private final ProfiledPIDController m_rotationController;

  public TurretSystem(
      WPI_TalonFX motor,
      Gyroscope gyro,
      Limelight vision,
      double targetHeight,
      GearRatioConfig gearRatio,
      LimitConfig limitConfig,
      double startAngle,
      double tickTolerance,
      boolean canSpinFreely,
      ProfiledPidConfig pidConfig) {
    m_turretMotor = motor;
    m_gyroscope = gyro;
    m_targetHeight = targetHeight;
    m_vision = vision;
    m_gearRatio = gearRatio;
    m_maxAngle = limitConfig.MAX;
    m_minAngle = limitConfig.MIN;
    m_startAngle = startAngle;
    m_tickTolerance = tickTolerance; // Smaller number less then res
    m_rotationController = pidConfig.getProfiledPidController();

    if (canSpinFreely) {
      m_rotationController.enableContinuousInput(m_maxAngle, m_minAngle);
    } else {
      m_rotationController.disableContinuousInput();
    }

    MotorUtils.setSoftLimits(
        m_gearRatio.fromDegrees(m_maxAngle),
        m_gearRatio.fromDegrees(m_minAngle),
        m_turretMotor);
    m_turretMotor.setSensorPhase(false);
    m_turretMotor.setNeutralMode(NeutralMode.Brake);
  }

  public double getPositionTicks() {
    return m_turretMotor.getSelectedSensorPosition();
  }

  public double getVelocity() {
    return m_turretMotor.getSelectedSensorVelocity();
  }

  /*
    * Get the angle of the turret, where zero is straight forward (relative).
    * @param isAbsolute if the angle should consider the robot heading.
    */
  public double getAngle(boolean isAbsolute) {
    double relAngle = m_gearRatio.toDegrees(getPositionTicks()) - m_startAngle;
    return isAbsolute ? MathUtils.angleWrap(m_gyroscope.getGyroRotation().getDegrees() + relAngle) : relAngle;
  }

  /*
  * @param extraDistance physics util extra shooting distance.
  * @param hoodAngle angle of the hood in degrees.
  * @return a double array containing a distance and angle.
  */
  public double[] getTurretCalculations(double extraDistance, double hoodAngle) {
    double initialD = m_targetHeight
        / Math.tan(Math.toRadians(m_vision.getTargetPosition().y + (hoodAngle)));
    double tx = Math.toRadians(m_vision.getTargetPosition().x);
    double xInitialD = Math.sin(tx) * initialD;
    double yInitialD = Math.cos(tx) * initialD + extraDistance;
    double dist = Math.sqrt(xInitialD * xInitialD + yInitialD * yInitialD);
    tx = Math.atan(xInitialD / yInitialD);
    return new double[] { dist, MathUtils.angleWrap(Math.toDegrees(tx) + getAngle(true)) };
  }

  /*
  * Set the position of the turret. Uses position PID loop configuration.
  *
  * @param positionTicks the position to set in ticks.
  */
  public void setPositionTicks(double positionTicks) {
    positionTicks -= m_errorOffset;
    m_turretMotor.set(ControlMode.MotionMagic, positionTicks);
  }

  /*
  * Set the angle of the turret. Uses position PID loop configuration.
  * Wraps set value to be within safety bounds and take the shortest
  * distance/path. Will consider robot yaw if absolute
  *
  * @param angle the angle to set in degrees.
  * @param isAbsolute if the set angle should be absolute.
  */
  public void setAngle(double angle, boolean isAbsolute) {
    double initialTicks = getTargetTicks(angle, isAbsolute);
    m_turretMotor.set(ControlMode.MotionMagic, initialTicks);
  }

  public double getTargetTicks(double angle, boolean isAbsolute) {
    angle %= 360;
    if (isAbsolute) {
      angle -= m_gyroscope.getGyroRotation().getDegrees();
    }
    double min = m_gearRatio.fromDegrees(m_startAngle)
        + m_gearRatio.fromDegrees(m_minAngle);
    double max = m_gearRatio.fromDegrees(m_startAngle)
        + m_gearRatio.fromDegrees(m_maxAngle);
    double initialTicks = m_gearRatio.fromDegrees(m_startAngle + angle);
    double ticks = MathUtils.clamp(initialTicks, min, max);
    if (ticks == max) {
      initialTicks -= MotorUtils.TALON_TICK_RESOLUTION;
    } else if (ticks == min) {
      initialTicks += MotorUtils.TALON_TICK_RESOLUTION;
    }
    initialTicks = MathUtils.clamp(initialTicks, min, max);
    if (initialTicks == max || initialTicks == min) {
      initialTicks = getPositionTicks() - m_gearRatio.fromDegrees(m_startAngle) < 0 ? ticks : max;
    }
    initialTicks -= m_errorOffset;
    return initialTicks;
  }

  /**
  * Checks for turret overshoot / undershoot encoder issue.
  * Encoder will jump +/- 4096 ticks (full rotation) during runtime.
  * Fixes by offsetting returned values and modifying soft limits.
  */
  private void checkWrapError() {
    final double lastWrapOffset = m_errorOffset;
    final double currPos = m_turretMotor.getSelectedSensorPosition();
    boolean isOverRotated = currPos > m_gearRatio.fromDegrees(m_maxAngle) + m_tickTolerance;
    boolean isUnderRotated = currPos < m_gearRatio.fromDegrees(m_minAngle) - m_tickTolerance;
    this.m_errorOffset = isOverRotated ? -MotorUtils.TALON_TICK_RESOLUTION
        : isUnderRotated ? MotorUtils.TALON_TICK_RESOLUTION : 0;

    if (lastWrapOffset != m_errorOffset) {
      MotorUtils.setSoftLimits(
          m_gearRatio.fromDegrees(m_maxAngle) - m_errorOffset,
          m_gearRatio.fromDegrees(m_minAngle) - m_errorOffset,
          m_turretMotor);
    }
  }

  public void onPeriodic() {
    // Runs every 0.4s after init (0.4s/0.02s=20x)
    m_errorCounter++;
    if (m_errorCounter >= 20) {
      checkWrapError();
      m_errorCounter = 0;
    }
  }
}

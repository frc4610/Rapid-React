package beartecs.math;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import beartecs.configs.PidConfig;

public class MotorUtils {
  public static final double TALON_TICK_RESOLUTION = 2048.0; // Ticks per Rotation
  public static final int TALON_MAX_RPM = 6380;
  public static final int FRAME_PERIOD_MS = 20;
  public static final int TIMEOUT_MS = 10;

  @SafeVarargs
  public static <T extends BaseTalon> void setPIDF(PidConfig pidConfig, int slotIdx, T... motors) {
    for (T motor : motors) {
      motor.config_kP(slotIdx, pidConfig.P);
      motor.config_kI(slotIdx, pidConfig.I);
      motor.config_kD(slotIdx, pidConfig.D);
      motor.config_kF(slotIdx, pidConfig.F);
    }
  }

  /**
   * Places limits on the range of CTRE Talon, Victor, or Falcon controlled motors.
   *
   * <p>Limits are defined in ticks and apply to both power and positional control sets.
   * It is still recommended that both are limited manually if possible. Note that
   * this is a hard stop (despite being a soft limit) and does not account for velocity
   * accumulated while moving. This value does not persist after power-off.</p>
   *
   * @param forward the maximum ticks in the forward/positive direction.
   * @param reverse the minimum ticks in the reverse/backward/negative direction.
   * @param motors  the motors to apply the soft limits onto.
   */
  @SafeVarargs
  public static <T extends BaseTalon> void setSoftLimits(double forward, double reverse, T... motors) {
    for (T motor : motors) {
      motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
      motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
      motor.configForwardSoftLimitThreshold(forward);
      motor.configReverseSoftLimitThreshold(reverse);
      motor.configForwardSoftLimitEnable(true);
      motor.configReverseSoftLimitEnable(true);
    }
  }

  /**
    * Configures Motion Magic motion profiling on given CTRE
    * Talon, Victor, or Falcon controlled motors.
    *
    * <p>Uses the internal 1 kHz clock of the controller instead of the 20 ms
    * RoboRio clock. This is recommended as it removes the need to make
    * custom motion profiles, leading to faster turnaround times on subsystems.
    * As a warning, this first resets all motor settings to factory default
    * and then configures the feedback sensor based on the passed value.
    * As such is is recommended that this be the first motor configuration call
    * in any subsystem. Note that maximum velocity and acceleration will
    * not be set if {@code maxVel == 0}.</p>
    *
    * @param sensor    the sensor attached to the controller used for loop feedback.
    * @param pidConfig the PIDF and range values to use on the controller.
    * @param maxVel    maximum velocity of the profile in ticks per 100ms.
    * @param motors    the motors for which Motion Magic is enabled on.
    */
  @SafeVarargs
  public static <T extends BaseTalon> void setupMotionMagic(FeedbackDevice sensor, PidConfig pidConfig, int maxVel,
      T... motors) {
    for (T motor : motors) {
      motor.configFactoryDefault();
      motor.selectProfileSlot(0, 0);
      motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, FRAME_PERIOD_MS, TIMEOUT_MS);
      motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, FRAME_PERIOD_MS, TIMEOUT_MS);
      if (sensor == FeedbackDevice.PulseWidthEncodedPosition || sensor == FeedbackDevice.IntegratedSensor) {
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, FRAME_PERIOD_MS, TIMEOUT_MS);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, FRAME_PERIOD_MS, TIMEOUT_MS);
      }
      motor.configSelectedFeedbackSensor(sensor, 0, TIMEOUT_MS);

      motor.configNominalOutputForward(0);
      motor.configNominalOutputReverse(0);
      motor.configPeakOutputForward(pidConfig.RANGE);
      motor.configPeakOutputReverse(-pidConfig.RANGE);

      setPIDF(pidConfig, 0, motor);

      if (maxVel != 0) {
        motor.configMotionCruiseVelocity(maxVel);
        motor.configMotionAcceleration(maxVel);
      }
    }
  }

}
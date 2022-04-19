package beartecs.swerve.ctre;

import static beartecs.swerve.ctre.CtreUtils.checkCtreError;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.Constants;
import beartecs.math.MotorUtils;
import beartecs.swerve.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Robot;

public final class Falcon500SteerControllerFactoryBuilder {
    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    // PID configuration
    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

    // Motion magic configuration
    private double velocityConstant = Double.NaN;
    private double accelerationConstant = Double.NaN;
    private double staticConstant = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;
    private String canBusName = "rio";

    public Falcon500SteerControllerFactoryBuilder withPidConstants(double proportional, double integral,
            double derivative) {
        this.proportionalConstant = proportional;
        this.integralConstant = integral;
        this.derivativeConstant = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant)
                && Double.isFinite(derivativeConstant);
    }

    public Falcon500SteerControllerFactoryBuilder withMotionMagic(double velocityConstant, double accelerationConstant,
            double staticConstant) {
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
        return this;
    }

    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant)
                && Double.isFinite(staticConstant);
    }

    public Falcon500SteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public Falcon500SteerControllerFactoryBuilder withCanBusName(String canBusName) {
        this.canBusName = canBusName;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public Falcon500SteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public <T> SteerControllerFactory<ControllerImplementation, Falcon500SteerConfiguration<T>> build(
            AbsoluteEncoderFactory<T> absoluteEncoderFactory) {
        return new FactoryImplementation<>(absoluteEncoderFactory);
    }

    private class FactoryImplementation<T>
            implements SteerControllerFactory<ControllerImplementation, Falcon500SteerConfiguration<T>> {
        private final AbsoluteEncoderFactory<T> encoderFactory;

        private FactoryImplementation(AbsoluteEncoderFactory<T> encoderFactory) {
            this.encoderFactory = encoderFactory;
        }

        @Override
        public void addDashboardEntries(ShuffleboardContainer container, ControllerImplementation controller) {
            SteerControllerFactory.super.addDashboardEntries(container, controller);
            final double absoluteAngle = controller.absoluteEncoder.getAbsoluteAngle();
            if (!Double.isNaN(absoluteAngle)) {
                if (container != null) {
                    container.addNumber("Absolute Encoder Angle",
                            () -> Math.toDegrees(controller.absoluteEncoder.getAbsoluteAngle()));
                }
            }
        }

        @Override
        public ControllerImplementation create(Falcon500SteerConfiguration<T> steerConfiguration,
                ModuleConfiguration moduleConfiguration) {
            AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());

            final double sensorPositionCoefficient = 2.0 * Math.PI / MotorUtils.TALON_TICK_RESOLUTION
                    * moduleConfiguration.getSteerReduction();
            final double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            if (hasPidConstants()) {
                motorConfiguration.slot0.kP = proportionalConstant;
                motorConfiguration.slot0.kI = integralConstant;
                motorConfiguration.slot0.kD = derivativeConstant;
            }
            if (hasMotionMagic()) {
                if (hasVoltageCompensation()) {
                    motorConfiguration.slot0.kF = (1023.0 * sensorVelocityCoefficient / nominalVoltage)
                            * velocityConstant;
                }
                motorConfiguration.motionCruiseVelocity = 2.0 / velocityConstant / sensorVelocityCoefficient;
                motorConfiguration.motionAcceleration = (8.0 - 2.0) / accelerationConstant / sensorVelocityCoefficient;
            }
            if (hasVoltageCompensation()) {
                motorConfiguration.voltageCompSaturation = nominalVoltage;
            }
            if (hasCurrentLimit()) {
                motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
                motorConfiguration.supplyCurrLimit.enable = true;
            }

            WPI_TalonFX motor = new WPI_TalonFX(steerConfiguration.getMotorPort(), canBusName);
            checkCtreError(motor.configAllSettings(motorConfiguration, CAN_TIMEOUT_MS),
                    "Failed to configure Falcon 500 settings");

            if (hasVoltageCompensation()) {
                motor.enableVoltageCompensation(true);
            }
            checkCtreError(
                    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
                    "Failed to set Falcon 500 feedback sensor");
            motor.setSensorPhase(!moduleConfiguration.isSteerInverted());
            motor.setInverted(moduleConfiguration.isSteerInverted() ? TalonFXInvertType.CounterClockwise
                    : TalonFXInvertType.Clockwise);
            motor.setNeutralMode(NeutralMode.Brake); // Meant to be in brake

            double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
            if (Double.isNaN(absoluteAngle)) {
                absoluteAngle = 0.0;
            }

            checkCtreError(
                    motor.setSelectedSensorPosition(absoluteAngle / sensorPositionCoefficient, 0, CAN_TIMEOUT_MS),
                    "Failed to set Falcon 500 encoder position");

            // Reduce CAN status frame rates
            CtreUtils.checkCtreError(
                    motor.setStatusFramePeriod(
                            StatusFrameEnhanced.Status_1_General,
                            Robot.isSimulation() ? Constants.Sim.STATUS_FRAME_PERIOD_MS
                                    : STATUS_FRAME_GENERAL_PERIOD_MS,
                            CAN_TIMEOUT_MS),
                    "Failed to configure Falcon status frame period");

            return new ControllerImplementation(motor,
                    sensorPositionCoefficient,
                    sensorVelocityCoefficient,
                    hasMotionMagic() ? TalonFXControlMode.MotionMagic : TalonFXControlMode.Position,
                    absoluteEncoder);
        }
    }

    private static class ControllerImplementation implements SteerController {
        private static final int ENCODER_RESET_ITERATIONS = 500;
        private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

        private final WPI_TalonFX motor;
        private final double motorEncoderPositionCoefficient;
        private final double motorEncoderVelocityCoefficient;
        private final TalonFXControlMode motorControlMode;
        private final AbsoluteEncoder absoluteEncoder;

        private double referenceAngleRadians = 0.0;

        private double resetIteration = 0;

        private ControllerImplementation(WPI_TalonFX motor,
                double motorEncoderPositionCoefficient,
                double motorEncoderVelocityCoefficient,
                TalonFXControlMode motorControlMode,
                AbsoluteEncoder absoluteEncoder) {
            this.motor = motor;
            this.motorEncoderPositionCoefficient = motorEncoderPositionCoefficient;
            this.motorEncoderVelocityCoefficient = motorEncoderVelocityCoefficient;
            this.motorControlMode = motorControlMode;
            this.absoluteEncoder = absoluteEncoder;
        }

        @Override
        public Object getSteerMotor() {
            return this.motor;
        }

        @Override
        public AbsoluteEncoder getSteerEncoder() {
            return this.absoluteEncoder;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;

            // Reset the NEO's encoder periodically when the module is not rotating.
            // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
            // end up getting a good reading. If we reset periodically this won't matter anymore.
            if (Constants.Swerve.ENABLE_ABS_SET_MOTOR) {
                if (Robot.isReal() && motor.getSelectedSensorVelocity()
                        * motorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                    if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                        resetIteration = 0;
                        final double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                        if (!Double.isNaN(absoluteAngle)) {
                            motor.setSelectedSensorPosition(absoluteAngle / motorEncoderPositionCoefficient);
                            currentAngleRadians = absoluteAngle;
                        }
                    }
                } else {
                    resetIteration = 0;
                }
            }

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            motor.set(motorControlMode, adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);
            if (RobotBase.isSimulation()) {
                motor.getSimCollection().setIntegratedSensorRawPosition(
                        (int) (adjustedReferenceAngleRadians / motorEncoderPositionCoefficient));
            }

            this.referenceAngleRadians = referenceAngleRadians;
        }

        @Override
        public double getStateAngle() {
            double motorAngleRadians = motor.getSelectedSensorPosition() * motorEncoderPositionCoefficient;
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }

        @Override
        public double getOutputVoltage() {
            return motor.getMotorOutputVoltage();
        }

        @Override
        public void setSteerEncoder(double position, double velocity) {
            motor.getSimCollection().setIntegratedSensorRawPosition((int) (position * motorEncoderPositionCoefficient));
            motor.getSimCollection()
                    .setIntegratedSensorVelocity((int) (velocity / 600 * motorEncoderPositionCoefficient));
        }
    }
}

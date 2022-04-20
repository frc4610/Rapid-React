package beartecs.swerve.ctre;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.Constants;
import beartecs.math.MotorUtils;
import beartecs.swerve.DriveController;
import beartecs.swerve.DriveControllerFactory;
import beartecs.swerve.ModuleConfiguration;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;
    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;
    private String canBusName = "rio";

    public Falcon500DriveControllerFactoryBuilder withPidConstants(double proportional, double integral,
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

    public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public Falcon500DriveControllerFactoryBuilder withCanBusName(String canBusName) {
        this.canBusName = canBusName;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction() / MotorUtils.TALON_TICK_RESOLUTION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0; // 100ms of a second

            if (hasPidConstants()) {
                motorConfiguration.slot0.kP = proportionalConstant;
                motorConfiguration.slot0.kI = integralConstant;
                motorConfiguration.slot0.kD = derivativeConstant;
                motorConfiguration.slot0.kF = 0.5;
            }

            if (hasVoltageCompensation()) {
                motorConfiguration.voltageCompSaturation = nominalVoltage;
            }

            if (hasCurrentLimit()) {
                motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
                motorConfiguration.supplyCurrLimit.enable = true;
            }

            WPI_TalonFX motor = new WPI_TalonFX(driveConfiguration, canBusName);
            motor.configAllSettings(motorConfiguration);
            CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");

            if (hasVoltageCompensation()) {
                // Enable voltage compensation
                motor.enableVoltageCompensation(true);
            }

            motor.setNeutralMode(NeutralMode.Brake);

            motor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise
                    : TalonFXInvertType.CounterClockwise);
            motor.setSensorPhase(true);

            CtreUtils.checkCtreError(
                    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MS),
                    "Failed to set Falcon 500 feedback sensor");

            // Reduce CAN status frame rates
            CtreUtils.checkCtreError(
                    motor.setStatusFramePeriod(
                            StatusFrameEnhanced.Status_1_General,
                            Robot.isSimulation() ? Constants.Sim.STATUS_FRAME_PERIOD_MS
                                    : STATUS_FRAME_GENERAL_PERIOD_MS,
                            CAN_TIMEOUT_MS),
                    "Failed to configure Falcon status frame period");

            return new ControllerImplementation(motor, sensorVelocityCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final WPI_TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation()
                ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage
                : 12.0;

        private ControllerImplementation(WPI_TalonFX motor, double sensorVelocityCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
        }

        @Override
        public Object getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setVelocity(double velocity) {
            motor.set(TalonFXControlMode.Velocity, velocity / sensorVelocityCoefficient);
            if (RobotBase.isSimulation()) {
                if (motor.getInverted()) {
                    velocity *= -1.0;
                }
                motor.getSimCollection()
                        .setIntegratedSensorVelocity((int) (velocity / sensorVelocityCoefficient));
            }
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            double percentOutput = voltage / nominalVoltage;
            motor.set(TalonFXControlMode.PercentOutput, percentOutput);

            if (RobotBase.isSimulation()) {
                if (motor.getInverted()) {
                    percentOutput *= -1.0;
                }
                motor.getSimCollection()
                        .setIntegratedSensorVelocity((int) (percentOutput / 600 * sensorVelocityCoefficient));
            }
        }

        @Override
        public double getVelocity() {
            double sensorVelocity = motor.getSelectedSensorVelocity();
            if (sensorVelocity < 0) {
                sensorVelocity *= 0.98;
            }
            return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
        }

        @Override
        public double getOutputVoltage() {
            return motor.getMotorOutputVoltage();
        }

        @Override
        public void setDriveEncoder(double position, double velocity) {
            // Position is in revolutions.  Velocity is in RPM
            // CANCoder wants steps for postion.  Steps per 100ms for velocity
            motor.getSimCollection().setIntegratedSensorRawPosition((int) (position * sensorVelocityCoefficient));
            // Divide by 600 to go from RPM to Rotations per 100ms.  Multiply by encoder ticks per revolution.
            motor.getSimCollection().setIntegratedSensorVelocity((int) (velocity / 600 * sensorVelocityCoefficient));
        }

        @Override
        public void resetEncoder() {
            motor.setSelectedSensorPosition(0);
        }

        @Override
        public void configRampRate(double rampRate) {
            this.motor.configOpenloopRamp(rampRate);
        }

    }
}

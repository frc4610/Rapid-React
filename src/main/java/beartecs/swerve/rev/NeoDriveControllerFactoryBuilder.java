package beartecs.swerve.rev;

import static beartecs.swerve.rev.RevUtils.checkNeoError;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import beartecs.swerve.DriveController;
import beartecs.swerve.DriveControllerFactory;
import beartecs.swerve.ModuleConfiguration;

public final class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;
    private double pidProportional = Double.NaN;
    private double pidIntegral = Double.NaN;
    private double pidDerivative = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public NeoDriveControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.pidProportional = proportional;
        this.pidIntegral = integral;
        this.pidDerivative = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(pidProportional) && Double.isFinite(pidIntegral) && Double.isFinite(pidDerivative);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer id, ModuleConfiguration moduleConfiguration) {
            CANSparkMax motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
            motor.setInverted(moduleConfiguration.isDriveInverted());

            // Setup voltage compensation
            if (hasVoltageCompensation()) {
                checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
            }

            if (hasCurrentLimit()) {
                checkNeoError(motor.setSmartCurrentLimit((int) currentLimit), "Failed to set current limit for NEO");
            }

            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
                    "Failed to set periodic status frame 0 rate");
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
                    "Failed to set periodic status frame 1 rate");
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
                    "Failed to set periodic status frame 2 rate");
            // Set neutral mode to brake
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

            // Setup encoder
            RelativeEncoder encoder = motor.getEncoder();
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction();
            encoder.setPositionConversionFactor(positionConversionFactor);
            encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);

            SparkMaxPIDController controller = motor.getPIDController();
            if (hasPidConstants()) {
                controller.setP(pidProportional);
                controller.setI(pidIntegral);
                controller.setD(pidDerivative);
            }
            controller.setFeedbackDevice(encoder);

            return new ControllerImplementation(motor, encoder);
        }
    }

    private static class ControllerImplementation implements DriveController {
        private final CANSparkMax motor;
        private final RelativeEncoder encoder;
        private final SparkMaxPIDController controller;

        private ControllerImplementation(CANSparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
            this.controller = motor.getPIDController();
        }

        @Override
        public Object getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setVelocity(double velocity) {
            controller.setReference(velocity, ControlType.kVelocity);
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getVelocity() {
            return encoder.getVelocity();
        }

        @Override
        public double getOutputVoltage() {
            return motor.getBusVoltage() * motor.getAppliedOutput();
        }

        @Override
        public void resetEncoder() {
            encoder.setPosition(0);
        }

        @Override
        public void setDriveEncoder(double position, double velocity) {
            motor.getEncoder().setPosition(position);
        }

        @Override
        public void configRampRate(double rampRate) {
            this.motor.setOpenLoopRampRate(rampRate);
        }
    }
}

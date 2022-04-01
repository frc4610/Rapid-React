package beartecs.swerve.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import beartecs.swerve.AbsoluteEncoder;
import beartecs.swerve.AbsoluteEncoderFactory;
import beartecs.Constants;
import beartecs.math.MathUtils;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;
    private String canBusName = "rio";

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = MathUtils.clamp(periodMilliseconds, 10, 200);
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public CanCoderFactoryBuilder withCanBusName(String canivoreName) {
        this.canBusName = canivoreName;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
            config.sensorDirection = direction == Direction.CLOCKWISE;
            config.initializationStrategy = configuration.getInitStrategy();

            final var encoderId = configuration.getId();
            if (encoderId != -1) {
                WPI_CANCoder encoder = new WPI_CANCoder(configuration.getId(), canBusName);
                CtreUtils.checkCtreError(encoder.configAllSettings(config, 30), "Failed to configure CANCoder");

                CtreUtils.checkCtreError(
                        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 30),
                        "Failed to configure CANCoder update rate");
                return new EncoderImplementation(encoder);
            } else {
                return null;
            }

        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final WPI_CANCoder encoder;

        private EncoderImplementation(WPI_CANCoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition());
            /* 
                This will attempt to verify the abs encoder rather then software
                Due to high can bus usage this can sometimes cause a hang so don't loop only have a set amount of tries
                
            */
            if (Constants.ENABLE_ABS_ENCODER_POS_ERROR_CHECKS) {
                ErrorCode code = encoder.getLastError();

                for (int i = 0; i < Constants.ABS_ENCODER_ERROR_RETRY_COUNT; i++) {
                    if (code == ErrorCode.OK)
                        break;
                    angle = Math.toRadians(encoder.getAbsolutePosition());
                    code = encoder.getLastError();
                }

                CtreUtils.checkCtreError(code, "Failed to retrieve CANcoder " + encoder.getDeviceID()
                        + " absolute position after " + Constants.ABS_ENCODER_ERROR_RETRY_COUNT + " tries");
            }
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public void setAbsoluteEncoder(double position, double velocity, double motorEncoderPositionCoefficient) {
            // Position is in revolutions.  Velocity is in RPM
            // CANCoder wants steps for postion.  Steps per 100ms for velocity
            encoder.getSimCollection().setRawPosition((int) (position * motorEncoderPositionCoefficient));
            // Divide by 600 to go from RPM to Rotations per 100ms.  Multiply by encoder ticks per revolution.
            encoder.getSimCollection().setVelocity((int) (velocity / 600 * motorEncoderPositionCoefficient));
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}

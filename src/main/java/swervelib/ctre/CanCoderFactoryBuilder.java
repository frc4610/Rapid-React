package swervelib.ctre;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import swervelib.AbsoluteEncoder;
import swervelib.AbsoluteEncoderFactory;
import swervelib.Constants;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
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
                CANCoder encoder = new CANCoder(configuration.getId());
                CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

                CtreUtils.checkCtreError(
                        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250),
                        "Failed to configure CANCoder update rate");
                return new EncoderImplementation(encoder);
            } else {
                return new AbsoluteEncoder() {
                    @Override
                    public double getAbsoluteAngle() {
                        return Double.NaN;
                    }
                };
            }

        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANCoder encoder;

        private EncoderImplementation(CANCoder encoder) {
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
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}

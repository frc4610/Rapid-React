package swervelib.ctre;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import swervelib.Constants;

public class CanCoderAbsoluteConfiguration {
    private final int id;
    private final double offset;
    private final SensorInitializationStrategy initStrategy;

    public CanCoderAbsoluteConfiguration(int id, double offset) {
        this(id, offset, Constants.BOOT_TO_ABS ? SensorInitializationStrategy.BootToAbsolutePosition
                : SensorInitializationStrategy.BootToZero);
    }

    public CanCoderAbsoluteConfiguration(int id, double offset, SensorInitializationStrategy initStrategy) {
        this.id = id;
        this.offset = offset;
        this.initStrategy = initStrategy;
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }

    public SensorInitializationStrategy getInitStrategy() {
        return initStrategy;
    }
}

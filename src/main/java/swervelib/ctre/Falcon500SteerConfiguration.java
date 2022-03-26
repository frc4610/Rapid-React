package swervelib.ctre;

import java.util.Objects;

public class Falcon500SteerConfiguration<EncoderConfiguration> {
    private final int motorPort;
    private final EncoderConfiguration encoderConfiguration;
    private String canivoreName = "";

    public Falcon500SteerConfiguration(int motorPort, EncoderConfiguration encoderConfiguration) {
        this(motorPort, encoderConfiguration, "");
    }

    public Falcon500SteerConfiguration(int motorPort, EncoderConfiguration encoderConfiguration, String canivoreName) {
        this.motorPort = motorPort;
        this.encoderConfiguration = encoderConfiguration;
        this.canivoreName = canivoreName;
    }

    public int getMotorPort() {
        return motorPort;
    }

    public EncoderConfiguration getEncoderConfiguration() {
        return encoderConfiguration;
    }

    /**
     * Gets name of CANivore to use for this module.
     */
    public String getCanivoreName() {
        return canivoreName;
    }

    public boolean useCanivore() {
        // null or empty canivore name means don't use canivore
        return !(canivoreName == null || canivoreName.isEmpty());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o)
            return true;
        if (o == null || getClass() != o.getClass())
            return false;
        Falcon500SteerConfiguration<?> that = (Falcon500SteerConfiguration<?>) o;
        return getMotorPort() == that.getMotorPort()
                && getEncoderConfiguration().equals(that.getEncoderConfiguration());
    }

    @Override
    public int hashCode() {
        return Objects.hash(getMotorPort(), getEncoderConfiguration());
    }

    @Override
    public String toString() {
        return "Falcon500SteerConfiguration{" +
                "motorPort=" + motorPort +
                ", encoderConfiguration=" + encoderConfiguration +
                '}';
    }
}

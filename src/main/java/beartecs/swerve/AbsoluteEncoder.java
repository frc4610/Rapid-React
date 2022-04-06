package beartecs.swerve;

public interface AbsoluteEncoder {
    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [0, 2pi)
     */
    double getAbsoluteAngle();

    // raw position * coefficient
    // raw velocity / 600 * coefficient
    void setAbsoluteEncoder(double rawPosition, double rawVelocity);
}

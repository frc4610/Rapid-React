package beartecs.swerve;

public interface DriveController {
    Object getDriveMotor();

    void setVelocity(double velocity);

    void setReferenceVoltage(double voltage);

    double getVelocity();

    double getOutputVoltage();

    void resetEncoder();

    void setDriveEncoder(double position, double velocity);

    void configRampRate(double rampRate);
}

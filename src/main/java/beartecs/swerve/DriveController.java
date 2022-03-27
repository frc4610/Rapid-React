package beartecs.swerve;

public interface DriveController {
    Object getDriveMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getOutputVoltage();

    void resetEncoder();

    void setDriveEncoder(double position, double velocity);

    void configRampRate(double rampRate);
}

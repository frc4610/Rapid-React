package swervelib;

public interface DriveController {
    Object getDriveMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    void configRampRate(double rampRate);
}

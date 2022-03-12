package swervelib;

public interface SwerveModule {
    Object getDriveMotor();

    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getDriveVelocity();

    double getSteerAngle();

    void configRampRate(double rampRate);

    void set(double driveVoltage, double steerAngle);
}

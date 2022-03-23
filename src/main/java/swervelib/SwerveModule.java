package swervelib;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    Object getDriveMotor();

    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getDriveVelocity();

    double getSteerAngle();

    SwerveModuleState getState();

    void configRampRate(double rampRate);

    void set(double driveVoltage, double steerAngle);
}

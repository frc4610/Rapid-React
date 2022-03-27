package beartecs.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    Object getDriveMotor();

    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getDriveVelocity();

    double getSteerAngle();

    DriveController getDriveController();

    SteerController getSteerController();

    SwerveModuleState getState();

    void resetDriveEncoder();

    void configRampRate(double rampRate);

    void set(double driveVoltage, double steerAngle);
}

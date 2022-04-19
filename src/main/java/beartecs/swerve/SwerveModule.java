package beartecs.swerve;

import beartecs.configs.GearRatioConfig;
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

    GearRatioConfig getDriveGearRatioConfig();

    GearRatioConfig getSteerGearRatioConfig();

    void resetDriveEncoder();

    void configRampRate(double rampRate);

    void set(double driveVelocity, double steerAngle);

    void setVelocity(double driveVelocity, double steerAngle);
}

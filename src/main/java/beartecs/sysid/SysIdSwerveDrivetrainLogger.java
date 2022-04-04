package beartecs.sysid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.Constants.Motor;
import beartecs.math.Conversions;
import beartecs.swerve.SwerveModule;
import beartecs.swerve.config.Mk3ModuleConfiguration;

public class SysIdSwerveDrivetrainLogger extends SysIdLogger {
  public static double getDrivePositionMeters(WPI_TalonFX driveMotor) {
    return Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
        Mk3ModuleConfiguration.STANDARD.getWheelCircumference(), Motor.DRIVE_GEAR_RATIO);
  }

  public static double getDriveMetersPerSec(WPI_TalonFX driveMotor) {
    return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
        Mk3ModuleConfiguration.STANDARD.getWheelCircumference(), Motor.DRIVE_GEAR_RATIO);
  }

  public void log(SwerveModule[] modules,
      double measuredAngle, double angularRate) {
    updateData();
    m_data.add(m_timestamp);
    m_data.add(m_motorVoltage);
    for (SwerveModule mod : modules) {
      m_data.add(getDrivePositionMeters((WPI_TalonFX) mod.getDriveMotor()));
      m_data.add(getDriveMetersPerSec((WPI_TalonFX) mod.getDriveMotor()));
    }
    m_data.add(measuredAngle);
    m_data.add(angularRate);
  }

  void reset() {
    super.reset();
    m_motorVoltage = 0.0;
  }

  Boolean isWrongMechanism() {
    return !m_mechanism.equals("Drivetrain") && !m_mechanism.equals("Drivetrain (Angular)");
  }

}

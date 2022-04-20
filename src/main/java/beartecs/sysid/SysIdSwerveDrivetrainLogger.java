package beartecs.sysid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.configs.GearRatioConfig;
import beartecs.swerve.SwerveModule;

public class SysIdSwerveDrivetrainLogger extends SysIdLogger {
  public static double getDrivePositionMeters(WPI_TalonFX driveMotor, GearRatioConfig gearRatio) {
    return gearRatio.toMeters(driveMotor.getSelectedSensorPosition());
  }

  public static double getDriveMetersPerSec(WPI_TalonFX driveMotor, GearRatioConfig gearRatio) {
    return gearRatio.toVelocity(driveMotor.getSelectedSensorVelocity());
  }

  public void log(SwerveModule[] modules,
      double measuredAngle, double angularRate) {
    updateData();
    m_data.add(m_timestamp);
    m_data.add(m_motorVoltage);
    for (SwerveModule mod : modules) {
      m_data.add(getDrivePositionMeters((WPI_TalonFX) mod.getDriveMotor(), mod.getDriveGearRatioConfig()));
      m_data.add(getDriveMetersPerSec((WPI_TalonFX) mod.getDriveMotor(), mod.getDriveGearRatioConfig()));
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

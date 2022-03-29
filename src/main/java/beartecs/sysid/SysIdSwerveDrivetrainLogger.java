package beartecs.sysid;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import beartecs.Constants.Motor;
import beartecs.math.Conversions;
import beartecs.swerve.SwerveModule;
import beartecs.swerve.config.Mk3ModuleConfiguration;

public class SysIdSwerveDrivetrainLogger extends SysIdLogger {
  double m_moduleVoltages[];

  public static double getDrivePositionMeters(WPI_TalonFX driveMotor) {
    return Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
        Mk3ModuleConfiguration.STANDARD.getWheelCircumference(), Motor.DRIVE_GEAR_RATIO);
  }

  public static double getDriveMetersPerSec(WPI_TalonFX driveMotor) {
    return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
        Mk3ModuleConfiguration.STANDARD.getWheelCircumference(), Motor.DRIVE_GEAR_RATIO);
  }

  public double getVoltage(int index) {
    return m_moduleVoltages[index];
  }

  public void log(SwerveModule[] modules,
      double measuredAngle, double angularRate) {
    updateData();
    m_data.add(m_timestamp);
    for (int i = 0; i < m_moduleVoltages.length; i++) {
      m_data.add(m_moduleVoltages[i]);
      m_moduleVoltages[i] = m_motorVoltage;
    }
    for (SwerveModule mod : modules) {
      m_data.add(getDrivePositionMeters((WPI_TalonFX) mod.getDriveMotor()));
      m_data.add(getDriveMetersPerSec((WPI_TalonFX) mod.getDriveMotor()));
    }
    m_data.add(measuredAngle);
    m_data.add(angularRate);
  }

  void reset() {
    super.reset();
    for (int i = 0; i < m_moduleVoltages.length; i++) {
      m_moduleVoltages[i] = 0.0;
    }
  }

  Boolean isWrongMechanism() {
    return !m_mechanism.equals("Drivetrain") && !m_mechanism.equals("Drivetrain (Angular)");
  }

}

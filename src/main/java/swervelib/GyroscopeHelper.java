package swervelib;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import swervelib.kauailabs.NavXFactory;

public final class GyroscopeHelper {
  private GyroscopeHelper() {
  }

  public static Gyroscope createNavXMXP() {
    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    return new NavXFactory().build(navx);
  }

  public static Gyroscope createNavXUSB() {
    AHRS navx = new AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, (byte) 200);
    return new NavXFactory().build(navx);
  }
}
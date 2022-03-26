package swervelib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import swervelib.ctre.PigeonFactory;
import swervelib.kauailabs.NavXFactory;

public final class GyroscopeHelper {
  private GyroscopeHelper() {
  }

  public static Gyroscope createNavXMXP() {
    return new NavXFactory().build(new AHRS(SPI.Port.kMXP, (byte) 200));
  }

  public static Gyroscope createNavXUSB() {
    return new NavXFactory().build(new AHRS(SerialPort.Port.kUSB, SerialDataType.kProcessedData, (byte) 200));
  }

  public static Gyroscope createNavXI2C(I2C.Port port) {
    return new NavXFactory().build(new AHRS(port, (byte) 200));
  }

  public static Gyroscope createPigeonRibbonCabled(TalonSRX speedController) {
    return new PigeonFactory().build(new WPI_PigeonIMU(speedController));
  }

  public static Gyroscope createPigeonCAN(Integer canId) {
    return new PigeonFactory().build(new WPI_PigeonIMU(canId));
  }
}
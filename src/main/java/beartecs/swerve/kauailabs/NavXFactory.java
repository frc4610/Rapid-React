package beartecs.swerve.kauailabs;

import com.kauailabs.navx.frc.AHRS;

import beartecs.swerve.Gyroscope;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;

public class NavXFactory {
  public Gyroscope build(AHRS navX) {
    return new GyroscopeImplementation(navX);
  }

  private static class GyroscopeImplementation implements Gyroscope {
    private final AHRS navX;
    private final SimDouble angleSim;

    private GyroscopeImplementation(AHRS navX) {
      this.navX = navX;

      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      angleSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    }

    @Override
    public Rotation2d getGyroRotation() {
      //if (navX.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      //return Rotation2d.fromDegrees(navX.getFusedHeading());
      //}
      // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
      return Rotation2d.fromDegrees(360.0 - navX.getYaw());
    }

    @Override
    public double getGyroRate() {
      return navX.getRate();
    }

    @Override
    public void zeroGyro() {
      navX.zeroYaw();
    }

    @Override
    public void setAngle(double angle) {
      angleSim.set(angle);
    }
  }
}
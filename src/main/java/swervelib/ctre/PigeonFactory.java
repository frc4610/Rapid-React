package swervelib.ctre;

import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import swervelib.Gyroscope;

public class PigeonFactory {
  private static BasePigeonSimCollection pigeonSim;

  public Gyroscope build(WPI_PigeonIMU pigeon) {
    return new GyroscopeImplementation(pigeon);
  }

  private static class GyroscopeImplementation implements Gyroscope {
    private final WPI_PigeonIMU pigeon;

    private GyroscopeImplementation(WPI_PigeonIMU pigeon) {
      this.pigeon = pigeon;
      pigeonSim = pigeon.getSimCollection();
    }

    @Override
    public Rotation2d getGyroRotation() {
      return Rotation2d.fromDegrees(pigeon.getFusedHeading());
    }

    @Override
    public double getGyroRate() {
      return pigeon.getRate();
    }

    @Override
    public void zeroGyro() {
      pigeon.setFusedHeading(0.0);
    }

    @Override
    public void setAngle(double angle) {
      pigeonSim.setRawHeading(angle);
    }
  }
}

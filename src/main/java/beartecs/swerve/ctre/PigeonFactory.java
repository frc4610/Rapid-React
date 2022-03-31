package beartecs.swerve.ctre;

import com.ctre.phoenix.sensors.*;

import beartecs.math.MathUtils;
import beartecs.swerve.Gyroscope;
import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonFactory {

  public Gyroscope build(WPI_PigeonIMU pigeon) {
    return new GyroscopeImplementation(pigeon);
  }

  private static class GyroscopeImplementation implements Gyroscope {
    private final WPI_PigeonIMU pigeon;

    private GyroscopeImplementation(WPI_PigeonIMU pigeon) {
      this.pigeon = pigeon;
    }

    @Override
    public Rotation2d getGyroRotation() {
      return Rotation2d.fromDegrees(MathUtils.angleWrap(-pigeon.getFusedHeading()));
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
      pigeon.getSimCollection().setRawHeading(angle);
    }
  }
}

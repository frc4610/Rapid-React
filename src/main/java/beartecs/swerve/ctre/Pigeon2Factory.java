package beartecs.swerve.ctre;

import com.ctre.phoenix.sensors.*;

import beartecs.math.MathUtils;
import beartecs.swerve.Gyroscope;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Factory {
  public Gyroscope build(WPI_Pigeon2 pigeon) {
    return new GyroscopeImplementation(pigeon);
  }

  private static class GyroscopeImplementation implements Gyroscope {
    private final WPI_Pigeon2 pigeon;

    private GyroscopeImplementation(WPI_Pigeon2 pigeon) {
      this.pigeon = pigeon;
    }

    @Override
    public Rotation2d getGyroRotation() {
      return Rotation2d.fromDegrees(MathUtils.angleWrap(-pigeon.getYaw()));
    }

    @Override
    public double getGyroRate() {
      return pigeon.getRate();
    }

    @Override
    public void zeroGyro() {
      pigeon.setYaw(0);
    }

    @Override
    public void setAngle(double angle) {
      pigeon.getSimCollection().setRawHeading(angle);
    }
  }
}

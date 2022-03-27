package beartecs.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyroscope {

  Rotation2d getGyroRotation();

  void zeroGyro();

  double getGyroRate();

  void setAngle(double angle);
}
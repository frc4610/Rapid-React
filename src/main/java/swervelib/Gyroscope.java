package swervelib;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyroscope {

  Rotation2d getGyroRotation();

  void zeroGyroscope();

  void setAngle(double angle);
}
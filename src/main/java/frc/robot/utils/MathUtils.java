package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class MathUtils {
  public static final double EPSILON = 1e-9;

  // If we use it in diffrent places move to a Utils class
  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public static double modifyAxis(double value, double deadband_value) {
    // Deadband
    value = deadband(value, deadband_value);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static double normalize(double angleDegree) {
    double a;
    a = angleDegree / 360;
    a = a - (int) a;
    if (a > 0) {
      return a * 360;
    } else {
      return (1 + a) * 360;
    }
  }

  public static Rotation2d normalize(Rotation2d angle) {
    return Rotation2d.fromDegrees(normalize(angle.getDegrees()));
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, EPSILON);
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

}

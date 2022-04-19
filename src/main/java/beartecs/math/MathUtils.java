package beartecs.math;

import java.util.Random;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;

public final class MathUtils {
  static final double EPSILON = 1e-9;
  public static final Random random = new Random();

  // If we use it in diffrent places move to a Utils class
  public static double deadband(double value, double deadband) {
    return MathUtil.applyDeadband(value, deadband);
  }

  public static double modifyAxis(double value, double deadband_value) {
    // Deadband
    value = deadband(value, deadband_value);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public static boolean closeTo(double a, double b, double epsilon) {
    return epsilonEquals(a, b, epsilon);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, EPSILON);
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

  /*
   * return clamp
   */
  public static <T extends Comparable<T>> T clamp(T val, T min, T max) {
    if (val.compareTo(min) < 0)
      return min;
    else if (val.compareTo(max) > 0)
      return max;
    else
      return val;
  }

  public static <T extends Comparable<T>> Boolean withinRange(T val, T min, T max) {
    if (val.compareTo(min) < 0 || val.compareTo(max) > 0)
      return false;
    return true;
  }

  /*
  * Checks if a value is within a certain tolerance of a target. Directions irrelevant.
  */
  public static boolean withinTolerance(double value, double target, double tolerance) {
    return Math.abs(value - target) < tolerance;
  }

  /*
   * returns degree normalized withing 0-360
   */
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

  /*
   * returns Closest angle within 0-360
   */
  public static double normalizeInPlace(double curAngle, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = curAngle % 360;
    if (lowerOffset >= 0) {
      lowerBound = curAngle - lowerOffset;
      upperBound = curAngle + (360 - lowerOffset);
    } else {
      upperBound = curAngle - lowerOffset;
      lowerBound = curAngle - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - curAngle > 180) {
      newAngle -= 360;
    } else if (newAngle - curAngle < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public static double standardDeviation(double[] arr) {
    double mean = 0.0;
    double[] temp = new double[arr.length];

    mean = mean(arr);

    for (int i = 0; i < temp.length; i++) {
      temp[i] = Math.pow((arr[i] - mean), 2);
    }

    return Math.sqrt(mean(temp));
  }

  public static double mean(double[] arr) {
    double sum = 0.0;

    for (int i = 0; i < arr.length; i++) {
      sum += arr[i];
    }

    return sum / arr.length;
  }

  public static double max(double[] arr) {
    double max = 0;
    for (int i = 0; i < arr.length; i++) {
      if (arr[i] > max) {
        max = arr[i];
      }
    }

    return max;
  }

  public static int lerp(int a, int b, double v) {
    return (int) lerp(a, b, v);
  }

  public static double lerp(double a, double b, double v) {
    return MathUtil.interpolate(a, b, v);
  }

  /*
    0-360
    returns degrees relative to -180-180
  */
  public static double angleWrap(double degrees) {
    return MathUtil.inputModulus(degrees, -180.0, 180.0);
  }

  /*
    returns delta between two degrees
  */
  public static double angleDelta(double src, double dest) {
    double delta = (dest - src) % 360.0;
    if (Math.abs(delta) > 180.0) {
      delta = delta - (Math.signum(delta) * 360.0);
    }
    return delta;
  }

  /*
   * returns Rotation normalized
   */
  public static Rotation2d normalize(Rotation2d angle) {
    return Rotation2d.fromDegrees(normalize(angle.getDegrees()));
  }

  /*
  * returns angle within -PI and PI range
  */
  public static Rotation2d angleWrap(Rotation2d angle) {
    return new Rotation2d(MathUtil.angleModulus(angle.getRadians()));
  }

  /**
  * Creates a pure translating transform
  * 
  * @param translation The translation to create the transform with
  * @return The resulting transform
  */
  public static Transform2d transformFromTranslation(
      Translation2d translation) {
    return new Transform2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure translating transform
   * 
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d transformFromTranslation(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
   * Creates a pure rotating transform
   * 
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  public static Transform2d transformFromRotation(Rotation2d rotation) {
    return new Transform2d(new Translation2d(), rotation);
  }

  /**
   * Creates a pure translated pose
   * 
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d poseFromTranslation(Translation2d translation) {
    return new Pose2d(translation, new Rotation2d());
  }

  /**
   * Creates a pure rotated pose
   * 
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  public static Pose2d poseFromRotation(Rotation2d rotation) {
    return new Pose2d(new Translation2d(), rotation);
  }

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   * 
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
   * chain
   * 
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  public static Pose2d transformToPose(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /**
   * Interpolates between two poses based on the scale factor t. For example, t=0 would result in
   * the first pose, t=1 would result in the last pose, and t=0.5 would result in a pose which is
   * exactly halfway between the two poses. Values of t less than zero return the first pose, and
   * values of t greater than 1 return the last pose.
   * 
   * @param lhs The left hand side, or first pose to use for interpolation
   * @param rhs The right hand side, or last pose to use for interpolation
   * @param t The scale factor, 0 <= t <= 1
   * @return The pose which represents the interpolation. For t <= 0, the "lhs" parameter is
   *         returned directly. For t >= 1, the "rhs" parameter is returned directly.
   */
  public static Pose2d interpolate(Pose2d lhs, Pose2d rhs, double t) {
    t = MathUtil.clamp(t, 0, 1);
    if (t <= 0) {
      return lhs;
    } else if (t >= 1) {
      return rhs;
    }
    Twist2d twist = lhs.log(rhs);
    Twist2d scaled = new Twist2d(twist.dx * t, twist.dy * t, twist.dtheta * t);
    return lhs.exp(scaled);
  }

  /**
   * Returns the direction that this translation makes with the origin as a Rotation2d
   * 
   * @param translation The translation
   * @return The direction of the translation
   */
  public static Rotation2d direction(Translation2d translation) {
    return new Rotation2d(translation.getX(), translation.getY());
  }

  public static double gUnitsToAccelerationMeters(double gUnit) {
    return gUnit * 9.80665;
  }
}

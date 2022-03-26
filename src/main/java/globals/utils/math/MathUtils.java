package globals.utils.math;

import java.util.Random;

import edu.wpi.first.math.geometry.Rotation2d;

public class MathUtils {
  static final double sq2p1 = 2.414213562373095048802e0;
  static final double sq2m1 = .414213562373095048802e0;
  static final double p4 = .161536412982230228262e2;
  static final double p3 = .26842548195503973794141e3;
  static final double p2 = .11530293515404850115428136e4;
  static final double p1 = .178040631643319697105464587e4;
  static final double p0 = .89678597403663861959987488e3;
  static final double q4 = .5895697050844462222791e2;
  static final double q3 = .536265374031215315104235e3;
  static final double q2 = .16667838148816337184521798e4;
  static final double q1 = .207933497444540981287275926e4;
  static final double q0 = .89678597403663861962481162e3;
  static final double PIO2 = 1.5707963267948966135E0;
  static final double nan = (0.0 / 0.0);
  static final double EPSILON = 1e-9;
  public static final Random random = new Random();

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

  public static boolean closeTo(double a, double b, double epsilon) {
    return epsilonEquals(a, b, epsilon);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, EPSILON);
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

  private static double mxatan(double arg) {
    double argsq, value;

    argsq = arg * arg;
    value = ((((p4 * argsq + p3) * argsq + p2) * argsq + p1) * argsq + p0);
    value = value / (((((argsq + q4) * argsq + q3) * argsq + q2) * argsq + q1) * argsq + q0);
    return value * arg;
  }

  private static double msatan(double arg) {
    if (arg < sq2m1) {
      return mxatan(arg);
    }
    if (arg > sq2p1) {
      return PIO2 - mxatan(1 / arg);
    }
    return PIO2 / 2 + mxatan((arg - 1) / (arg + 1));
  }

  // implementation of arc tan
  public static double atan(double arg) {
    if (arg > 0) {
      return msatan(arg);
    }
    return -msatan(-arg);
  }

  // implementation of arc tan2
  public static double atan2(double arg1, double arg2) {
    if (arg1 + arg2 == arg1) {
      if (arg1 >= 0) {
        return PIO2;
      }
      return -PIO2;
    }
    arg1 = atan(arg1 / arg2);
    if (arg2 < 0) {
      if (arg1 <= 0) {
        return arg1 + Math.PI;
      }
      return arg1 - Math.PI;
    }
    return arg1;

  }

  // implementation of arc sin
  public static double asin(double arg) {
    double temp;
    int sign;

    sign = 0;
    if (arg < 0) {
      arg = -arg;
      sign++;
    }
    if (arg > 1) {
      return nan;
    }
    temp = Math.sqrt(1 - arg * arg);
    if (arg > 0.7) {
      temp = PIO2 - atan(temp / arg);
    } else {
      temp = atan(arg / temp);
    }
    if (sign > 0) {
      temp = -temp;
    }
    return temp;
  }

  // implementation of acos
  public static double acos(double arg) {
    if (arg > 1 || arg < -1) {
      return nan;
    }
    return PIO2 - asin(arg);
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

  /*
   * returns degree normalized withing 0-369
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
   * returns Rotation normalized
   */
  public static Rotation2d normalize(Rotation2d angle) {
    return Rotation2d.fromDegrees(normalize(angle.getDegrees()));
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

  // Why java have no templates comparable is nasty
  public static int lerp(int a, int b, double v) {
    return (int) (a + (b - a) * v);
  }

  public static float lerp(float a, float b, float v) {
    return a + (b - a) * v;
  }

  public static double lerp(double a, double b, double v) {
    return a + (b - a) * v;
  }

  public static <T extends Comparable<T>> Boolean withinRange(T val, T min, T max) {
    if (val.compareTo(min) < 0 || val.compareTo(max) > 0)
      return false;
    return true;
  }

  public static double angleWrap(double degrees) {
    double start = Math.floorMod((int) degrees + 180, 360); //will work for positive angles

    //angle is (-360, 0), add 360 to make (0, 360)
    if (start < 0) {
      start += 360;
    }

    //bring it back to (-180, 180)
    return start - 180;
  }

  public static double angleDelta(double src, double dest) {
    double delta = (dest - src) % 360.0;
    if (Math.abs(delta) > 180) {
      delta = delta - (Math.signum(delta) * 360);
    }
    return delta;
  }
}

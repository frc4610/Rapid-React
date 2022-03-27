package beartecs.math;

public class Conversions {
  public final static double FALCON_ENCODER = 2048.0;

  public static double falconToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * FALCON_ENCODER));
  }

  public static double degreesToFalcon(double degrees, double gearRatio) {
    double ticks = degrees / (360.0 / (gearRatio * FALCON_ENCODER));
    return ticks;
  }

  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / FALCON_ENCODER);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (FALCON_ENCODER / 600.0);
    return sensorCounts;
  }

  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }
}
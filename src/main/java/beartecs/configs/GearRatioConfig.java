package beartecs.configs;

public class GearRatioConfig {
  public final double TICK_RESOLUTION;
  public final int MAX_RPM;
  public final double GEAR_RATIO;
  public final double WHEEL_CIRCUMFRENCE;

  public GearRatioConfig(double tickRes, int maxRpm, double gearRatio) {
    this(tickRes, maxRpm, gearRatio, 0);
  }

  public GearRatioConfig(double tickRes, int maxRpm, double gearRatio, double wheelCircumference) {
    TICK_RESOLUTION = tickRes;
    MAX_RPM = maxRpm;
    GEAR_RATIO = gearRatio;
    WHEEL_CIRCUMFRENCE = wheelCircumference;
  }

  public double toDegrees(double counts) {
    return counts * (360.0 / (GEAR_RATIO * TICK_RESOLUTION));
  }

  public double fromDegrees(double degrees) {
    double ticks = degrees / (360.0 / (GEAR_RATIO * TICK_RESOLUTION));
    return ticks;
  }

  public double toRPM(double velocityCounts) {
    double motorRPM = velocityCounts * (600.0 / TICK_RESOLUTION);
    double mechRPM = motorRPM / GEAR_RATIO;
    return mechRPM;
  }

  public double fromRPM(double RPM) {
    double motorRPM = RPM * GEAR_RATIO;
    double sensorCounts = motorRPM * (TICK_RESOLUTION / 600.0);
    return sensorCounts;
  }

  // REQUIRED: WHEEL_CIRCUMFRENCE
  public double toMPS(double velocitycounts) {
    double wheelRPM = toRPM(velocitycounts);
    double wheelMPS = (wheelRPM * WHEEL_CIRCUMFRENCE) / 60;
    return wheelMPS;
  }

  // REQUIRED: WHEEL_CIRCUMFRENCE
  public double fromVelocity(double velocity) {
    double wheelRPM = ((velocity * 60) / WHEEL_CIRCUMFRENCE);
    double wheelVelocity = fromRPM(wheelRPM);
    return wheelVelocity;
  }

  // REQUIRED: WHEEL_CIRCUMFRENCE
  public double toMeters(double falconTiks) {
    double wheelRevs = (falconTiks / TICK_RESOLUTION) / GEAR_RATIO;
    double meters = wheelRevs * WHEEL_CIRCUMFRENCE;
    return meters;
  }
}

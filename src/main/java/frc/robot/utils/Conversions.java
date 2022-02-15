package frc.robot.utils;

import frc.robot.Constants;

public class Conversions {
  public static class Base {
    public static double falconToDegrees(double counts, double gearRatio) {
      return counts * (360.0 / (gearRatio * 2048.0));
    }

    public static double degreesToFalcon(double degrees, double gearRatio) {
      double ticks = degrees / (360.0 / (gearRatio * 2048.0));
      return ticks;
    }

    public static double falconToRPM(double velocityCounts, double gearRatio) {
      double motorRPM = velocityCounts * (600.0 / 2048.0);
      double mechRPM = motorRPM / gearRatio;
      return mechRPM;
    }

    public static double RPMToFalcon(double RPM, double gearRatio) {
      double motorRPM = RPM * gearRatio;
      double sensorCounts = motorRPM * (2048.0 / 600.0);
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

  /**
   * @param counts Falcon Counts
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double counts) {
    return counts * (360.0 / (Constants.Wheels.GEAR_RATIO * 2048.0));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @return Falcon Counts
   */
  public static double degreesToFalcon(double degrees) {
    double ticks = degrees / (360.0 / (Constants.Wheels.GEAR_RATIO * 2048.0));
    return ticks;
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / Constants.Wheels.GEAR_RATIO;
    return mechRPM;
  }

  /**
   * @param RPM RPM of mechanism
   * @return RPM of Mechanism
   */
  public static double RPMToFalcon(double RPM) {
    double motorRPM = RPM * Constants.Wheels.GEAR_RATIO;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }

  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference  Circumference of Wheel
   * @return Falcon Velocity Counts
   */
  public static double falconToMPS(double velocitycounts, double circumference) {
    double wheelRPM = falconToRPM(velocitycounts);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity      Velocity MPS
   * @param circumference Circumference of Wheel
   * @return Falcon Velocity Counts
   */
  public static double MPSToFalcon(double velocity, double circumference) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM);
    return wheelVelocity;
  }
}

package frc.robot.utils;

public class MathController {
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
}

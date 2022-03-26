package globals.utils;

import edu.wpi.first.math.controller.PIDController;

public class PidConfig {
  public double P;
  public double I;
  public double D;

  public PidConfig(double p, double i, double d) {
    P = p;
    I = i;
    D = d;
  }

  public PIDController getPidController() {
    return new PIDController(P, I, D);
  }
}

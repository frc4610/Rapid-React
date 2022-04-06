package beartecs.configs;

import edu.wpi.first.math.controller.PIDController;

public class PidConfig {
  public double P;
  public double I;
  public double D;
  public double F;

  public PidConfig(double p, double i, double d) {
    this(p, i, d, 0);
  }

  public PidConfig(double p, double i, double d, double f) {
    P = p;
    I = i;
    D = d;
    F = f;
  }

  public PIDController getPidController() {
    return new PIDController(P, I, D);
  }
}

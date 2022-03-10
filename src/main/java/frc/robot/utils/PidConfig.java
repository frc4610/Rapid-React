package frc.robot.utils;

public class PidConfig {
  public double P;
  public double I;
  public double D;

  public PidConfig(double p, double i, double d) {
    P = p;
    I = i;
    D = d;
  }
}

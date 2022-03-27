package beartecs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ProfiledPidConfig extends PidConfig {
  public Constraints constraints;

  public ProfiledPidConfig(double p, double i, double d, Constraints constraint) {
    super(p, i, d);
    constraints = constraint;
  }

  public ProfiledPIDController getProfiledPidController() {
    return new ProfiledPIDController(P, I, D, constraints);
  }
}

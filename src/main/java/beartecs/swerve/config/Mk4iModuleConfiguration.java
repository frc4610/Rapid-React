package beartecs.swerve.config;

import java.util.Objects;

import beartecs.swerve.ModuleConfiguration;

/**
 * Additional Mk4 module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the Mk4 swerve module.
 * Each setting is initialized to a default that should be adequate for most use cases.
 */
public class Mk4iModuleConfiguration {
  public static final ModuleConfiguration L1 = new ModuleConfiguration(
      0.10033,
      (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
      true,
      (14.0 / 50.0) * (10.0 / 60.0),
      false);
  public static final ModuleConfiguration L2 = new ModuleConfiguration(
      0.10033,
      (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
      true,
      (14.0 / 50.0) * (10.0 / 60.0),
      false);
  public static final ModuleConfiguration L3 = new ModuleConfiguration(
      0.10033,
      (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
      true,
      (14.0 / 50.0) * (10.0 / 60.0),
      false);

  private double nominalVoltage = 12.0;
  private double driveCurrentLimit = 80.0;
  private double steerCurrentLimit = 20.0;
  private String canivoreName = "";
  private boolean CANCoderCanivoreOnly = false;

  public void setCanivoreName(String canivoreName, boolean CANCoderOnly) {
    this.canivoreName = canivoreName;
    this.CANCoderCanivoreOnly = CANCoderOnly;
  }

  public void setCanivoreName(String canivoreName) {
    this.canivoreName = canivoreName;
  }

  public String getCanivoreName(boolean isCanCoder) {
    return CANCoderCanivoreOnly ? "rio" : canivoreName;
  }

  public boolean useCanivore() {
    // null or empty canivore name means don't use canivore
    return !(canivoreName == null || canivoreName.isEmpty());
  }

  public double getNominalVoltage() {
    return nominalVoltage;
  }

  public void setNominalVoltage(double nominalVoltage) {
    this.nominalVoltage = nominalVoltage;
  }

  public double getDriveCurrentLimit() {
    return driveCurrentLimit;
  }

  public void setDriveCurrentLimit(double driveCurrentLimit) {
    this.driveCurrentLimit = driveCurrentLimit;
  }

  public double getSteerCurrentLimit() {
    return steerCurrentLimit;
  }

  public void setSteerCurrentLimit(double steerCurrentLimit) {
    this.steerCurrentLimit = steerCurrentLimit;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o)
      return true;
    if (o == null || getClass() != o.getClass())
      return false;
    Mk4ModuleConfiguration that = (Mk4ModuleConfiguration) o;
    return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0
        && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0
        && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
  }

  @Override
  public int hashCode() {
    return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
  }

  @Override
  public String toString() {
    return "Mk4ModuleConfiguration{" +
        "nominalVoltage=" + nominalVoltage +
        ", driveCurrentLimit=" + driveCurrentLimit +
        ", steerCurrentLimit=" + steerCurrentLimit +
        ", canivoreName='" + canivoreName +
        '}';
  }
}

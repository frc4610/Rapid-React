package beartecs.swerve;

public interface SteerController {
    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    double getOutputVoltage();

    void setSteerEncoder(double position, double velocity);
}

package beartecs.swerve;

import beartecs.Calibration;
import beartecs.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModuleFactory<DriveConfiguration, SteerConfiguration> {
    private final ModuleConfiguration moduleConfiguration;
    private final DriveControllerFactory<?, DriveConfiguration> driveControllerFactory;
    private final SteerControllerFactory<?, SteerConfiguration> steerControllerFactory;

    public SwerveModuleFactory(ModuleConfiguration moduleConfiguration,
            DriveControllerFactory<?, DriveConfiguration> driveControllerFactory,
            SteerControllerFactory<?, SteerConfiguration> steerControllerFactory) {
        this.moduleConfiguration = moduleConfiguration;
        this.driveControllerFactory = driveControllerFactory;
        this.steerControllerFactory = steerControllerFactory;
    }

    public SwerveModule create(DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration) {
        var driveController = driveControllerFactory.create(driveConfiguration, moduleConfiguration);
        var steerController = steerControllerFactory.create(steerConfiguration, moduleConfiguration);

        return new ModuleImplementation(driveController, steerController);
    }

    public SwerveModule create(ShuffleboardLayout container, DriveConfiguration driveConfiguration,
            SteerConfiguration steerConfiguration) {
        var driveController = driveControllerFactory.create(
                container,
                driveConfiguration,
                moduleConfiguration);
        var steerContainer = steerControllerFactory.create(
                container,
                steerConfiguration,
                moduleConfiguration);

        return new ModuleImplementation(driveController, steerContainer);
    }

    private static class ModuleImplementation implements SwerveModule {
        private final DriveController driveController;
        private final SteerController steerController;

        private ModuleImplementation(DriveController driveController, SteerController steerController) {
            this.driveController = driveController;
            this.steerController = steerController;

        }

        @Override
        public Object getDriveMotor() {
            return driveController.getDriveMotor();
        }

        @Override
        public Object getSteerMotor() {
            return steerController.getSteerMotor();
        }

        @Override
        public AbsoluteEncoder getSteerEncoder() {
            return steerController.getSteerEncoder();
        }

        @Override
        public double getDriveVelocity() {
            return driveController.getStateVelocity();
        }

        @Override
        public double getSteerAngle() {
            return steerController.getStateAngle();
        }

        @Override
        public DriveController getDriveController() {
            return driveController;
        }

        @Override
        public SteerController getSteerController() {
            return steerController;
        }

        @Override
        public SwerveModuleState getState() {
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
        }

        @Override
        public void resetDriveEncoder() {
            driveController.resetEncoder();
        }

        @Override
        public void configRampRate(double rampRate) {
            this.driveController.configRampRate(rampRate);
        }

        @Override
        public void set(double driveVoltage, double steerAngle) {
            if (Calibration.CALIBRATION_MODE.getBoolean(false)) {
                if (Calibration.ZERO_WHEELS.getBoolean(false)) {
                    driveController.setReferenceVoltage(0);
                    steerController.setReferenceAngle(0);
                }
                return;
            }

            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0)

            {
                steerAngle += 2.0 * Math.PI;
            }

            double difference = steerAngle - getSteerAngle();

            if (Constants.ALIGN_RANGE_ENABLE && Math.abs(difference) < Math.toRadians(0.5)) { // Within range of a deadband don't update
                difference = 0;
            }

            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getSteerAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0)

            {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                driveVoltage *= -1.0;
            }

            // Put the target angle back into the range [0, 2pi)
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            driveController.setReferenceVoltage(driveVoltage);
            steerController.setReferenceAngle(steerAngle);
        }

        @Override
        public void setVelocity(double driveVelocity, double steerAngle) {
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            double difference = steerAngle - getSteerAngle();
            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getSteerAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                driveVelocity *= -1.0;
            }

            // Put the target angle back into the range [0, 2pi)
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            driveController.setVelocity(driveVelocity);
            steerController.setReferenceAngle(steerAngle);
        }
    }
}

package beartecs.configs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDrivetrainConfig {
    public final double TRACKWIDTH_METERS;
    public final double WHEELBASE_METERS;
    public final double MASS;
    public final double INERTIA;
    public final SwerveDriveKinematics KINEMATICS;

    public SwerveDrivetrainConfig(double trackwidth, double wheelbase, double mass) {
        TRACKWIDTH_METERS = trackwidth;
        WHEELBASE_METERS = wheelbase;
        MASS = mass;
        INERTIA = 1.0 / 12.0 * MASS * Math.pow((TRACKWIDTH_METERS * 1.1), 2) * 2;
        KINEMATICS = new SwerveDriveKinematics(
                // Front left
                new Translation2d(TRACKWIDTH_METERS / 2,
                        WHEELBASE_METERS / 2),
                // Front right
                new Translation2d(TRACKWIDTH_METERS / 2,
                        -WHEELBASE_METERS / 2),
                // Back left
                new Translation2d(-TRACKWIDTH_METERS / 2,
                        WHEELBASE_METERS / 2),
                // Back right
                new Translation2d(-TRACKWIDTH_METERS / 2,
                        -WHEELBASE_METERS / 2));
    }
}

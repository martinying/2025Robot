package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class DriveConstants {
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.3556 - 0.065, 0.3556 - 0.068),
        new Translation2d(0.3556 - 0.066, -0.3556 + 0.066),
        new Translation2d(0.3556 - 0.645, -0.3556 + 0.644),
        new Translation2d(0.3556 - 0.644, -0.3556 + 0.063)
    );
    
}

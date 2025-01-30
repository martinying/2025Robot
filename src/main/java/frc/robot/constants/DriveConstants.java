package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public final class DriveConstants {
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.3556 - 0.065, 0.3556 - 0.068), //front left
        new Translation2d(0.3556 - 0.066, -0.3556 + 0.066), //front right
        new Translation2d(0.3556 - 0.645, -0.3556 + 0.644), //back left
        new Translation2d(0.3556 - 0.644, -0.3556 + 0.063) //back right
    );

    public static final int GEAR_RATIO = 5;
    public static final int IMU_CAN_ID = 3;

    public static final int FRONT_LEFT_MODULE_INDEX = 0;
    public static final int FRONT_RIGHT_MODULE_INDEX = 1;
    public static final int BACK_LEFT_MODULE_INDEX = 2;
    public static final int BACK_RIGHT_MODULE_INDEX = 3;
    
    public static final double MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE = 5.0;
    public static final double WHEEL_RADIUS_DEFAULT_VALUE = 0.072; //6in diameter wheel in meters

    public static final RobotConfig PATH_PLANNER_CONFIG = new RobotConfig(
        59.0, //MASS OF ROBOT IN KG
        5.5, //MOI OF ROBOT SHOULD BE FROM CAD
        new ModuleConfig(//THIS IS THE CONFIG OF JUST THE DRIVE MOTOR IN THE MODULE
            WHEEL_RADIUS_DEFAULT_VALUE,
            MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE,
            1.0, //COEFFICIENT OF FRICTION BETWEEN TEH DRIVE WHEEL AND CARPET
            DCMotor.getKrakenX60(1).withReduction(GEAR_RATIO),
            60.0, //CURRENT (AMP) LIMIT OF THE DRIVE MOTOR
            1
        ),
        new Translation2d(0.3556 - 0.065, 0.3556 - 0.068), //front left
        new Translation2d(0.3556 - 0.066, -0.3556 + 0.066), //front right
        new Translation2d(0.3556 - 0.645, -0.3556 + 0.644), //back left
        new Translation2d(0.3556 - 0.644, -0.3556 + 0.063) //back right
    );
}

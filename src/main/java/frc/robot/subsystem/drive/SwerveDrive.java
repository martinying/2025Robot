package frc.robot.subsystem.drive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.AutoLogOutput;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private OdometryThread odometryThread = new OdometryThread();

    public SwerveDrive() {
        initializeOdometryThread();
    }

    private void initializeOdometryThread() {
        ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
        scheduler.scheduleAtFixedRate(odometryThread, 4, 10, TimeUnit.MILLISECONDS);
    }

    /**
     * This is used for Teleop with the Joystick. The joystick is in x, y and theta.
     * The only way to translate that to information that can be used in a
     * SwerveDrive
     * is to use Kinematics and get SwerveModuleState. Kinematics does not translate
     * joystick x,y, theta to ChassisSpped.
     * 
     * Kinematicss allows SwerveModuleState to translate to ChassisSpeed but we
     * would
     * add another calcuation to get to ChassisSpeed introducing futher calculaton
     * inaccuracies.
     * 
     * @param desiredSwerveModuleStates
     */
    public void setModuleStates(SwerveModuleState[] desiredSwerveModuleStates) {
        odometryThread.setModuleStates(desiredSwerveModuleStates);
    }

    @Override
    public void periodic() {
        // all periodic is not taken care of in OdometryThread.run

        // the periodic use to update input (get measured readings) object
        // and then update odometry with the measured readings
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return odometryThread.getPoseMeters();
    }

    public Rotation2d getMeasuredAngle() {
        return odometryThread.getMeasuredAngle();
    }
}

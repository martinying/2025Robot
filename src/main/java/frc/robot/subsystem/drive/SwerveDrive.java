package frc.robot.subsystem.drive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
    private OdometryThread odometryThread = new OdometryThread();

    public SwerveDrive() {
        initializeOdometryThread();
        initPathPlanner();
    }

    private void initializeOdometryThread() {
        ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
        scheduler.scheduleAtFixedRate(odometryThread, 4, 10, TimeUnit.MILLISECONDS);
    }

    private void initPathPlanner(){
        AutoBuilder.configure(
            this::getPose, 
            this::setPose, 
            this::getChassisSpeeds, 
            (speeds, feedforwards) -> driveRobot(speeds, feedforwards), 
            new PPHolonomicDriveController(
                new PIDConstants(5.0,0.0,0,0), new PIDConstants(5.0,0.0,0.0)
            ),
            DriveConstants.PATH_PLANNER_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );
        PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
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

    public void setPose(Pose2d pose) {
        odometryThread.resetPosition(pose);
    }

    public Rotation2d getMeasuredAngle() {
        return odometryThread.getMeasuredAngle();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(odometryThread.getModuleStates());
    }

    public void driveRobot(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
        Logger.recordOutput("SwerveDrive/DesiredChassisSpeed", chassisSpeeds);

        SwerveModuleState [] desiredSwerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        Logger.recordOutput("SwerveDrive/DesiredSwerveModuleStates", desiredSwerveModuleStates);
        double [] moduleAnglesInDegrees = {desiredSwerveModuleStates[0].angle.getDegrees(), desiredSwerveModuleStates[1].angle.getDegrees(), desiredSwerveModuleStates[2].angle.getDegrees(), desiredSwerveModuleStates[3].angle.getDegrees()};
        Logger.recordOutput("SwerveDrive/DesiredModuleAnglesInDegrees", moduleAnglesInDegrees);
        Logger.recordOutput("SwerveDrive/ClockwiseBy90Degrees",Rotation2d.kCW_90deg.getDegrees());
        Logger.recordOutput("SwerveDrive/CounterClockwiseBy90Degrees",Rotation2d.kCCW_90deg.getDegrees());
    
        odometryThread.setModuleStates(desiredSwerveModuleStates);
    }
}

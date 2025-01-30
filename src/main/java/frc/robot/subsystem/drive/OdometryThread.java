package frc.robot.subsystem.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.DriveConstants;

public class OdometryThread implements Runnable{
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final Canandgyro m_imu = new Canandgyro(DriveConstants.IMU_CAN_ID);

    private SwerveModule[] swerveModules = new SwerveModule[4];
    private SwerveModulePosition[] lastSwerveModulePositions = new SwerveModulePosition[4];
    private SwerveModulePosition[] moduleDeltas = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    SwerveDriveOdometry odometry;

    public OdometryThread() {
        swerveModules[DriveConstants.FRONT_LEFT_MODULE_INDEX] = new SwerveModule(0,1,2);
        swerveModules[DriveConstants.FRONT_RIGHT_MODULE_INDEX] = new SwerveModule(3,4,5);
        swerveModules[DriveConstants.BACK_LEFT_MODULE_INDEX] = new SwerveModule(6,7,8);
        swerveModules[DriveConstants.BACK_RIGHT_MODULE_INDEX] = new SwerveModule(9,10,11);

        updateInputs();//initialize inputs
        lastSwerveModulePositions = getModulePositions();//prepare for delta calculations
        odometry  = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getMeasuredAngle(), lastSwerveModulePositions);
    }

    @Override
    public void run() {
        long start = System.currentTimeMillis();

        updateInputs();//get motor measured value for new module positions
        //update odometry with new module positions
        odometry.update(getMeasuredAngle(), getModulePositions());

        if(System.currentTimeMillis()-start > 10) {
            System.out.println("OVERRUN");
        }
    }    

    //prevent reads while new module state are being updated
    public synchronized void setModuleStates(SwerveModuleState [] desiredSwerveModuleStates) {
        for(int counter = 0; counter < 4; counter++) {
            swerveModules[counter].setModuleState(desiredSwerveModuleStates[counter]);
            if(RobotBase.isSimulation()){//prepare info to simulate gyro
                calculateSwerveModuleDeltas(counter);                
            }
        }
    }

    public synchronized SwerveModuleState [] getModuleStates() {
        updateInputs();
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int counter = 0; counter < 4; counter++) {
            states[counter] = swerveModules[counter].getState();
        }    
        return states;
    }

    public void resetPosition(Pose2d pose) {
        odometry.resetPosition(getMeasuredAngle(),getModulePositions(),pose);
    }
    
    @AutoLogOutput(key = "Gyro/Angle")
    public Rotation2d getMeasuredAngle() {
        if(RobotBase.isReal()) {
            rawGyroRotation = m_imu.getRotation2d();
        } else if(RobotBase.isSimulation()) {
            //https://www.chiefdelphi.com/t/any-gyro-sims-recommendations/444425/2
            synchronized(rawGyroRotation){//in simulation only one call at a time to get value while it's being simulated
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas).dtheta));
            }
        }//else it's replay
        return rawGyroRotation;
    }

    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }

    private void calculateSwerveModuleDeltas(int index) {
        swerveModules[index].updateInputs();//need updated inputs for delta calculations
        SwerveModulePosition currentPosition = swerveModules[index].getPosition();

        moduleDeltas[index] = new SwerveModulePosition(
            currentPosition.distanceMeters
            - lastSwerveModulePositions[index].distanceMeters,
            currentPosition.angle
        );
        lastSwerveModulePositions[index] = currentPosition;
    }

    //prevent reads while the inputs are being updated
    private synchronized void updateInputs(){
        for(int counter = 0; counter < 4; counter++) {
            swerveModules[counter].updateInputs();
            Logger.processInputs("SwerveDrive/Module/"+counter,swerveModules[counter].getInputs());
        }
    }

    private synchronized SwerveModulePosition[] getModulePositions() {
        updateInputs();
        SwerveModulePosition[] positons = new SwerveModulePosition[4];
        for (int counter = 0; counter < 4; counter++) {
            positons[counter] = swerveModules[counter].getPosition();
        }    
        return positons;
    }
}

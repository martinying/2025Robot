package frc.robot.subsystem;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
    private final Canandgyro m_imu = new Canandgyro(DriveConstants.IMU_CAN_ID);
    private SwerveModule[] swerveModules = new SwerveModule[4];
    private SwerveModulePosition[] lastSwerveModulePositions = new SwerveModulePosition[4];
    private SwerveModulePosition[] moduleDeltas = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    private Rotation2d rawGyroRotation = new Rotation2d();
    SwerveDriveOdometry odometry;

    public SwerveDrive() {
        swerveModules[DriveConstants.FRONT_LEFT_MODULE_INDEX] = new SwerveModule(0,1,2);
        swerveModules[DriveConstants.FRONT_RIGHT_MODULE_INDEX] = new SwerveModule(3,4,5);
        swerveModules[DriveConstants.BACK_LEFT_MODULE_INDEX] = new SwerveModule(6,7,8);
        swerveModules[DriveConstants.BACK_RIGHT_MODULE_INDEX] = new SwerveModule(9,10,11);

        //initialize inputs
        for(int counter = 0; counter < 4; counter++) {
            swerveModules[counter].updateInputs();
        }
        lastSwerveModulePositions = getModulePositions();

        odometry  = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getMeasuredAngle(), getModulePositions());
    }

    @AutoLogOutput(key = "Gyro/Angle")
    public Rotation2d getMeasuredAngle() {
        if(RobotBase.isReal()) {
            rawGyroRotation = m_imu.getRotation2d();
        } else if(RobotBase.isSimulation()) {
            //https://www.chiefdelphi.com/t/any-gyro-sims-recommendations/444425/2
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas).dtheta));
        }//else it's replay
        return rawGyroRotation;
    }

    /**
     * This is used for Teleop with the Joystick.  The joystick is in  x, y and theta.
     * The only way to translate that to information that can be used in a SwerveDrive
     * is to use Kinematics and get SwerveModuleState.  Kinematics does not translate
     * joystick x,y, theta to ChassisSpped.
     * 
     * Kinematicss allows SwerveModuleState to translate to ChassisSpeed but we would
     * add another calcuation to get to ChassisSpeed introducing futher calculaton
     * inaccuracies.
     * @param desiredSwerveModuleStates
     */
    public void setModuleStates(SwerveModuleState [] desiredSwerveModuleStates) {
        for(int counter = 0; counter < 4; counter++) {
            swerveModules[counter].setModuleState(desiredSwerveModuleStates[counter]);
            if(RobotBase.isSimulation()){//prepare info to simulate gyro
                swerveModules[counter].updateInputs();//need updated inputs for delta calculations
                SwerveModulePosition currentPosition = swerveModules[counter].getPosition();

                moduleDeltas[counter] = new SwerveModulePosition(
                    currentPosition.distanceMeters
                    - lastSwerveModulePositions[counter].distanceMeters,
                    currentPosition.angle
                );
                lastSwerveModulePositions[counter] = currentPosition;
            }
        }
    }
    
    @Override
    public void periodic() {
        for(int counter = 0; counter < 4; counter++) {
            swerveModules[counter].updateInputs();
            Logger.processInputs("SwerveDrive/Module/"+counter,swerveModules[counter].getInputs());
        }

        //pulled out so we can record the module postions
        SwerveModulePosition[] currentSwerveModulePositions = getModulePositions();

        odometry.update(getMeasuredAngle(), currentSwerveModulePositions);

        //may need to handle simulations by doing calculations for gyro
        //lastSwerveModulePositions = currentSwerveModulePositions;
        
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(getMeasuredAngle(), getModulePositions(), pose);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int counter = 0; counter < 4; counter++) {
          states[counter] = swerveModules[counter].getPosition();
        }
        return states;
      }
}

package frc.robot.subsystem;

import org.littletonrobotics.junction.Logger;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;

public class SwerveDrive extends SubsystemBase {
    private final Canandgyro m_imu = new Canandgyro(DriveConstants.kIMUCanID);
    private SwerveModuleIOInputsAutoLogged[] inputs = {new SwerveModuleIOInputsAutoLogged(),new SwerveModuleIOInputsAutoLogged(),new SwerveModuleIOInputsAutoLogged(),new SwerveModuleIOInputsAutoLogged()};
    private SwerveModule[] swerveModules = new SwerveModule[4];
    private SwerveModulePosition[] lastSwerveModulePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    private SwerveModulePosition[] moduleDeltas = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    private Rotation2d rawGyroRotation = new Rotation2d();

    public SwerveDrive() {
        swerveModules[DriveConstants.FRONT_LEFT_MODULE_INDEX] = new SwerveModule(0,1,2);
        swerveModules[DriveConstants.FRONT_RIGHT_MODULE_INDEX] = new SwerveModule(3,4,5);
        swerveModules[DriveConstants.BACK_LEFT_MODULE_INDEX] = new SwerveModule(6,7,8);
        swerveModules[DriveConstants.BACK_RIGHT_MODULE_INDEX] = new SwerveModule(9,10,11);
    }

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
                swerveModules[counter].updateInputs(inputs[counter]);//need updated inputs for delta calculations
                SwerveModulePosition currentPosition = swerveModules[counter].getPosition(inputs[counter]);
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
            swerveModules[counter].updateInputs(inputs[counter]);
            Logger.processInputs("SwerveDrive/Module/"+counter, inputs[counter]);
        }
    }
}

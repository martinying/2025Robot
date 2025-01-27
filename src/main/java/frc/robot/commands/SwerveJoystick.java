package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.PreferenceKeys;
import frc.robot.subsystem.SwerveDrive;

public class SwerveJoystick extends Command {
    private final Joystick joyStick;
    private final SwerveDrive swerveDrive;
    
    public SwerveJoystick(SwerveDrive swerveDrive, Joystick joyStick) {
        this.joyStick = joyStick;
        this.swerveDrive = swerveDrive;
        // User addRequirements() to declare subsystem dependencies
        addRequirements(this.swerveDrive);
    }
    @Override
    public void execute() {
        double xSpeed = joyStick.getRawAxis(IOConstants.kJoystickXAxis);
        double ySpeed = joyStick.getRawAxis(IOConstants.kJoystickYxis);
        double turningSpeed = joyStick.getRawAxis((IOConstants.kJoystickRotAxis));

        //LOG RAW VALUES
        SmartDashboard.putNumber("Joystick/xSpeedRaw", xSpeed);
        SmartDashboard.putNumber("Joystick/ySpeedRaw", ySpeed);
        SmartDashboard.putNumber("Joystick/turningSpeedRaw", turningSpeed);

        //Makes the speed response x squared in relation to the joystick input.
        //That way, the first little bit of joystick input gives more control.
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);

        //LOG SQUARED VALUES
        SmartDashboard.putNumber("Joystick/xSpeedSquared", xSpeed);
        SmartDashboard.putNumber("Joystick/ySpeedSquared", ySpeed);

        //apply deadband
        xSpeed = Math.abs(xSpeed) > IOConstants.JOYSTICK_DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > IOConstants.JOYSTICK_DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > IOConstants.JOYSTICK_DEADBAND ? turningSpeed : 0.0;

        //scale to meters per second
        xSpeed = xSpeed*Preferences.getDouble(PreferenceKeys.MAX_SPEED_METER_PER_SECCONDS, DriveConstants.MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE);
        ySpeed = ySpeed*Preferences.getDouble(PreferenceKeys.MAX_SPEED_METER_PER_SECCONDS, DriveConstants.MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE);
        turningSpeed = turningSpeed*Preferences.getDouble(PreferenceKeys.MAX_SPEED_METER_PER_SECCONDS, DriveConstants.MAX_SPEED_METER_PER_SECCONDS_DEFAULT_VALUE);

        //LOG FINAL VALUES
        SmartDashboard.putNumber("Joystick/xSpeedFinal", xSpeed);
        SmartDashboard.putNumber("Joystick/ySpeedFinal", ySpeed);
        SmartDashboard.putNumber("Joystick/turningSpeedFinal", turningSpeed);

        //IN TELEOP WE WANT FIELD RELATIVE
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveDrive.getMeasuredAngle());
        Logger.recordOutput(getName()+"/ChassisSpeed", chassisSpeed);

        SwerveModuleState [] desiredSwerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeed);
        //Using recordOutputs allows for AdvantageScope integraiton during simulation to see in the UI each ServeModule vector and angle
        Logger.recordOutput(getName()+"/DesiredSwerveModuleStates", desiredSwerveModuleStates);
        double [] moduleAnglesInDegrees = {desiredSwerveModuleStates[0].angle.getDegrees(), desiredSwerveModuleStates[1].angle.getDegrees(), desiredSwerveModuleStates[2].angle.getDegrees(), desiredSwerveModuleStates[3].angle.getDegrees()};
        Logger.recordOutput(getName()+"/ModuleAnglesInDegrees", moduleAnglesInDegrees);
        Logger.recordOutput(getName()+"/CounterClockwiseBy90Degrees",Rotation2d.kCW_90deg.getDegrees());
        swerveDrive.setModuleStates(desiredSwerveModuleStates);
    }
}

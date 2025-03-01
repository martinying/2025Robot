package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.PreferenceKeys;
import frc.robot.subsystem.drive.SwerveDrive;

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
        swerveDrive.driveRobot(chassisSpeed, null);
    }
}

package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    public Rotation2d getMeasuredAngle() {
      // TODO Auto-generated method stub
      return new Rotation2d(0.0);
    }

    public void setModuleStates(SwerveModuleState [] desiredModuleStates) {
      // TODO Auto-generated method stub
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveJoystick;
import frc.robot.constants.IOConstants;
import frc.robot.subsystem.SwerveDrive;

public class RobotContainer {
  private final Joystick joyStick = new Joystick(IOConstants.kDriveJoystickId);
  private SwerveDrive swerveDrive = new SwerveDrive();

  public RobotContainer() {
    bindCommandsToSubsystems();
    configureBindings();
  }

  private void bindCommandsToSubsystems() {
    swerveDrive.setDefaultCommand(new SwerveJoystick(swerveDrive,joyStick));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

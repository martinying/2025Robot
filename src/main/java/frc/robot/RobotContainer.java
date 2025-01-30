// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveJoystick;
import frc.robot.constants.IOConstants;
import frc.robot.subsystem.drive.SwerveDrive;

public class RobotContainer {
  private final Joystick joyStick = new Joystick(IOConstants.kDriveJoystickId);
  private SwerveDrive swerveDrive = new SwerveDrive();
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    bindCommandsToSubsystems();
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

  private void bindCommandsToSubsystems() {
    swerveDrive.setDefaultCommand(new SwerveJoystick(swerveDrive,joyStick));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

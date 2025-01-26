// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Test;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public RobotContainer() {



    //driverController.a().whileTrue(new Test());
  }

  
  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(
        new RunCommand(
          () -> swerveSubsystem.setChassisSpeeds(getChassisSpeedsFromController()), swerveSubsystem
        )
    );
    driverController.getLeftX();
  }

 
  public Command getAutonomousCommand() {
    return null;
  }
  private ChassisSpeeds getChassisSpeedsFromController() {
    // Read joystick axes
    double xSpeed = -driverController.getLeftY(); // Forward/backward
    double ySpeed = -driverController.getLeftX(); // Strafe left/right
    double rot = -driverController.getRightX();   // Rotation (CW/CCW)

    // Apply deadband (optional, prevents drift from small joystick values)
    xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0.0;
    rot = Math.abs(rot) > 0.1 ? rot : 0.0;

    return new ChassisSpeeds(xSpeed, ySpeed, rot);


    // // Scale joystick values for max speeds
    // xSpeed *= SwerveSubsystem.MAX_SPEED_METERS_PER_SECOND;
    // ySpeed *= SwerveSubsystem.MAX_SPEED_METERS_PER_SECOND;
    // rot *= SwerveSubsystem.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
  
  }
}

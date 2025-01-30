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
  public static CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public RobotContainer() {

    if(driverController.a().getAsBoolean()){
      System.out.println("A is pressed");
    }
    //swerve module
    swerveSubsystem.setDefaultCommand(

    new RunCommand(
      () -> swerveSubsystem.drive(
          -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
          true),
          swerveSubsystem));



    configureBindings();
    //driverController.a().whileTrue(new Test());
  }

  
  private void configureBindings() {

    //driverController.getLeftX();
  }

 
  public Command getAutonomousCommand() {
    return null;
  }
}

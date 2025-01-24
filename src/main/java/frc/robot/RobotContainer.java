// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final SwerveSubsystem m_robotDrive = new SwerveSubsystem();
  // private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  // private final CommandXboxController m_driverControllerCommand = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(m_driverController);
  

  
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  //   m_robotDrive.setDefaultCommand(
  //   // The left stick controls translation of the robot.
  //   // Turning is controlled by the X axis of the right stick.

  //   //square button for aim assist
  //   //uncomment if no worky
  //   new RunCommand(
  //       () -> m_robotDrive.DriverDrive(
  //           -MathUtil.applyDeadband(m_driverControllerCommand.getLeftY(), OIConstants.kDriveDeadband),
  //           -MathUtil.applyDeadband(m_driverControllerCommand.getLeftX(), OIConstants.kDriveDeadband),
  //           -MathUtil.applyDeadband(m_driverControllerCommand.getRightX(), OIConstants.kDriveDeadband),
  //           true, true, m_driverController),
  //       m_robotDrive));
  m_swerveSubsystem.setDefaultCommand(
    new RunCommand(
        () -> m_swerveSubsystem.drive(
            -m_driverController.getLeftY(),
            m_driverController.getLeftX(),
            m_driverController.getRightX()
            // true,  // fieldRelative (change as needed)
            // false  // rateLimit (change as needed)
        ),
        m_swerveSubsystem
    ));
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    //return nothing
    return null;
  }
}

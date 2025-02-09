// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Test;
import frc.robot.commands.ElevatorCommand.ElevatorCommand;
import frc.robot.commands.ElevatorCommand.ElevatorManualCommand;
import frc.robot.commands.ElevatorCommand.ElevatorSetPositionCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
//  public static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static CommandXboxController m_driverController = new CommandXboxController(1);

  public static CommandXboxController m_auxController = new CommandXboxController(OIConstants.kAuxControllerPort);
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final Trigger auxY = m_auxController.y();
  private final Trigger auxA = m_auxController.a();
  private final Trigger auxB = m_auxController.b();
  private final Trigger auxX = m_auxController.x();
  private final Trigger auxRightBumper = m_auxController.rightBumper();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> swerveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            swerveSubsystem));

    // elevatorSubsystem.setDefaultCommand(
    // new ElevatorManualCommand(
    // () -> -elevatorSubsystem.manualControl(
    // -MathUtil.applyDeadband(m_driverController.getLeftY(),
    // OIConstants.kDriveDeadband)
    // ),elevatorSubsystems
    // )); // Invert Y-axis

    configureBindings();
  }

  private void configureBindings() {
    auxA.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_ROTATIONS)
    .alongWith(Commands.print("Elevator Level 1, Height: " + Constants.ElevatorConstants.STAGE_1_HEIGHT_ROTATIONS)));
    auxB.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_ROTATIONS)
    .alongWith(Commands.print("Elevator Level 2, Height: " + Constants.ElevatorConstants.STAGE_2_HEIGHT_ROTATIONS)));
    auxX.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_ROTATIONS)
    .alongWith(Commands.print("Elevator Level 3, Height: " + Constants.ElevatorConstants.STAGE_3_HEIGHT_ROTATIONS)));
    auxY.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_ROTATIONS)
    .alongWith(Commands.print("Elevator Level 4, Height: " + Constants.ElevatorConstants.STAGE_4_HEIGHT_ROTATIONS)));
    auxRightBumper.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.SOURCE_HEIGHT_ROTATIONS)
    .alongWith(Commands.print("Elevator Source, Height: " + Constants.ElevatorConstants.SOURCE_HEIGHT_ROTATIONS)));

    
    // new JoystickButton(m_auxController, XboxController.Button.kA.value)
    //     .onTrue(
    //         new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_ROTATIONS));

    // new JoystickButton(m_auxController, XboxController.Button.kB.value)
    //     .onTrue(
    //         new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_ROTATIONS));

    // new JoystickButton(m_auxController, XboxController.Button.kX.value)
    //     .onTrue(
    //         new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_ROTATIONS));

    // new JoystickButton(m_auxController, XboxController.Button.kY.value)
    //     .onTrue(
    //         new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_ROTATIONS));

    // new JoystickButton(m_auxController, XboxController.Button.kStart.value)
    //     .onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.SOURCE_HEIGHT_ROTATIONS));

    // // driverController.getLeftX();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

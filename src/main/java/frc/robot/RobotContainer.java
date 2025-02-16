package frc.robot;

import frc.robot.Constants.ArmConstant;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand.ArmSetPositionCommand;
import frc.robot.commands.ElevatorCommand.ElevatorSetPositionCommand;
import frc.robot.commands.IntakeCommand.IntakeWithDetectionCommand;
import frc.robot.subsystems.newArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveSubsystem.TunerConstants;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.hardware.CANrange;
import frc.robot.Constants.IntakeConstant;;

import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class RobotContainer {
    // swerve drive stuff
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
                                                                                    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // end of swerve drive things

    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    public static CommandXboxController m_auxController = new CommandXboxController(OIConstants.kAuxControllerPort);

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final newArmSubsystem arm = new newArmSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    private final Trigger auxY = m_auxController.y();
    private final Trigger auxA = m_auxController.a();
    private final Trigger auxB = m_auxController.b();
    private final Trigger auxX = m_auxController.x();
    private final Trigger auxRightBumper = m_auxController.rightBumper();
    private final Trigger auxRightTrigger = m_auxController.rightTrigger();
    private final Trigger auxLeftBumper = m_auxController.leftBumper();
    private final Trigger auxLeftTrigger = m_auxController.leftTrigger();
    private final Trigger auxPovUP = m_auxController.povUp();
    private final Trigger auxPovDOWN = m_auxController.povDown();
    private final Trigger auxPovLEFT = m_auxController.povLeft();
    private final Trigger auxPovRIGHT = m_auxController.povRight();

    private final Trigger driveY = m_driverController.y();
    private final Trigger driveA = m_driverController.a();
    private final Trigger driveB = m_driverController.b();
    private final Trigger driveX = m_driverController.x();
    private final Trigger driveRightBumper = m_driverController.rightBumper();
    private final Trigger driveRightTrigger = m_driverController.rightTrigger();
    private final Trigger driveLeftBumper = m_driverController.leftBumper();
    private final Trigger driveLeftTrigger = m_driverController.leftTrigger();
    private final Trigger drivePovDOWN = m_driverController.povDown();

    public RobotContainer() {

        configureBindings();
    }

    private void configureBindings() {
        // begin of swerve drive bindings
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain
                        .applyRequest(() -> drive
                                .withVelocityX(
                                        -mathProfiles.exponentialDrive(m_driverController.getLeftY(), 3) * MaxSpeed)
                                .withVelocityY(
                                        -mathProfiles.exponentialDrive(m_driverController.getLeftX(), 3) * MaxSpeed)
                                .withRotationalRate(-mathProfiles.exponentialDrive(m_driverController.getRightX(), 2)
                                        * MaxAngularRate))
        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
        // negative Y (forward)
        // .withVelocityY(-joystick.getLeftX()* MaxSpeed) // Drive left with negative X
        // (left)
        // .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
        // counterclockwise with negative X (left)
        // )
        );

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(
                        new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on POV down press
        drivePovDOWN.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        // end of swerve drive bindings

        //Intake binds
        //auxPovRight.onTrue();
        //auxPovLeft.onTrue();

        // Elevator and Arm bindings

        //SOURCE
        auxRightBumper.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA)
                .alongWith(Commands.print("Elevator Source, Height: " + Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA.in(Units.Meters))));
        auxRightBumper.onTrue(new ArmSetPositionCommand(arm, ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Source, Angles: " + ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))));

        //GO DOWN
        auxRightTrigger.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA)
                .alongWith(Commands.print("Elevator Down, Height: " + Constants.ElevatorConstants.ELEVATOR_BASE_DELTA.in(Units.Meters))));
        auxRightTrigger.onTrue(new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Base, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))));

        //L1
        auxA.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA)
                .alongWith(Commands.print("Elevator Level 1, Height: " + Constants.ElevatorConstants.STAGE_1_HEIGHT.in(Units.Meters))));
        auxA.onTrue(new ArmSetPositionCommand(arm, ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Level 1, Angles: " + ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))));

        //L1
        auxB.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA)
                .alongWith(Commands.print("Elevator Level 2, Height: " + Constants.ElevatorConstants.STAGE_2_HEIGHT.in(Units.Meters))));
        auxB.onTrue(new ArmSetPositionCommand(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Level 2, Angles: " + ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))));

        //L3
        auxX.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA)
                .alongWith(Commands.print("Elevator Level 3, Height: " + Constants.ElevatorConstants.STAGE_3_HEIGHT.in(Units.Meters))));
        auxX.onTrue(new ArmSetPositionCommand(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Level 3, Angles: " + ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))));

        //L4
        auxY.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                .alongWith(Commands.print("Elevator Level 4, Height: " + Constants.ElevatorConstants.STAGE_4_HEIGHT.in(Units.Meters))));
        auxY.onTrue(new ArmSetPositionCommand(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                 .alongWith(Commands.print("Arm Level 4, Angles: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))));

        // auxPovUP.whileTrue(new RunCommand(
        //         () -> {
        //                 elevatorSubsystem.manualControl(0.5);
        //                 arm.manualControl(0.5);
        //         }, elevatorSubsystem, arm).alongWith(Commands.print("value for controller: "+m_auxController.getRightX())));

        // auxRightBumper.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem,
        // Constants.ElevatorConstants.SOURCE_HEIGHT)
        // .alongWith(Commands.print("Elevator Source, Height: " +
        // Constants.ElevatorConstants.SOURCE_HEIGHT.in(Units.Meters))));

        elevatorSubsystem.setDefaultCommand(new RunCommand(() -> {
            double rightXAxis = m_auxController.getRightX();
            elevatorSubsystem.manualControl(rightXAxis);
        }, elevatorSubsystem).alongWith(Commands.print("value for controller: "+m_auxController.getRightX())));

        arm.setDefaultCommand(new RunCommand(() -> {
            double leftYAxis = m_auxController.getLeftY();
            arm.manualControl(leftYAxis);
        }, arm).alongWith(Commands.print("value for controller: "+m_auxController.getLeftY())));

        //Climber Bindings
        auxPovUP.onTrue(new RunCommand(() -> {climb.expand();}, climb))
                .onFalse(new RunCommand(() -> {climb.stop();}, climb));
        auxPovDOWN.onTrue(new RunCommand(() -> {climb.retract();}, climb))
                .onFalse(new RunCommand(() -> {climb.stop();}, climb));

        //Intake Bindings
        driveRightBumper.onTrue(new RunCommand(() -> {intake.feedWest();}, intake))
                     .onFalse(new RunCommand(() -> {intake.stop();}, intake));
        driveRightTrigger.onTrue(new RunCommand(() -> {intake.feedEast();}, intake))
                      .onFalse(new RunCommand(() -> {intake.stop();}, intake));

        driveLeftBumper.onTrue(new IntakeWithDetectionCommand(intake, intake.getCANrangeE())).onFalse(new RunCommand(() -> {intake.stop();}, intake));
        


    }

    public Command getAutonomousCommand() {
        return null;
    }
}

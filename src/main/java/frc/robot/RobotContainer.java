package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand.ArmSetPositionCommand;
import frc.robot.commands.ElevatorCommand.ElevatorSetPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveSubsystem.TunerConstants;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;

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
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // end of swerve drive stuff

    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    public static CommandXboxController m_auxController = new CommandXboxController(OIConstants.kAuxControllerPort);

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();

    private final Trigger auxY = m_auxController.y();
    private final Trigger auxA = m_auxController.a();
    private final Trigger auxB = m_auxController.b();
    private final Trigger auxX = m_auxController.x();
    private final Trigger auxRightBumper = m_auxController.rightBumper();

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
                                        -mathProfiles.exponentialDrive(m_driverController.getLeftY(), 2) * MaxSpeed)
                                .withVelocityY(
                                        -mathProfiles.exponentialDrive(m_driverController.getLeftX(), 2) * MaxSpeed)
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

        // reset the field-centric heading on left bumper press
        m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        // end of swerve drive bindings

        // Elevator and Arm bindings
        auxA.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT)
                .alongWith(Commands
                        .print("Elevator Level 1, Height: "
                                + Constants.ElevatorConstants.STAGE_1_HEIGHT.in(Units.Meters))));
        auxB.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT)
                .alongWith(Commands
                        .print("Elevator Level 2, Height: "
                                + Constants.ElevatorConstants.STAGE_2_HEIGHT.in(Units.Meters))));
        auxX.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT)
                .alongWith(Commands
                        .print("Elevator Level 3, Height: "
                                + Constants.ElevatorConstants.STAGE_3_HEIGHT.in(Units.Meters))));
        auxY.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT)
                .alongWith(Commands
                        .print("Elevator Level 4, Height: "
                                + Constants.ElevatorConstants.STAGE_4_HEIGHT.in(Units.Meters))));

        // auxRightBumper.onTrue(new ElevatorSetPositionCommand(elevatorSubsystem,
        // Constants.ElevatorConstants.SOURCE_HEIGHT)
        // .alongWith(Commands.print("Elevator Source, Height: " +
        // Constants.ElevatorConstants.SOURCE_HEIGHT.in(Units.Meters))));

        elevatorSubsystem.setDefaultCommand(new RunCommand(() -> {
            double rightXAxis = m_auxController.getRightX();
            elevatorSubsystem.manualControl(rightXAxis);
        }, elevatorSubsystem));

        auxRightBumper.onTrue(
                new ParallelCommandGroup(
                        new ElevatorSetPositionCommand(elevatorSubsystem,
                                Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.COROAL_STATION_ANGLE.in(Units.Degrees)))
                        .alongWith(Commands
                                .print("Delta Source, Height: "
                                        + Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA.in(Units.Meters)
                                        + " Angle: " + Constants.ArmConstant.COROAL_STATION_ANGLE.in(Units.Degrees))));


    }

    public Command getAutonomousCommand() {
        return null;
    }
}

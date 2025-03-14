package frc.robot;

import frc.robot.Constants.ArmConstant;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand.ArmSetPositionCommand;
import frc.robot.commands.ArmCommand.youParyArm;
import frc.robot.commands.AutoCommands.ElevatorAutonComomands;
import frc.robot.commands.AutoCommands.ArmAutonCommands;
import frc.robot.commands.AutoCommands.AutonIntakeWithDetectionCommand;
import frc.robot.commands.ElevatorCommand.ElevatorSetPositionCommand;
import frc.robot.commands.ElevatorCommand.youPary;
import frc.robot.commands.IntakeCommand.IntakeWithDetectionCommand;
import frc.robot.commands.IntakeCommand.IntakeHoldPositionCommand;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveSubsystem.TunerConstants;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    // swerve drive stuff
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
                                                                                    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 2.5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // end of swerve drive things

    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    public static CommandXboxController m_auxController = new CommandXboxController(OIConstants.kAuxControllerPort);
    public static XboxController m_Controller = new XboxController(OIConstants.kAuxControllerPort);

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
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

    private boolean sadMode = false;
    private final SendableChooser<Command> autoChooser;

    Command AEI_Scoring_L4 = new SequentialCommandGroup(
        new ParallelCommandGroup(
                new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA), 
                new ArmAutonCommands(arm,ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                ).withTimeout(2),
        new InstantCommand(() -> intake.feedWest()).withTimeout(2)
        );
    Command AEI_Scoring_L3 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))
                                    ).withTimeout(1.8),
            new InstantCommand(() -> intake.feedWest()).withTimeout(2)
                                            ).withTimeout(4);
    Command AEI_Scoring_L2 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))
                                    ).withTimeout(1.7),
            new InstantCommand(() -> intake.feedWest()).withTimeout(2)
                                            ).withTimeout(4);
    Command AEI_Scoring_L1 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))
                                    ).withTimeout(1.6),
            new InstantCommand(() -> intake.feedWest()).withTimeout(2)
                                            ).withTimeout(4);
    Command AEI_Source = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))
                                    ).withTimeout(1.9),
            new InstantCommand(() -> intake.feedEast()).withTimeout(2)
                                            ).withTimeout(4);
        Command AEI_Zero = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                                     ).withTimeout(1.6),
            new InstantCommand(() -> intake.stop()).withTimeout(2)
                                       );

                                       
        Command AEI_Scoring_L4_OCR_FIX = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA), 
                        new ArmAutonCommands(arm,ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                        ).withTimeout(1.3),
                        Commands.startEnd(()->intake.feedWest(),() -> intake.stop(),intake).withTimeout(2),
                        AEI_Zero
                );
        // Command SourceLoading = new SequentialCommandGroup(
        //         new ParallelCommandGroup(
        //                 new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA), 
        //                 new ArmAutonCommands(arm,ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))
        //                 ).withTimeout(2),
        //                 new FunctionalCommand(
        //                 ()->{
        //                 intake.feedEast();
        //                 },
        //                 interrupted -> intake.stop,
        //                 () -> intake.getCANrangeRight().getDistance().getValue().in(Centimeter) < 18   
        //                 ,
        //                 intake),
        //                 AEI_Zero
        //         );



    public RobotContainer() {

        NamedCommands.registerCommand("Elevator_L4_Happy", new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA).withTimeout(1));

        NamedCommands.registerCommand("Elevator_Zero", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_Source_Happy", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_L1_Happy", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_L2_Happy", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_L3_Happy", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_L4_Happy", new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA).withTimeout(1));

        NamedCommands.registerCommand("Arm_Zero", new ArmAutonCommands(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_Source_Happy", new ArmAutonCommands(arm, ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_L4_Happy", new ArmAutonCommands(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_L3_Happy", new ArmAutonCommands(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_L2_Happy", new ArmAutonCommands(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_L1_Happy", new ArmAutonCommands(arm, ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));


        NamedCommands.registerCommand("AE_L1_Happy", new ParallelCommandGroup(new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA), new ArmAutonCommands(arm, ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))).withTimeout(2));
        NamedCommands.registerCommand("AE_L2_Happy", new ParallelCommandGroup(new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA), new ArmAutonCommands(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))).withTimeout(2));
        NamedCommands.registerCommand("AE_L3_Happy", new ParallelCommandGroup(new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA), new ArmAutonCommands(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))).withTimeout(2));
        NamedCommands.registerCommand("AE_L4_Happy", new ParallelCommandGroup(new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA), new ArmAutonCommands(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))).andThen(new InstantCommand(() -> intake.feedWest()).withTimeout(3)).withTimeout(2));
        NamedCommands.registerCommand("AE_Source_Happy", new ParallelCommandGroup( new ArmAutonCommands(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)), new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA)).withTimeout(2));

        NamedCommands.registerCommand("AE_Zero", new ParallelCommandGroup( new ArmAutonCommands(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA)).withTimeout(2));






        NamedCommands.registerCommand("Intake_Scoring_West", new InstantCommand(() -> intake.feedWest()).withTimeout(300));

        NamedCommands.registerCommand("Intake_Source", 
        new SequentialCommandGroup(new AutonIntakeWithDetectionCommand(intake, intake.getCANrangeLeft(),intake.getCANrangeRight(), true), 
                                   new IntakeHoldPositionCommand(intake)));

        NamedCommands.registerCommand("AEI_Scoring_L1", AEI_Scoring_L1);
        NamedCommands.registerCommand("AEI_Scoring_L2", AEI_Scoring_L2);
        NamedCommands.registerCommand("AEI_Scoring_L3", AEI_Scoring_L3);
        NamedCommands.registerCommand("AEI_Scoring_L4", AEI_Scoring_L4);
        NamedCommands.registerCommand("AEI_Source", AEI_Source);
        NamedCommands.registerCommand("AEI_Zero", AEI_Zero);
        NamedCommands.registerCommand("AEI_Scoring_L4_OCR_FIX", AEI_Scoring_L4_OCR_FIX);
        

        autoChooser = AutoBuilder.buildAutoChooser("Taxi");
        SmartDashboard.putData("Auto Chooser", autoChooser);





      
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

        ///////////////////////////////////////GIFTS AND TRINKETS (NON DRIVE BINDS)

        // Elevator and Arm bindings, move to zero position
        auxRightTrigger.onTrue(
                new ParallelCommandGroup(
                new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA)
                .alongWith(Commands.print("Elevator Zero Position, Height: " + Constants.ElevatorConstants.ELEVATOR_BASE_DELTA.in(Units.Meters))),
                
                new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Base/zero Position, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))))
        );
        // auxRightTrigger.onTrue(new InstantCommand(() -> {
        //         new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
        //         .alongWith(Commands.print("Arm Base, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))).schedule();
        // }));
                
        // auxRightTrigger.onTrue(new RunCommand(() -> {arm.setState(0);}, arm));
        
        //SADMODE TRIGGER
        auxLeftBumper.onTrue(new InstantCommand(() -> {
                sadMode = true;
                System.out.println("sadMode: " + sadMode);
        }))
        .onFalse(new InstantCommand(() -> {
                sadMode = false;
                System.out.println("sadMode: " + sadMode);
        }));
        //SOURCE
        auxRightBumper.onTrue(new InstantCommand(() -> {
                new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA)
                        .alongWith(Commands.print("Elevator Source, Height: " + Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA.in(Units.Meters))).schedule();
                if(sadMode){
                        new ArmSetPositionCommand(arm, ArmConstant.SAD_CORAL_STATION_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("Arm Sad Source, Angles: " + ArmConstant.SAD_CORAL_STATION_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(5);
                }
                else{
                        new ArmSetPositionCommand(arm, ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("Arm Source, Angles: " + ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(-5);
                }
        }));
        //L1
        auxA.onTrue(new InstantCommand(() -> {
                new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator Level 1, Height: " + Constants.ElevatorConstants.STAGE_1_HEIGHT.in(Units.Meters))).schedule();
                if(sadMode){
                        new ArmSetPositionCommand(arm, ArmConstant.SAD_STAGE_1_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("Arm Sad Level 1, Angles: " + ArmConstant.SAD_STAGE_1_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(-1);
                }
                else{
                        new ArmSetPositionCommand(arm, ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("Arm Level 1, Angles: " + ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(1);
                }
        }));
        //L2
        auxB.onTrue(new InstantCommand(() -> {
                new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator Level 2, Height: " + Constants.ElevatorConstants.STAGE_2_HEIGHT.in(Units.Meters))).schedule();
                if(sadMode){
                        new ArmSetPositionCommand(arm, ArmConstant.SAD_STAGE_2_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("Arm Sad Level 2, Angles: " + ArmConstant.SAD_STAGE_2_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(-2);
                }
                else{
                        new ArmSetPositionCommand(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("Arm Level 2, Angles: " + ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(2);
                }
        }));
        //L3
        auxX.onTrue(new InstantCommand(() -> {
                new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator Level 3, Height: " + Constants.ElevatorConstants.STAGE_3_HEIGHT.in(Units.Meters))).schedule();
                if(sadMode){
                        new ArmSetPositionCommand(arm, ArmConstant.SAD_STAGE_3_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm Sad Level 3, Angles: " + ArmConstant.SAD_STAGE_3_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(-3);
                }
                else{
                        new ArmSetPositionCommand(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm Level 3, Angles: " + ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(3);
                }
        }));
        //L4
        auxY.onTrue(new InstantCommand(() -> {
                new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator Level 4, Height: " + Constants.ElevatorConstants.STAGE_4_HEIGHT.in(Units.Meters))).schedule();
                if(sadMode){
                        new ArmSetPositionCommand(arm, ArmConstant.SAD_STAGE_4_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm Sad Level 4, Angles: " + ArmConstant.SAD_STAGE_4_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(-4);
                }
                else{
                        new ArmSetPositionCommand(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm Level 4, Angles: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))).schedule();
                        arm.setState(4);

                }
        }));

        //SADNESS aura
        /*auxLeftBumper.onTrue(new RunCommand(() -> { //sad commands

                auxRightBumper.onTrue(new ArmSetPositionCommand(arm, ArmConstant.SAD_CORAL_STATION_ANGLE_VERTICAL.in(Degrees))           //SOURCE
                        .alongWith(Commands.print("Arm Source, Angles: " + ArmConstant.SAD_CORAL_STATION_ANGLE_VERTICAL.in(Degrees))));
                auxA.onTrue(new ArmSetPositionCommand(arm, ArmConstant.SAD_STAGE_1_ANGLE_VERTICAL.in(Degrees))                           //L1
                        .alongWith(Commands.print("Arm Source, Angles: " + ArmConstant.SAD_STAGE_1_ANGLE_VERTICAL.in(Degrees))));
                auxB.onTrue(new ArmSetPositionCommand(arm, ArmConstant.SAD_STAGE_2_ANGLE_VERTICAL.in(Degrees))                           //L2
                        .alongWith(Commands.print("Arm Level 2, Angles: " + ArmConstant.SAD_STAGE_2_ANGLE_VERTICAL.in(Degrees))));
                auxX.onTrue(new ArmSetPositionCommand(arm, ArmConstant.SAD_STAGE_3_ANGLE_VERTICAL.in(Degrees))                               //L3
                        .alongWith(Commands.print("Arm Level 3, Angles: " + ArmConstant.SAD_STAGE_3_ANGLE_VERTICAL.in(Degrees))));
                auxY.onTrue(new ArmSetPositionCommand(arm, ArmConstant.SAD_STAGE_4_ANGLE_VERTICAL.in(Degrees))                               //L4
                        .alongWith(Commands.print("Arm Level 4, Angles: " + ArmConstant.SAD_STAGE_4_ANGLE_VERTICAL.in(Degrees))));

        }, elevatorSubsystem))
        .onFalse(new RunCommand(() -> { //normal commands

                auxRightBumper.onTrue(new ArmSetPositionCommand(arm, ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))               //SOURCE
                        .alongWith(Commands.print("Arm Source, Angles: " + ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))));
                auxA.onTrue(new ArmSetPositionCommand(arm, ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))                               //L1
                        .alongWith(Commands.print("Arm Source, Angles: " + ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))));
                auxB.onTrue(new ArmSetPositionCommand(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))                               //L2
                        .alongWith(Commands.print("Arm Level 2, Angles: " + ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))));
                auxX.onTrue(new ArmSetPositionCommand(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))                               //L3
                        .alongWith(Commands.print("Arm Level 3, Angles: " + ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))));
                auxY.onTrue(new ArmSetPositionCommand(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))                               //L4
                        .alongWith(Commands.print("Arm Level 4, Angles: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))));
                
        }, elevatorSubsystem));*/
        


        //default elevator and arm manual control
        m_auxController.start().whileTrue(new youPary(elevatorSubsystem));
        elevatorSubsystem.setDefaultCommand(new RunCommand(() -> {
            double rightXAxis = m_auxController.getRightY();
            elevatorSubsystem.manualControl(-rightXAxis*0.25+0.035);
        }, elevatorSubsystem)
        .alongWith(Commands.print("Elevator Manual Controlling: " + m_auxController.getRightX())));

        m_auxController.back().whileTrue(new youParyArm(arm));
        arm.setDefaultCommand(new RunCommand(() -> {
            double leftYAxis = m_auxController.getLeftX();
            arm.manualControl(-leftYAxis*0.2);
        }, arm)
        .alongWith(Commands.print("Arm Manual Controlling: "+m_auxController.getLeftY())));

        //Climber Bindings
        auxPovUP.onTrue(new RunCommand(() -> {climb.expand();}, climb)).onFalse(new RunCommand(() -> {climb.stop();}, climb));
        auxPovDOWN.onTrue(new RunCommand(() -> {climb.retract();}, climb)).onFalse(new RunCommand(() -> {climb.stop();}, climb));

        // auxPovUP.onTrue(new RunCommand(() -> {climb.expand();}, climb));
        // System.out.println("climb" );
        // auxPovDOWN.onTrue(new RunCommand(() -> {climb.retract();}, climb));


        auxPovLEFT.onTrue(new RunCommand(() -> {intake.feedEast(0.1);}, intake)).onFalse(new RunCommand(() -> {intake.stop();}, intake));
        auxPovRIGHT.onTrue(new RunCommand(() -> {intake.feedWest(0.1);}, intake)).onFalse(new RunCommand(() -> {intake.stop();}, intake));


        //aux control the intake from the source
        //this is for getting the game pieces from the source
        driveLeftTrigger.onTrue(new RunCommand(() ->{//source intake


                // switch (arm.getStateE()) {
                //         case SOURCE:
                //                 new SequentialCommandGroup(
                //                         new IntakeWithDetectionCommand(intake, intake.getCANrangeE(), false),
                //                         new IntakeHoldPositionCommand(intake, intake.getCurrentPosition_Rotations())).schedule();
                //             break;
                //         case SAD_SOURCE:
                //             new IntakeWithDetectionCommand(intake, intake.getCANrangeE(), true).schedule();
                //             new IntakeHoldPositionCommand(intake, intake.getCurrentPosition_Rotations()).schedule();
                //             break;
                //         default:
                //             intake.stop();
                //             break;
                //     }
                if(arm.getState() == 5){
                        System.out.println("arm source state 5, sad is false");
                        new SequentialCommandGroup(
                        new IntakeWithDetectionCommand(intake, intake.getCANrangeLeft(),intake.getCANrangeRight(), true), //sad is false
                        new IntakeHoldPositionCommand(intake)
                        ).schedule();
                }
                else if(arm.getState() == -5){
                        System.out.println("arm source state -5, sad is true");
                        new SequentialCommandGroup(
                        new IntakeWithDetectionCommand(intake, intake.getCANrangeLeft(),intake.getCANrangeRight(), false), //sad is true
                        new IntakeHoldPositionCommand(intake)
                        ).schedule();


                }
                else{
                        System.out.println("nothing for now");
                        intake.stop();
                }
        }, intake))
        .onFalse(new RunCommand(() -> {
                intake.stop();
        }, intake));


        // //driver scoring, only control the
        driveRightTrigger.onTrue(new RunCommand(() ->{ //only scoring
                if(arm.getState() == 1 ||
                   arm.getState() == 2 ||
                   arm.getState() == -3 ||
                   arm.getState() == -4){
                        intake.feedEast();
                     
                }
                else if(arm.getState() == -1 ||
                        arm.getState() == -2 ||
                        arm.getState() == 3 ||
                        arm.getState() == 4){

                        intake.feedWest();
                }
                else{
                        intake.stop();
                }
        }, intake))
        .onFalse(new RunCommand(() ->{
                intake.stop();
        }, intake));

        
        

            //delete if enum works
        
        //Intake Bindings
        //driver scoring, only control the intake and out take spinning
        // driveRightTrigger.onTrue(new RunCommand(() -> {
        //         // Use the helper method
        //         handleIntakeByArmState(arm.getStateE(), 0.6);
        //     }, intake))
        //     .onFalse(new RunCommand(() -> {
        //         intake.stop();
        //     }, intake));




    }

    public Command getAutonomousCommand() {
//        return new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA);
        return autoChooser.getSelected();
        // try {
        //         PathPlannerPath path = PathPlannerPath.fromPathFile("blueUpPreloadPath");
        //         return AutoBuilder.followPath(path);
        // } catch (FileVersionException e) {
        //         // TODO Auto-generated catch block
        //         e.printStackTrace();
        // } catch (IOException e) {
        //         // TODO Auto-generated catch block
        //         e.printStackTrace();
        // } catch (ParseException e) {
        //         // TODO Auto-generated catch block
        //         e.printStackTrace();
        // }
        // return Commands.none();


    }

    private void handleIntakeByArmState(ArmState state, double speed) {
    switch (state) {
        case LEVEL1:
        case LEVEL2:
        case SAD_LEVEL3:
        case SAD_LEVEL4:
            intake.feedEast(speed);
            break;
        case SAD_LEVEL1:
        case SAD_LEVEL2:
        case LEVEL3:
        case LEVEL4:
            intake.feedWest(speed);
            break;
        default:
            intake.stop();
            break;
    }
}
}

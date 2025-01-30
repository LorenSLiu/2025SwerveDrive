package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.Pigeon2;

@SuppressWarnings("all") // you know we all hate this

public class SwerveSubsystem extends SubsystemBase {
    private double m_currentRotation = 0.0;
    CommandXboxController m_driverController;
    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;
    // define each module
    SwerveModule frontLeftModule = new SwerveModule(
            frc.robot.Constants.DriveConstants.kFrontLeftDrivingID,
            frc.robot.Constants.DriveConstants.kFrontLeftTurningID,
            frc.robot.Constants.DriveConstants.kFrontLeftTurningEncoderID,
            frc.robot.Constants.DriveConstants.kFrontLeftAbsoluteEncoderOffsetRadians);

    SwerveModule frontRightModule = new SwerveModule(
            frc.robot.Constants.DriveConstants.kFrontRightDrivingID,
            frc.robot.Constants.DriveConstants.kFrontRightTurningID,
            frc.robot.Constants.DriveConstants.kFrontRightTurningEncoderID,
            frc.robot.Constants.DriveConstants.kFrontRightAbsoluteEncoderOffsetRadians);

    SwerveModule backLeftModule = new SwerveModule(
            frc.robot.Constants.DriveConstants.kBackLeftDrivingID,
            frc.robot.Constants.DriveConstants.kBackLeftTurningID,
            frc.robot.Constants.DriveConstants.kBackLeftTurningEncoderID,
            frc.robot.Constants.DriveConstants.kBackLeftAbsoluteEncoderOffsetRadians);

    SwerveModule backRightModule = new SwerveModule(
            frc.robot.Constants.DriveConstants.kBackRightDrivingID,
            frc.robot.Constants.DriveConstants.kBackRightTurningID,
            frc.robot.Constants.DriveConstants.kBackRightTurningEncoderID,
            frc.robot.Constants.DriveConstants.kBackRightAbsoluteEncoderOffsetRadians);

    // Translation 2d for the swerve drive/kinematics, kinematics need to the each
    // module's location relative to the center
    // of the robot and then the kinematics object will calculate the speed and
    // angle for each module

    // define a kinematics object
    SwerveDriveKinematics m_Kinematics = DriveConstants.kDriveKinematics;

    public PIDController m_botAnglePID = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI,
            ModuleConstants.kTurningD);

    public SwerveSubsystem() {
        // m_driverController = driverController; // Set the controller
        pigeon = new Pigeon2(frc.robot.Constants.OperatorConstants.GYRO_ID);// add id later
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100);// Hz booom
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
        // m_driverController = driverController;
        System.out.println("SwerveSubsystem constructor");
    }

    public void setChassisSpeeds(ChassisSpeeds desired) {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        swerveModuleStates = m_Kinematics.toSwerveModuleStates(desired);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // xSpeed = applyDeadband(xSpeed, 0.1);
        // ySpeed = applyDeadband(ySpeed, 0.1);
        // rot = applyDeadband(rot, 0.1);
        // above doesn't need it as we already have the deadband in the controller in
        // RobotContainer.java
        // The origin is always blue. When our alliance is red, X and Y need to be
        // inverted
        
        var alliance = DriverStation.getAlliance();
        var invert = 1;
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            invert = -1;
        }
        // Create field relative ChassisSpeeds for controlling Swerve
        var chassisSpeeds = ChassisSpeeds
                .fromFieldRelativeSpeeds(xSpeed * invert, ySpeed * invert, rot, pigeon.getRotation2d());// convert field
                                                                                                        // relative to
                                                                                                        // robot
                                                                                                        // relative, not
                                                                                                        // sure if this
                                                                                                        // shit will
                                                                                                        // work or not
                                                                                                        // yet

        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;// havent figured out how does the
                                                                                  // conversion work yet, copied from
                                                                                  // last year's code
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        // System.out.println("SwerveSubsystem drive");
        // System.out.println("xSpeed: " + xSpeedDelivered);
        // System.out.println("ySpeed: " + ySpeedDelivered);
        // System.out.println("rot: " + rotDelivered);


        setChassisSpeeds(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    }

    public void zeroChassisSpeeds() {
        setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    // public void drive(double xSpeed, double ySpeed, double rot, boolean
    // fieldRelative, boolean rateLimit) {
    // // Optional: Implement rate limiting here if needed
    // ChassisSpeeds chassisSpeeds = fieldRelative
    // ? ChassisSpeeds.fromFieldRelativeSpeeds(
    // xSpeed, ySpeed, rot, Rotation2d.fromDegrees(yaw.getValue().getDegrees()))
    // : new ChassisSpeeds(xSpeed, ySpeed, rot);

    // setChassisSpeeds(chassisSpeeds);
    // }

    // jsut for the information for advantage scope, you might want to consider:, as
    // the array is not a struct and it will be removed in 2026
    // StructArrayPublisher<SwerveModuleState> publisher =
    // NetworkTableInstance.getDefault()
    // .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    @Override
    public void periodic() {
        // ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(
        // // leftside pushing forward will make robot move forward
        // m_driverController.getLeftY(),
        // // leftside pushing right will make robot move right
        // m_driverController.getLeftX(),
        // // rightside pushing right will make robot turn right
        // m_driverController.getRightX());

        // System.out.println(m_chassisSpeeds);
        // setChassisSpeeds(m_chassisSpeeds);

        // advantage scope stuffS
        // This method will be called once per scheduler run
        System.out.println("SwerveSubsystem periodic");
        System.out.println("Front Left Module State: " + frontLeftModule.getSwerveModuleState());
        double loginstate[] = {

                // switch back to original

                // frontLeft.getDriveEncoder().getVelocity(),
                // frontLeft.getTurnEncoder().getVelocity(),
                // frontRight.getDriveEncoder().getVelocity(),
                // frontRight.getTurnEncoder().getVelocity(),
                // backLeft.getDriveEncoder().getVelocity(),
                // backLeft.getTurnEncoder().getVelocity(),
                // backRight.getDriveEncoder().getVelocity(),
                // backRight.getTurnEncoder().getVelocity()
        };
        SmartDashboard.putNumberArray("Peridoc", loginstate);
        if (frontLeftModule.getSwerveModuleState().angle.getDegrees() > 0.1
                || frontLeftModule.getSwerveModuleState().angle.getDegrees() < -0.1) {
                    System.out.println("----------------------------------------");
            System.out.println("Front Left Module State: " + frontLeftModule.getSwerveModuleState());

            System.out.println("SwerveSubsystem periodic");
            System.out.println("Front Left Module State: " + frontLeftModule.getSwerveModuleState());
            System.out.println("Front Right Module State: " + frontRightModule.getSwerveModuleState());
            System.out.println("Back Left Module State: " + backLeftModule.getSwerveModuleState());
            System.out.println("Back Right Module State: " + backRightModule.getSwerveModuleState());
        }
    }

}

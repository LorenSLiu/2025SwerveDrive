package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.configs.Slot0Configs;

@SuppressWarnings("all")//you know we all hate this

public class SwerveSubsystem extends SubsystemBase {
    CommandXboxController m_driverController;

    //define each module
    SwerveModule frontLeftModule  = new SwerveModule(
        frc.robot.Constants.DriveConstants.kFrontLeftDrivingID, 
        frc.robot.Constants.DriveConstants.kFrontLeftTurningID,
        frc.robot.Constants.DriveConstants.kFrontLeftTurningEncoderID,
        frc.robot.Constants.DriveConstants.kFrontLeftAbsoluteEncoderOffsetRadians,
        frc.robot.Constants.DriveConstants.kFrontLeftTurningEncoderReversed);

    SwerveModule frontRightModule = new SwerveModule(
        frc.robot.Constants.DriveConstants.kFrontRightDrivingID, 
        frc.robot.Constants.DriveConstants.kFrontRightTurningID,
        frc.robot.Constants.DriveConstants.kFrontRightTurningEncoderID,
        frc.robot.Constants.DriveConstants.kFrontRightAbsoluteEncoderOffsetRadians,
        frc.robot.Constants.DriveConstants.kFrontRightTurningEncoderReversed);

    SwerveModule backLeftModule   = new SwerveModule(
        frc.robot.Constants.DriveConstants.kBackLeftDrivingID,
        frc.robot.Constants.DriveConstants.kBackLeftTurningID,
        frc.robot.Constants.DriveConstants.kBackLeftTurningEncoderID,
        frc.robot.Constants.DriveConstants.kBackLeftAbsoluteEncoderOffsetRadians,
        frc.robot.Constants.DriveConstants.kBackLeftTurningEncoderReversed);

    SwerveModule backRightModule  = new SwerveModule(
        frc.robot.Constants.DriveConstants.kBackRightDrivingID,
        frc.robot.Constants.DriveConstants.kBackRightTurningID,
        frc.robot.Constants.DriveConstants.kBackRightTurningEncoderID,
        frc.robot.Constants.DriveConstants.kBackRightAbsoluteEncoderOffsetRadians,
        frc.robot.Constants.DriveConstants.kBackRightTurningEncoderReversed);  

    //Translation 2d for the swerve drive/kinematics, kinematics need to the each module's location relative to the center 
    //of the robot and then the kinematics object will calculate the speed and angle for each module
    Translation2d m_frontLeftLocation = new Translation2d(
        (frc.robot.Constants.DriveConstants.kmDriveWidth) / 2, 
        frc.robot.Constants.DriveConstants.kmDriveLength / 2);
    Translation2d m_frontRightLocation = new Translation2d(
        frc.robot.Constants.DriveConstants.kmDriveWidth / 2, 
        -frc.robot.Constants.DriveConstants.kmDriveLength / 2);
    Translation2d m_backLeftLocation = new Translation2d(
        -frc.robot.Constants.DriveConstants.kmDriveWidth / 2, 
        frc.robot.Constants.DriveConstants.kmDriveLength / 2);
    Translation2d m_backRightLocation = new Translation2d(
        -frc.robot.Constants.DriveConstants.kmDriveWidth / 2, 
        -frc.robot.Constants.DriveConstants.kmDriveLength / 2);

    //define a kinematics object
    SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, 
        m_frontRightLocation, 
        m_backLeftLocation, 
        m_backRightLocation
    );


    public SwerveSubsystem(CommandXboxController driverController) {
        m_driverController = driverController;
        System.out.println("SwerveSubsystem constructor");
    }


    public void setChassisSpeeds(ChassisSpeeds desired){
        SwerveModuleState[] swerveModuleStates = m_Kinematics.toSwerveModuleStates(desired);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }

    //jsut for the information for advantage scope, you might want to consider:, as the array is not a struct and it will be removed in 2026
    //StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    //.getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    @Override
    public void periodic() {
        ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(
            //leftside pushing forward will make robot move forward
            m_driverController.getLeftY(),
            //leftside pushing right will make robot move right
            m_driverController.getLeftX(),
            //rightside pushing right will make robot turn right
            m_driverController.getRightX()
        );

        System.out.println(m_chassisSpeeds);
        setChassisSpeeds(m_chassisSpeeds);




        //advantage scope stuff
        // This method will be called once per scheduler run
        double loginstate[] = {
            frontLeftModule.getSwerveModuleState().angle.getDegrees(),
            frontLeftModule.getSwerveModuleState().speedMetersPerSecond,

            frontRightModule.getSwerveModuleState().angle.getDegrees(),
            frontRightModule.getSwerveModuleState().speedMetersPerSecond,

            backLeftModule.getSwerveModuleState().angle.getDegrees(),
            backLeftModule.getSwerveModuleState().speedMetersPerSecond,

            backRightModule.getSwerveModuleState().angle.getDegrees(),
            backRightModule.getSwerveModuleState().speedMetersPerSecond,
            
            // frontLeft.getDriveEncoder().getVelocity(),
            // frontLeft.getTurnEncoder().getVelocity(),
            // frontRight.getDriveEncoder().getVelocity(),
            // frontRight.getTurnEncoder().getVelocity(),
            // backLeft.getDriveEncoder().getVelocity(),
            // backLeft.getTurnEncoder().getVelocity(),
            // backRight.getDriveEncoder().getVelocity(),
            // backRight.getTurnEncoder().getVelocity()
        };
        SmartDashboard.putNumberArray("SwerveState", loginstate);
    }
    
}

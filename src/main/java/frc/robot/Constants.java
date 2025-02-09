// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  static Translation2d m_frontLeftLocation = new Translation2d(
      (frc.robot.Constants.DriveConstants.kmDriveWidth) / 2,
      frc.robot.Constants.DriveConstants.kmDriveLength / 2);
  static Translation2d m_frontRightLocation = new Translation2d(
      frc.robot.Constants.DriveConstants.kmDriveWidth / 2,
      -frc.robot.Constants.DriveConstants.kmDriveLength / 2);
  static Translation2d m_backLeftLocation = new Translation2d(
      -frc.robot.Constants.DriveConstants.kmDriveWidth / 2,
      frc.robot.Constants.DriveConstants.kmDriveLength / 2);
  static Translation2d m_backRightLocation = new Translation2d(
      -frc.robot.Constants.DriveConstants.kmDriveWidth / 2,
      -frc.robot.Constants.DriveConstants.kmDriveLength / 2);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int GYRO_ID = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static class ElevatorConstants {
    public static final int kElevatorMotorID = 1;

    public static final double kElevatorP = 4.8;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0.1;
    public static final double kElevatorS = 0.25;
    public static final double kElevatorV = 0.12;
    public static final double kElevatorA = 0.1;


    public static final double kElevatorMinOutput = -1;
    public static final double kElevatorMaxOutput = 1;
    public static final int kElevatorCurrentLimit = 60;

    public static final double TOLERANCE = 1; // 1 rotation from motor, 2.51cm

     // Software limits (to prevent breaking the elevator)
     public static final double kMinHeight = 0;      // Lowest safe position
     public static final double kMaxHeight = 72.8; // Highest safe position
     public static final double kManualSpeedMultiplier = 1000; // Adjust for fine control


    public static final double ENCODER_TICKS_PER_REV = 2048; // Kraken default
    private static final double GEAR_RATIO = 3; // 3:1, confirmed
    private static final double PULLEY_DIAMETER = 0.048; // 4.8cm
    public static final double PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER * Math.PI; // cm

    public static final double kElevatorEncoderDistancePerPulse = PULLEY_CIRCUMFERENCE / (ENCODER_TICKS_PER_REV * GEAR_RATIO);

    //conversion factors
    public static final double METERS_PER_TICK = PULLEY_CIRCUMFERENCE/(ENCODER_TICKS_PER_REV * GEAR_RATIO);
    public static final double METERS_PER_ROTATION = PULLEY_CIRCUMFERENCE / GEAR_RATIO;

    // Elevator stages
    public static final double STAGE_1_HEIGHT_METERS = 0.46;  //46cm
    public static final double STAGE_2_HEIGHT_METERS = 0.81;  //81cm
    public static final double STAGE_3_HEIGHT_METERS = 1.21;  //121cm 
    public static final double STAGE_4_HEIGHT_METERS = 1.83;  //183cm 
    public static final double SOURCE_HEIGHT_METERS = 0.95;  //95cm


    public static final double STAGE_1_HEIGHT_ROTATIONS = (STAGE_1_HEIGHT_METERS * GEAR_RATIO) / (Math.PI * PULLEY_DIAMETER);
    public static final double STAGE_2_HEIGHT_ROTATIONS = (STAGE_2_HEIGHT_METERS * GEAR_RATIO) / (Math.PI * PULLEY_DIAMETER);
    public static final double STAGE_3_HEIGHT_ROTATIONS = (STAGE_3_HEIGHT_METERS * GEAR_RATIO) / (Math.PI * PULLEY_DIAMETER);
    public static final double STAGE_4_HEIGHT_ROTATIONS = (STAGE_4_HEIGHT_METERS * GEAR_RATIO) / (Math.PI * PULLEY_DIAMETER);
    public static final double SOURCE_HEIGHT_ROTATIONS  = (SOURCE_HEIGHT_METERS  * GEAR_RATIO) / (Math.PI * PULLEY_DIAMETER);




  }


  public static class ModuleConstants {
    // Calculations required for driving motor conversion factors and feed forward
//    public static final double kDrivingMotorFreeSpeedRps = 6000 / 60;// what the hell is this
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = 3.56;// need to change it, talk with Stanford

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.05;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 0.01;// kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.05;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final int kDrivingMotorCurrentLimit = 80; // amps
    public static final int kTurningMotorCurrentLimit = 50; // amps

    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction); // meters per second
  }

  public static class DriveConstants {

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation
        );

    public static final double kMaxSpeedMetersPerSecond = 6.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    // CAN bus ID
    public static final int kFrontLeftTurningID = 1;
    public static final int kFrontLeftDrivingID = 2;
    public static final int kFrontRightTurningID = 3;
    public static final int kFrontRightDrivingID = 4;
    public static final int kBackLeftTurningID = 5;
    public static final int kBackLeftDrivingID = 6;
    public static final int kBackRightTurningID = 7;
    public static final int kBackRightDrivingID = 8;
    public static final int kFrontLeftTurningEncoderID = 9;
    public static final int kFrontRightTurningEncoderID = 10;
    public static final int kBackLeftTurningEncoderID = 11;
    public static final int kBackRightTurningEncoderID = 12;

    // Module offsets
    public static final double kFrontLeftAbsoluteEncoderOffsetRadians = 0.0;
    public static final double kFrontRightAbsoluteEncoderOffsetRadians = 0;
    public static final double kBackLeftAbsoluteEncoderOffsetRadians = 0;
    public static final double kBackRightAbsoluteEncoderOffsetRadians = 0;

    // Moudle reversed or nah
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    // frame size
    public static final double kmDriveWidth = Units.inchesToMeters(26.5);
    public static final double kmDriveLength = Units.inchesToMeters(26.5);

  }
  public static final class AutoConstants {//copied from last year's code, not sure how this works, delete if needed
    public static final double kSwerveDiscreteTimestep = 0.02;
    public static final double kSwerveDriveRadiusMeters = Units.inchesToMeters(DriveConstants.kmDriveWidth) / 2;
   
  }
}

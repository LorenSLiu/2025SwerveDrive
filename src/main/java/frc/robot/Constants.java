// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;

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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int GYRO_ID = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static class ClimbConstant {
    public static final int kClimbMotorID = 40;
    public static final CANBus kClimbCANbus = new CANBus("rio");
    public static final int kClimbCurrentLimit = 80;
  }

  public static class IntakeConstant {
    public static final int kIntakeMotorID = 31;
    public static final CANBus kIntakeCANbus = new CANBus("rio");
    public static final int kIntakeCurrentLimit = 40;

    public static final int kCANrange1ID = 32;
    //public static final int kCANrange2ID = ;
  }

  public static class ArmConstant {
    public static final CANBus kArmCANbus = new CANBus("rio");

    public static final int kArmMotorID = 30;

    public static final double TOLERANCE = 0.01;

    public static final double ArmGearRatio = 52.09722 / 1;

    public static final Distance ArmLength = Inches.of(16.1);

    public static final Angle ARM_BASE_ANGLE_VERTICAL       = Degrees.of(0);

    public static final Angle CORAL_STATION_ANGLE_VERTICAL  = Degrees.of(-36);
    public static final Angle STAGE_1_ANGLE_VERTICAL        = Degrees.of(23); 
    public static final Angle STAGE_2_ANGLE_VERTICAL        = Degrees.of(26);
    public static final Angle STAGE_3_ANGLE_VERTICAL        = Degrees.of(-220);
    public static final Angle STAGE_4_ANGLE_VERTICAL        = Degrees.of(-224);

    //public static final Angle SAD_CORAL_STATION_ANGLE_VERTICAL  = Degrees.of(CORAL_STATION_ANGLE_VERTICAL.in(Degrees) * -1);
    public static final Angle SAD_CORAL_STATION_ANGLE_VERTICAL  = Degrees.of(CORAL_STATION_ANGLE_VERTICAL.in(Degrees) * -1);
    public static final Angle SAD_STAGE_1_ANGLE_VERTICAL        = Degrees.of(STAGE_1_ANGLE_VERTICAL.in(Degrees) * -1); 
    public static final Angle SAD_STAGE_2_ANGLE_VERTICAL        = Degrees.of(STAGE_2_ANGLE_VERTICAL.in(Degrees) * -1);
    public static final Angle SAD_STAGE_3_ANGLE_VERTICAL        = Degrees.of(STAGE_3_ANGLE_VERTICAL.in(Degrees) * -1);
    public static final Angle SAD_STAGE_4_ANGLE_VERTICAL        = Degrees.of(STAGE_4_ANGLE_VERTICAL.in(Degrees) * -1);

    //random stuff
    public static final double test = 0.0;
    //public static final double kArmP = 0.05;
    public static final double kArmP = 0.04;
    public static final double kArmI = 0.004;
    public static final double kArmD = 0;

    public static final double kMaxAngle = 160;



    // public static final double kArmS = 0.25;
    // public static final double kArmV = 6.47;
    // public static final double kArmA = 0.01; //r^s^2
    // public static final double kArmG = 0.11; //r^s

    public static final double kArmMinOutput = -1;
    public static final double kArmMaxOutput = 1;
    public static final int kArmCurrentLimit = 60;
  }

  public static class ElevatorConstants {
    public static final CANBus kElevatorCANbus = new CANBus("rio");

    public static final int kElevatorMotorID = 21;
    public static final int kElevatorMotorFollowerID = 20;

    public static final double kElevatorP = 0.05;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorG = 0;
    public static final double kElevatorS = 0.1;


    // public static final double kElevatorP = 7;
    // public static final double kElevatorI = 0;
    // public static final double kElevatorD = 0.1;
    // public static final double kElevatorS = 0.1;
    // public static final double kElevatorV = 3.11;
    // public static final double kElevatorA = 0.5;

    public static final double kElevatorMinOutput = -1;
    public static final double kElevatorMaxOutput = 1;
    public static final int kElevatorCurrentLimit = 60;

    //public static final double TOLERANCE = 1; // 1 rotation from motor, 2.51cm// determine if need to delete

    public static final Distance kMinHeight = Inches.of(0); // Lowest safe position
    public static final Distance kMaxHeight = Inches.of(27); // Highest safe position

    public static final double kManualSpeedMultiplier = 1000; // Adjust for fine control

//    public static final double ENCODER_TICKS_PER_REV = 2048; // Kraken default
    public static final double GEAR_RATIO = 3/1;
    public static final Distance SprocketRadius = Inches.of(1.638/2);


    // public static final double kElevatorEncoderDistancePerPulse = PULLEY_CIRCUMFERENCE
    //     / (ENCODER_TICKS_PER_REV * GEAR_RATIO);

    // conversion factors
    //public static final double METERS_PER_TICK = PULLEY_CIRCUMFERENCE / (ENCODER_TICKS_PER_REV * GEAR_RATIO);
    //public static final double METERS_PER_ROTATION = PULLEY_CIRCUMFERENCE / GEAR_RATIO;

    // Elevator stages raw data (wrong right now i think)
    public static final Distance Elevator_HEIGHT_NOUGHT = Inches.of(37.433); // ok
    public static final Distance STAGE_1_HEIGHT         = Meters.of(0.46); // 46cm
    public static final Distance STAGE_2_HEIGHT         = Meters.of(0.81); // 81cm
    public static final Distance STAGE_3_HEIGHT         = Meters.of(1.21); // 121cm
    public static final Distance STAGE_4_HEIGHT         = Meters.of(1.83); // 183cm
    public static final Distance SOURCE_HEIGHT          = Meters.of(0.95); // 95cm

    //Elevator stages effective data
    public static final Distance ELEVATOR_SOURCE_DELTA = Inches.of(14);//check
    public static final Distance ELEVATOR_BASE_DELTA = Inches.of(0);
    public static final Distance STAGE_1_HEIGHT_DELTA  = Inches.of(0); 
    public static final Distance STAGE_2_HEIGHT_DELTA  = Inches.of(11); 
    public static final Distance STAGE_3_HEIGHT_DELTA  = Inches.of(0);
    public static final Distance STAGE_4_HEIGHT_DELTA  = Inches.of(27);

  }

  public static class ModuleConstants {
    // Calculations required for driving motor conversion factors and feed forward
    // public static final double kDrivingMotorFreeSpeedRps = 6000 / 60;// what the
    // hell is this
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
    public static Distance kmDriveWidth = Inches.of(26.5);
    public static Distance kmDriveLength = Inches.of(26.5);

    static Translation2d m_frontLeftLocation = new Translation2d(
        frc.robot.Constants.DriveConstants.kmDriveWidth.in(Inches) / 2,
        frc.robot.Constants.DriveConstants.kmDriveLength.in(Inches) / 2);

    static Translation2d m_frontRightLocation = new Translation2d(
        frc.robot.Constants.DriveConstants.kmDriveWidth.in(Inches) / 2,
        -frc.robot.Constants.DriveConstants.kmDriveLength.in(Inches) / 2);

    static Translation2d m_backLeftLocation = new Translation2d(
        -frc.robot.Constants.DriveConstants.kmDriveWidth.in(Inches) / 2,
        frc.robot.Constants.DriveConstants.kmDriveLength.in(Inches) / 2);

    static Translation2d m_backRightLocation = new Translation2d(
        -frc.robot.Constants.DriveConstants.kmDriveWidth.in(Inches) / 2,
        -frc.robot.Constants.DriveConstants.kmDriveLength.in(Inches) / 2);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          m_frontLeftLocation,
          m_frontRightLocation,
          m_backLeftLocation,
          m_backRightLocation);

  }

  public static final class AutoConstants {// copied from last year's code, not sure how this works, delete if needed
    public static final double kSwerveDiscreteTimestep = 0.02;

  }

}

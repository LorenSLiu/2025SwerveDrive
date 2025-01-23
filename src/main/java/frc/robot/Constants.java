// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  
  public static class ModuleConstants{
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 6000 / 60;//what the hell is this
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = 3.56;//need to change it, talk with Stanford

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.001;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / 2;//kDriveWheelFreeSpeedRps;
    // public static final double kDrivingMinOutput = -1;
    // public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
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
    //CAN bus ID
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

    //Module offsets
    public static final double kFrontLeftAbsoluteEncoderOffsetRadians = 0;
    public static final double kFrontRightAbsoluteEncoderOffsetRadians = 0;
    public static final double kBackLeftAbsoluteEncoderOffsetRadians = 0;
    public static final double kBackRightAbsoluteEncoderOffsetRadians = 0;

    //Moudle reversed or nah
    public static final boolean kFrontLeftTurningEncoderReversed  = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed   = true;
    public static final boolean kBackRightTurningEncoderReversed  = true;


    //frame size
    public static final double kmDriveWidth = Units.inchesToMeters(26.5);
    public static final double kmDriveLength = Units.inchesToMeters(26.5);

  }
}

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

    //frame size
    public static final double kmDriveWidth = Units.inchesToMeters(26.5);
    public static final double kmDriveLength = Units.inchesToMeters(26.5);

  }
}

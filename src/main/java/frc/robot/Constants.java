// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final boolean ROTATION_ENCODER_DIRECTION = false; 

    /* * * MEASUREMENTS * * */
    public static final double WHEEL_DIAMETER = 4 * 2.5 / 100;
    public static final double TRACK_WIDTH = 0.635;
    public static final double WHEEL_BASE = 0.635;
    
    public static final double DRIVE_GEAR_RATIO = 8.14 / 1;
    public static final double ROTATION_GEAR_RATIO = 150 / 7;
    
    public static final double VOLTAGE = 7.2;

    /* * * SWERVE DRIVE KINEMATICS * * */
    // ORDER IS ALWAYS FL, BL, FR, BR 
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      // front left
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back left
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // front right
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      // back right
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    /* * * FRONT LEFT * * */
    public static final int FL_DRIVE_PORT = 1;
    public static final int FL_ROTATION_PORT = 5;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 9;
    public static final double FL_OFFSET = Math.toDegrees(1.78 - 0.05); //1.43 + 2;

    /* * * BACK LEFT * * */
    public static final int BL_DRIVE_PORT = 2;
    public static final int BL_ROTATION_PORT = 6;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 10;
    public static final double BL_OFFSET = Math.toDegrees(-1.4 - 0.03) + 180; //-1.73 + 1;

    /* * * BACK RIGHT * * */
    public static final int BR_DRIVE_PORT = 3;
    public static final int BR_ROTATION_PORT = 7;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 11;
    public static final double BR_OFFSET = Math.toDegrees(-2.67) + 180;//-0.4 - 55;

    /* * * FRONT RIGHT * * */
    public static final int FR_DRIVE_PORT = 4;
    public static final int FR_ROTATION_PORT = 8;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 12;
    public static final double FR_OFFSET = Math.toDegrees(-2.61);//-0.4 - 90 + 23;
    
    /* * * CONVERSIONS FOR ENCODERS * * */
    //velocity in meters per sec instead of RPM 
    public static final double DRIVE_ENCODER_POSITION_CONVERSION = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER; //drive enc rotation
    //velocity in meters instead of rotations 
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed 
 
    //public static final double ROTATION_ENCODER_POSITION_CONVERSION = ROTATION_GEAR_RATIO * Math.PI; //rotation enc rotation 
    //public static final double ROTATION_ENCODER_VELOCITY_CONVERSION = ROTATION_ENCODER_POSITION_CONVERSION / 60; //rotation enc speed

    /* * * PID VALUES * * */
    public static final double KP_TURNING = 0.0048;//0.008;
    public static final double KI_TURNING = 0.0002;
    public static final double KD_TURNING = 0.0001;//1;

    /* * * MAX * * */
    public static final double MAX_SPEED = 3.6576;
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    
  }
}

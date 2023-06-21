// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class SwerveConstants {
    public static final boolean ROTATION_ENCODER_DIRECTION = false; 
    /* * * MEASUREMENTS * * */
    public static final double WHEEL_DIAMETER = 4 * 2.5 / 100;
    public static final double TRACK_WIDTH = 0.635;
    public static final double WHEEL_BASE = 0.635;
    
    public static final double GEAR_RATIO = 8.14 / 1;
    public static final double STEER_GEAR_RATIO = 150 / 7;
    
    public static final double VOLTAGE = 7.2;
    
    /* * * CONVERSIONS FOR ENCODERS * * */
    public static final double DRIVE_ENCODER_POSITION_CONVERSION = GEAR_RATIO * Math.PI * WHEEL_DIAMETER; //drive enc rotation
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed 
 
    public static final double ROTATION_ENCODER_POSITION_CONVERSION = STEER_GEAR_RATIO * Math.PI; //rotation enc rotation 
    public static final double ROTATION_ENCODER_VELOCITY_CONVERSION = ROTATION_ENCODER_POSITION_CONVERSION / 60; //rotation enc speed

    /* * * PID VALUES * * */
    public static final double KP_TURNING = 0.5;
    public static final double KI_TURNING = 0.0;
    public static final double KD_TURNING = 0.0025;

    /* * * MAX * * */
    public static final double MAX_SPEED = 3.6576;
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    
  }
}

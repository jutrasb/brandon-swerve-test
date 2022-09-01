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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.2794;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 99; // Not used, hopefully.
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0;//-Math.toRadians(238.97);

    // public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    // public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    // public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 4;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(140.62);

    // public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    // public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
    // public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(325.63);

    // public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    // public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
    // public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 1;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(266.48);
}
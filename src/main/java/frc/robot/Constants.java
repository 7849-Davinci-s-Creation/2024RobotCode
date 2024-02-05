// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int driverControllerPort = 0;
  }

  public static class MotorConstants {
    public static final int LeftFrontMotor = 1;
    public static final int LeftBackMotor = 2;
    public static final int RightFrontMotor = 4;
    public static final int RightBackMotor = 3;
  }

  public static class DriveTrainConstants {
    // might need to be changed depending on field and robot weight.
    public static final double TORQUE_RESITANCE_THRESHOLD = 0.05;
    public static final double JOYSTICK_DEADZONE_DRIVE = 0.0055;
    public static final double JOYSTICK_DEADZONE_ROTATE = 0.0055;

    public static final double NORMAL_DRIVE_NERF = 0.5;
    public static final double NORMAL_ROTATE_NERF = 0.5;

    public static final double CREEP_DRIVE_NERF = 0.25;
    public static final double CREEP_ROTATE_NERF = 0.25;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class IntakeConstants {
    public static final double OUT_SECONDS = 2;
    public static final double INTAKE_GENERAL_PERCENT_OUTPUT = 0.8;
    public static final double NOTE_INTAKE_VOLTAGE_THRESHOLD = 10;
    public static final int BEAM_BREAKER_PORT = 0;
  }

  public static class MotorConstants {
    public static final int LEFT_FRONT_MOTOR = 1;
    public static final int LEFT_BACK_MOTOR = 2;
    public static final int RIGHT_FRONT_MOTOR = 4;
    public static final int RIGHT_BACK_MOTOR = 3;
    public static final int INTAKE_MOTOR = 6;
    public static final int FLY_WHEEL_BOTTOM = 7;
    public static final int FLY_WHEEL_TOP = 5;
  }

  public static class SysIDValues {
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double KP = 0;
  }

  public static class DriveTrainConstants {
    public static final double TORQUE_RESISTANCE_THRESHOLD = 0.05;
    public static final double JOYSTICK_DEADZONE_DRIVE = 0.0055;
    public static final double JOYSTICK_DEADZONE_ROTATE = 0.0055;

    public static final double NORMAL_DRIVE_NERF = 0.5;
    public static final double NORMAL_ROTATE_NERF = 0.5;

    public static final double CREEP_DRIVE_NERF = 0.25;
    public static final double CREEP_ROTATE_NERF = 0.25;

    public static final double TRACK_WIDTH_METERS = 0.584;
    public static final double MAX_SPEED_MPS = 1;
    public static final double MAX_ACCELERATION_MPS_SQ = 1;
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_Z = 0.7;
    public static final double GEAR_RATIO = 8.46; 
    public static final double WHEEL_RADIUS_INCHES = 3;
    public static final double LINEAR_CONVERSION_FACTOR = Units.inchesToMeters(
            1/(GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS_INCHES)) * 10);
  }

  public static final class VisionConstants {
    // TODO: change these for field and robot elements
    public static final double CAMERA_HEIGHT_METERS = 0;
    public static final double TARGET_HEIGHT_METERS = 0;
    public static final double CAMERA_PITCH_RADIANS = 0;
    public static final double GOAL_RANGE_METERS = 0;
    public static final double AIM_LINEAR_P = 0;
    public static final double AIM_LINEAR_D = 0;
    public static final double AIM_ANGULAR_P = 0;
    public static final double AIM_ANGULAR_D = 0;
    public static final String CAMERA_NAME = "";
  }

  public static class ShooterConstants {
    public static final double REV_TIME = 1.5;
    public static final double OPTIMAL_AMP_RPM = 1;
    public static final double P = 0.000314156269;
    public static final double I = 0.001;
    public static final double D = 0.000031;
    public static final double OPTIMAL_SPEAKER_RPM = 6200;
  }
}

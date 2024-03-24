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
    public static final double INTAKE_GENERAL_PERCENT_OUTPUT = 1;
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

  public static class DriveTrainConstants {
    public static final double TORQUE_RESISTANCE_THRESHOLD = 0.05;
    public static final double JOYSTICK_DEADZONE_DRIVE = 0.0055;
    public static final double JOYSTICK_DEADZONE_ROTATE = 0.0055;
    public static final double NORMAL_DRIVE_NERF = 0.5;
    public static final double NORMAL_ROTATE_NERF = 0.5;
    public static final double CREEP_DRIVE_NERF = 0.25;
    public static final double CREEP_ROTATE_NERF = 0.25;
    public static final double TRACK_WIDTH_METERS = 0.584;
    public static final double GEAR_RATIO = 8.46; 
    public static final double WHEEL_RADIUS_INCHES = 3;
    public static final double LINEAR_CONVERSION_FACTOR = Units.inchesToMeters(
            1/(GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS_INCHES)) * 10);
  }

  public static class ShooterConstants {
    public static final double REV_TIME = 1.5;
    public static final double P = 0.000314156269;
    public static final double I = 0.001;
    public static final double D = 0.000031;
    public static final double OPTIMAL_SPEAKER_RPM = 6200;
  }
}

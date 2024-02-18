// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class IntakeConstants {
    public static final int INTAKE_SWITCH = 0;
  }

  public static class MotorConstants {
    public static final int LEFT_FRONT_MOTOR = 1;
    public static final int LEFT_BACK_MOTOR = 2;
    public static final int RIGHT_FRONT_MOTOR = 4;
    public static final int RIGHT_BACK_MOTOR = 3;
    //TODO: change these when they are wired into CAN
    public static final int INTAKE_MOTOR = 0;
    public static final int FLY_WHEEL = 0;
    public static final int INTAKE_WHEEL = 0;
  }

  public static class DriveTrainConstants {
    //TODO: might need to be changed depending on field and robot weight.
    public static final double TORQUE_RESISTANCE_THRESHOLD = 0.05;
    public static final double JOYSTICK_DEADZONE_DRIVE = 0.0055;
    public static final double JOYSTICK_DEADZONE_ROTATE = 0.0055;

    public static final double NORMAL_DRIVE_NERF = 0.5;
    public static final double NORMAL_ROTATE_NERF = 0.5;

    public static final double CREEP_DRIVE_NERF = 0.25;
    public static final double CREEP_ROTATE_NERF = 0.25;
  }
}

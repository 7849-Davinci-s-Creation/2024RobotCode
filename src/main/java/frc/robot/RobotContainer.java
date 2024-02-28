// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BuiltCommands;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MurderShooter;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import lib.DashboardManager;

public class RobotContainer {
  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  // controllers
  private final CommandPS4Controller driverController = new CommandPS4Controller(
      Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
      Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // add subsystems to dashboard managers list (ex:
    // Dashboard.addSubsystemDashboard(subsystem); )
    DashboardManager.addSubsystemDashboard(driveTrain);
    DashboardManager.addSubsystemDashboard(intake);

    // Configure Subsystems dashboard configurations
    DashboardManager.configureDashboards();
  }

  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(new Drive(driverController.getHID(), driveTrain));
    operatorController.a().whileTrue(BuiltCommands.shootSequence(shooter, intake)).onFalse(new MurderShooter(shooter));
    intake.setDefaultCommand(new IntakeCommand(operatorController.getHID(), intake));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

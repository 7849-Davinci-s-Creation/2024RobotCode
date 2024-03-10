// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoTurnaround;
import frc.robot.commands.Autos;
import frc.robot.commands.BuiltCommands;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.shooter.MurderShooter;
import frc.robot.commands.timedcommands.RunIntakeSeconds;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import lib.DashboardManager;

public class RobotContainer {
  private final SendableChooser<Command> autoMenu = Autos.getAutoMenu();

  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  private static boolean debugMode = false;

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

    configureAutoMenu();

    SmartDashboard.putData(autoMenu);

    // We can set debug mode like so
    setDebugMode(true);
  }

  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(new Drive(driverController.getHID(), driveTrain));
    driverController.circle().onTrue(new AutoTurnaround(driveTrain));
    operatorController.a()
        .whileTrue(BuiltCommands.shootSequence(shooter, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM))
        .onFalse(new MurderShooter(shooter));
    operatorController.x()
        .whileTrue(BuiltCommands.shootSequence(shooter, intake, Constants.ShooterConstants.OPTIMAL_AMP_RPM))
        .onFalse(new MurderShooter(shooter));
    operatorController.b().whileTrue(new IntakeCommand(intake))
        .onFalse(new RunIntakeSeconds(intake, 0.5, -Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT));
  }

  private void configureAutoMenu() {
    if (debugMode) {
      autoMenu.addOption("Quasistatic Forward", Autos.sysIDQuasistatic(driveTrain, Direction.kForward));
      autoMenu.addOption("Quasistatic Reverse", Autos.sysIDQuasistatic(driveTrain, Direction.kReverse));
      autoMenu.addOption("Dynamic Forward", Autos.sysIDDynamic(driveTrain, Direction.kForward));
      autoMenu.addOption("Dynamic Reverse", Autos.sysIDDynamic(driveTrain, Direction.kReverse));
    }
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return Autos.getAutoMenu().getSelected();
  }

  private static void setDebugMode(boolean mode) {
    debugMode = mode;
  }

  public static Boolean isDebugMode() {
    return debugMode;
  }

  public void robotInit() {
  }

  public void robotPeriodic() {
  }


  public void disabledInit() {
  }

  public void disabledPeriodic() {
  }

  public void autonomousInit() {
  }

  public void autonomousPeriodic() {
  }

  public void teleopInit() {
  }

  public void teleopPeriodic() {
  }

  public void testInit() {
  }

  public void testPeriodic() {
  }

  public void simulationInit() {
  }

  public void simulationPeriodic() {
  }
}

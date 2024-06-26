// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BuiltCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.drivetrain.AutoTurnaround;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.intake.EatNote;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.MurderShooter;
import frc.robot.commands.timedcommands.RunIntakeSeconds;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import lib.DashboardManager;

public class RobotContainer {
  // controllers
  public static final CommandPS4Controller driverController = new CommandPS4Controller(
      Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  public static final CommandXboxController operatorController = new CommandXboxController(
      Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private static boolean debugMode = false;

  private final SendableChooser<Command> autoMenu = Autos.getAutoMenu();

  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  public RobotContainer() {
    // set debug mode true / false
    setDebugMode(false);

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
  }

  private void configureButtonBindings() {
    driveTrain.setDefaultCommand(new Drive(driverController.getHID(), driveTrain));

    driverController.circle().onTrue(new AutoTurnaround(driveTrain));

    // Shoot Speaker
    operatorController.a()
        .whileTrue(BuiltCommands.shootSequence(shooter, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM))
        .onFalse(new MurderShooter(shooter));

    // eat note (feeder station feed)
    operatorController.x()
        .whileTrue(new EatNote(intake, shooter))
        .onFalse(new RunIntakeSeconds(intake, 0.1,
            -Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT));

    // Manuele intake
    operatorController.b().whileTrue(new IntakeCommand(intake))
        .onFalse(new RunIntakeSeconds(intake, 0.1,
            -Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT));

    // debug reset encoders and compass heading
    operatorController.back().onTrue(
        new InstantCommand(
            () -> {
              driveTrain.zeroHeading();
              driveTrain.resetEncoders();
            }));
  }

  private void configureAutoMenu() {
    autoMenu.addOption("LazyBot (do nothing..)", Autos.lazyBot());
    autoMenu.addOption("Shoot Auto", Autos.shootAuto(shooter, intake, driveTrain));
    autoMenu.addOption("Red Center Auto", Autos.redCenterAuto(driveTrain, intake, shooter));
    autoMenu.addOption("Blue Center Auto", Autos.blueCenterAuto(driveTrain, intake, shooter));
    autoMenu.addOption("Red Amp Side 2Note Auto", Autos.redAmpside2note(driveTrain, intake, shooter));
    autoMenu.addOption("Blue Amp Side 2Note Auto", Autos.blueAmpside2note(driveTrain, intake, shooter));
    autoMenu.addOption("Blue Amp Side 2Note No Drive Off Auto", Autos.blueAmpside2noteNoDriveOff(driveTrain, intake, shooter));
    autoMenu.addOption("Red Amp Side 2Note No Drive Off Auto", Autos.redAmpside2noteNoDriveOff(driveTrain, intake, shooter));
  }

  public Command getAutonomousCommand() {
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
    Autos.periodic();
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

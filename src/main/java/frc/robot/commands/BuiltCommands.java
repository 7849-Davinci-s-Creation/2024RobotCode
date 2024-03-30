package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.AutoMurderShooter;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.timedcommands.RevUpShooterSeconds;
import frc.robot.commands.timedcommands.RunIntakeSeconds;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BuiltCommands {
    public static Command shootSequence(Shooter shoot, Intake intake, double rpm) {
        return new SequentialCommandGroup(
                new RevUpShooterSeconds(shoot, Constants.ShooterConstants.REV_TIME, rpm),
                new ParallelCommandGroup(
                        new RunIntakeSeconds(intake, Constants.IntakeConstants.OUT_SECONDS,
                                Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT),
                        new ShootCommand(shoot, rpm))

        );
    }

    public static Command autoshootSequence(Shooter shoot, Intake intake, double rpm) {
        return new SequentialCommandGroup(
                new RevUpShooterSeconds(shoot, Constants.ShooterConstants.REV_TIME, rpm),
                new ParallelCommandGroup(
                        new RunIntakeSeconds(intake, Constants.IntakeConstants.OUT_SECONDS,
                                Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT),
                        new RevUpShooterSeconds(shoot, 1, rpm),
                        new WaitCommand(1)),
                new AutoMurderShooter(shoot));
    }

    public static Command autonomousIntake(Intake intake) {
        return new SequentialCommandGroup(
                new IntakeCommand(intake),
                new RunIntakeSeconds(intake, 0.2, -Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT));
    }
}

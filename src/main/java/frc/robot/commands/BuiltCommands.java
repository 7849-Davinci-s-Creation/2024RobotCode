package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeUntilNoteCommand;
import frc.robot.commands.shooter.RunBottomFlywheel;
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
                        new ShootCommand(shoot, rpm, intake))

        );
    }

    public static Command intakeSequence(Intake intake, Shooter shooter, CommandXboxController controller) {
        if (controller == null) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new RunIntakeSeconds(intake, 0.5, Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT),
                            new IntakeUntilNoteCommand(intake),
                            new RunBottomFlywheel(shooter)
                            ),
                    new RunIntakeSeconds(intake, 0.5, -Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT));
        }

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                                    new RunIntakeSeconds(intake, 2, Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT),
                                    new IntakeUntilNoteCommand(intake),
                                    new RunBottomFlywheel(shooter)
                ),
                new RunIntakeSeconds(intake, 0.5, -Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT),
                new RumbleControllerCommand(controller, 0.5));
    }
}

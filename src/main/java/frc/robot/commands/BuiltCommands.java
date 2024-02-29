package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.timedcommands.RevUpShooterSeconds;
import frc.robot.commands.timedcommands.RunIntakeSeconds;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BuiltCommands {
    public static Command shootSequence(Shooter shoot, Intake intake) {
        return new SequentialCommandGroup(
                new RevUpShooterSeconds(shoot, 1, 1),
                new ParallelCommandGroup(
                        new RunIntakeSeconds(intake, 2, 1),
                        new ShootCommand(shoot))
        );
    }
    
}
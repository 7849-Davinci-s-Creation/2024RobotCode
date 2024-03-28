package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.MoveMeters;
import frc.robot.commands.drivetrain.MoveMetersBackwards;
import frc.robot.commands.drivetrain.RotateAngel;
import frc.robot.commands.drivetrain.RotateNegativeAngle;
import frc.robot.commands.shooter.MurderShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Autos {
    private static SendableChooser<Command> autoMenu = null;

    private Autos() {
    }

    static {
        SmartDashboard.putNumber("P", 0);
        SmartDashboard.putNumber("I", 0);
        SmartDashboard.putNumber("D", 0);
    }

    public static SendableChooser<Command> getAutoMenu() {
        if (autoMenu == null) {
            autoMenu = new SendableChooser<>();
        }

        return autoMenu;
    }

    public static void periodic() {
    }

    public static Command lazyBot() {
        return null;
    }

    public static Command shootAuto(Shooter shoot, Intake intake) {
        return BuiltCommands.autoshootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM);
    }

    public static Command redAuto(DriveTrain driveTrain, Intake intake, Shooter shoot) {
        // shoot note
        // move back 0.46285155415535 meters
        // turn to 21 degrees
        // move 0.561 meters back while intaking
        return new SequentialCommandGroup(
                BuiltCommands.autoshootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM),
                new MoveMeters(driveTrain, 0.46285155415535, 1, 0, 0),
                new WaitCommand(0.2),
                new RotateAngel(driveTrain, 15),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new MoveMeters(driveTrain, 1, 1, 0, 0),
                        BuiltCommands.autonomousIntake(intake)),
                new WaitCommand(0.2),
                new MoveMetersBackwards(driveTrain, 1, 1, 0, 0),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new MoveMetersBackwards(driveTrain, 0.1, 1, 0, 0),
                        BuiltCommands.autoshootSequence(shoot, intake,
                                Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM))
                );
    }
}

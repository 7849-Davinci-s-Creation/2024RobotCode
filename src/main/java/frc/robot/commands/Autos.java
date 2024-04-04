package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.MoveMeters;
import frc.robot.commands.drivetrain.MoveMetersBackwards;
import frc.robot.commands.drivetrain.RotateAngel;
import frc.robot.commands.drivetrain.RotateAngelCustom;
import frc.robot.commands.drivetrain.RotateNegativeAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Autos {
    private static SendableChooser<Command> autoMenu = null;

    private Autos() {
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

    public static Command redCenterAuto(DriveTrain driveTrain, Intake intake, Shooter shoot) {
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
                                Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM)));
    }

    public static Command blueCenterAuto(DriveTrain driveTrain, Intake intake, Shooter shoot) {
        // shoot note
        // move back 0.46285155415535 meters
        // turn to 21 degrees
        // move 0.561 meters back while intaking
        return new SequentialCommandGroup(
                BuiltCommands.autoshootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM),
                new MoveMeters(driveTrain, 0.66285155415535, 1, 0, 0),
                new WaitCommand(0.2),
                new RotateAngel(driveTrain, 22),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new MoveMeters(driveTrain, 1, 1, 0, 0),
                        BuiltCommands.autonomousIntake(intake)),
                new WaitCommand(0.2),
                new MoveMetersBackwards(driveTrain, 1, 1, 0, 0),
                new WaitCommand(0.2),
                new RotateNegativeAngle(driveTrain, -18),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                        new MoveMetersBackwards(driveTrain, 0.2, 1, 0, 0),
                        BuiltCommands.autoshootSequence(shoot, intake,
                                Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM)));
    }

    public static Command redAmpside2note(DriveTrain driveTrain, Intake intake, Shooter shoot) {
        return new SequentialCommandGroup(
                BuiltCommands.autoshootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM),
                new WaitCommand(0.2),
                //new MoveMeters(driveTrain, 1, 1, 0, 0),
                //new WaitCommand(0.2),
               // new RotateAngel(driveTrain, 9),
               // new WaitCommand(0.2),
                new ParallelCommandGroup(
                    new MoveMeters(driveTrain, 1.85, 1, 0, 0),
                    BuiltCommands.autonomousIntake(intake)),
                new WaitCommand(0.2), 
                new MoveMetersBackwards(driveTrain, 1.35, 1, 0, 0),
                new WaitCommand(0.2), 
                BuiltCommands.autoshootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM),
                new WaitCommand(0.2), 
                new RotateAngelCustom(driveTrain, 9,0.05,0,0), 
                new WaitCommand(0.2),
                new MoveMeters(driveTrain,5, 1, 0, 0)
                );
            
    }    

    public static Command blueAmpside2note(DriveTrain driveTrain, Intake intake, Shooter shoot) {
        return new SequentialCommandGroup(
                BuiltCommands.autoshootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM),
                new WaitCommand(0.2),
                new MoveMeters(driveTrain, 0.7, 1, 0, 0),
                new WaitCommand(0.2),
                new RotateNegativeAngle(driveTrain, 25),
                new WaitCommand(0.2),
                new ParallelCommandGroup(
                    new MoveMeters(driveTrain, 2.1, 1, 0, 0),
                    BuiltCommands.autonomousIntake(intake),
                    new WaitCommand(2)
                    ),
                new WaitCommand(0.2),
                new MoveMetersBackwards(driveTrain, 1, 1, 0, 0),
                new WaitCommand(0.2), 
                new RotateAngel(driveTrain, 43),
                new WaitCommand(0.4), 
                new MoveMetersBackwards(driveTrain, 0.87, 1, 0, 0),
                new WaitCommand(0.2), 
                BuiltCommands.autoshootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM),
                new MoveMeters(driveTrain, 1, 1, 0, 0),
                new RotateNegativeAngle(driveTrain, 19.5),
                new WaitCommand(0.2),
                new MoveMeters(driveTrain,5, 1, 0, 0)
                );
            
    }    

    
}

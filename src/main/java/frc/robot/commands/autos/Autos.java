package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.commands.BuiltCommands;
import frc.robot.commands.MoveMeters;
import frc.robot.commands.RotateAngel;
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

    public static Command lazyBot() {
        return null;
    }

    public static Command shootAuto(Shooter shoot, Intake intake) {
        return BuiltCommands.shootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM);
    }

    public static Command noteAutoRedCenter(DriveTrain driveTrain, Intake intake, Shooter shoot) {
        return new SequentialCommandGroup(
            BuiltCommands.shootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM),
            new WaitCommand(0.5),
            new MoveMeters(driveTrain,.273),
            new WaitCommand(0.5),
           new RotateAngel(driveTrain, 90),
           new WaitCommand(0.5),
           new ParallelCommandGroup(
            new MoveMeters(driveTrain,1.0421),
            BuiltCommands.autonomousIntake(intake)
           ),
           new WaitCommand(0.5)
           //new MoveMeters(driveTrain,-1.0421)
        );
    }
//0.273
//14.81
//1.0421
//-1.0421
}

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

    public static Command sysIDQuasistatic(DriveTrain driveTrain, Direction direction) {
        return driveTrain.getRoutine().quasistatic(direction);
    }

    public static Command sysIDDynamic(DriveTrain driveTrain, Direction direction) {
        return driveTrain.getRoutine().dynamic(direction);
    }

    public static Command testRamseteExampleTrajectory(DriveTrain driveTrain) {
        return RamseteUtil.testRamseteTrajectory(driveTrain);
    }

    public static Command testRamsetePathWeaverPath(DriveTrain driveTrain, String pathFile) {
        return RamseteUtil.loadPathWeaverTrajectoryToRamseteCommand(pathFile, true, driveTrain);
    }

    public static Command lazyBot() {
        return null;
    }

    public static Command shootAuto(Shooter shoot, Intake intake) {
        return BuiltCommands.shootSequence(shoot, intake, Constants.ShooterConstants.OPTIMAL_SPEAKER_RPM);
    }

    public static Command simpleMove(DriveTrain driveTrain) {
        return new MoveMeters(driveTrain, 3);
    }

    public static Command simpleRotate(DriveTrain driveTrain) {
        return new RotateAngel(driveTrain, 90);
    }

    public static Command simpleMovenRotateCommand(DriveTrain driveTrain) {
        return new SequentialCommandGroup(
                new MoveMeters(driveTrain, 3),
                new WaitCommand(0.5),
                new RotateAngel(driveTrain, 90));
    }
    public static Command simpleMoveRotateCommand(DriveTrain driveTrain) {
        return new SequentialCommandGroup(
                new RotateAngel(driveTrain, 260),
                new WaitCommand(0.5),
                new MoveMeters(driveTrain, -3));
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

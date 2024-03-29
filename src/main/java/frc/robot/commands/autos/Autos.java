package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.DriveTrain;

public class Autos {
    private static SendableChooser<Command> autoMenu = null;

    private Autos(){}

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
}

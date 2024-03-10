package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RamseteUtil {
    private static RamseteCommand generateRamseteCommand(Trajectory trajectory, DriveTrain driveTrain) {
        return new RamseteCommand(
                trajectory,
                driveTrain::getPose2d,
                new RamseteController(
                        Constants.DriveTrainConstants.RAMSETE_B,
                        Constants.DriveTrainConstants.RAMSETE_Z
                ),
                new SimpleMotorFeedforward(
                        Constants.SysIDValues.KS,
                        Constants.SysIDValues.KV,
                        Constants.SysIDValues.KA
                ),
                driveTrain.getKinematics(),
                driveTrain::getWheelSpeeds,
                new PIDController(
                        Constants.SysIDValues.KP,0,0
                ),
                new PIDController(
                        Constants.SysIDValues.KP, 0, 0
                ),
                driveTrain::voltageTankDrive
        );
    }

    public static Command loadPathWeaverTrajectoryToRamseteCommand(String file, boolean resetOdometry, DriveTrain driveTrain) {
        final Trajectory trajectory;

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(file);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + file, e.getStackTrace());
            System.out.println("Unable to read from file: " + file);
            return new InstantCommand();
        }

        RamseteCommand  ramseteCommand = generateRamseteCommand(trajectory, driveTrain);

        if (resetOdometry) {
            return new SequentialCommandGroup(
                    new InstantCommand(
                            () -> driveTrain.resetOdometry(trajectory.getInitialPose())
                    ),
                    ramseteCommand
            );
        } else {
            return ramseteCommand;
        }
    }

    // https://youtu.be/yVmJDOE3M2Y what path should look like.
    public static Command testRamseteTrajectory(DriveTrain driveTrain) {
        DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.SysIDValues.KS,
                        Constants.SysIDValues.KV,
                        Constants.SysIDValues.KA
                ),
                driveTrain.getKinematics(),
                5
        );

        TrajectoryConfig config = new TrajectoryConfig(
                Constants.DriveTrainConstants.MAX_SPEED_MPS,
                Constants.DriveTrainConstants.MAX_ACCELERATION_MPS_SQ
        ).setKinematics(driveTrain.getKinematics())
                .addConstraint(voltageConstraint);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        // Pass config
                        config);

        RamseteCommand  ramseteCommand = generateRamseteCommand(exampleTrajectory, driveTrain);

        return Commands.runOnce(
                () -> driveTrain.resetOdometry(exampleTrajectory.getInitialPose()))
                .andThen(ramseteCommand)
                .andThen(Commands.runOnce(() -> driveTrain.voltageTankDrive(0,0)));
    }

}

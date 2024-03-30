package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class RotateAngel extends Command {
    private final DriveTrain drive;
    private final PIDController pid;
    private final double angle;

    public RotateAngel(DriveTrain drive, double angle) {
        this.drive = drive;
        this.angle = angle;
        pid = new PIDController(Constants.DriveTrainConstants.ROTATE_P, Constants.DriveTrainConstants.ROTATE_I,
                Constants.DriveTrainConstants.ROTATE_D);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.zeroHeading();
    }

    @Override
    public void execute() {
        drive.arcadeDrive((pid.calculate(drive.getHeading(), angle)), 0);
    }

    @Override
    public void end(boolean interuppted) {
        drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return drive.getHeading() >= angle;
    }

}

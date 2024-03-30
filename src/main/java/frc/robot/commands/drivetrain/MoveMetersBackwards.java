package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class MoveMetersBackwards extends Command {
    private final DriveTrain drive;
    private final PIDController forward;
    private final PIDController turn = new PIDController(Constants.DriveTrainConstants.TURN_P, 0, 0);
    private final double targetMeters;

    public MoveMetersBackwards(DriveTrain drive, double targetMeters, double P, double I, double D) {
        this.drive = drive;
        this.targetMeters = targetMeters;
        this.forward = new PIDController(P, I, D);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
        drive.zeroHeading();
    }

    @Override
    public void execute() {
        double goForward = forward.calculate(drive.getAverageEncoderDistance(), targetMeters);
        double goRotate = turn.calculate(drive.getHeading(), 0);

        drive.arcadeDrive(goRotate, -goForward);
    }

    @Override
    public void end(boolean interuppted) {
        drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getLeftEncoderPosition()) >= targetMeters;
    }
}

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class MoveMeters extends Command {
    DriveTrain drive;
    PIDController forward = new PIDController(0, 0, 0);
    PIDController turn = new PIDController(0, 0, 0);
    double targetMeters;
    



    public MoveMeters(DriveTrain drive,double targetMeters) {
        this.drive = drive;
        this.targetMeters = targetMeters;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();
        drive.zeroHeading();
    }

    @Override
    public void execute() {
        double goForward = forward.calculate(drive.getAverageEncoderDistance(),targetMeters);
        double goRotate = turn.calculate(drive.getHeading(),0);

        drive.arcadeDrive(goRotate,goForward);
    }

    @Override
    public void end(boolean interuppted) {
        drive.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return drive.getAverageEncoderDistance() >= targetMeters;
    }

    
}

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class AutoTurnaround extends Command {
    private final DriveTrain drive;
    private final PIDController pid;

    public AutoTurnaround(DriveTrain drive) {
        this.drive = drive;
        pid = new PIDController(0.0033,0,0);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.zeroHeading();
    }

    @Override
    public void execute() {
      drive.arcadeDrive((pid.calculate(drive.getHeading(),180)),0);
    }

    @Override
    public void end(boolean interuppted) {
        drive.arcadeDrive(0, 0);


    }

    @Override
    public boolean isFinished() {
        return drive.getHeading() >= 180;
    }

}

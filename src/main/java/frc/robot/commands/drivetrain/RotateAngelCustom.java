package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class RotateAngelCustom extends Command {
    private final DriveTrain drive;
    private final PIDController pid;
    private final double angle;

    public RotateAngelCustom(DriveTrain drive, double angle, double P,double I,double D) {
        this.drive = drive;
        this.angle = angle;
        pid = new PIDController(P, I,D);
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

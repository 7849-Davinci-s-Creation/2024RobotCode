package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.intake(1);
    }

    @Override
    public void end(boolean interuppted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

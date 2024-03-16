package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeUntilNoteCommand extends Command {
    private final Intake intake;

    public IntakeUntilNoteCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.intake(Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT);
    }

    @Override
    public void end(boolean interuppted) {
        intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        return intake.getCurrent() >= Constants.IntakeConstants.NOTE_INTAKE_VOLTAGE_THRESHOLD;
    }
}

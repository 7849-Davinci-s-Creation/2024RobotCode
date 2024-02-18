package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final XboxController xboxController;
    private final Intake intake;

    public IntakeCommand(XboxController xboxController, Intake intake) {
        this.xboxController = xboxController;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (xboxController.getBButton() && !intake.hasNote()) {
            intake.intake();
        } else if (xboxController.getYButton() && intake.hasNote()) {
            intake.outake();
        }

    }

    @Override
    public void end(boolean interuppted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

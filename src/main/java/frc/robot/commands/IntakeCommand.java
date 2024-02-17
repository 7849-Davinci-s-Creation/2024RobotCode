package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final CommandXboxController xboxController;
    private final Intake intake;

    public IntakeCommand(CommandXboxController xboxController, Intake intake) {
        this.xboxController = xboxController;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() { 
        if (xboxController.getHID().getBButton() && !intake.hasNote()) {
            intake.intake();
        } else if (xboxController.getHID().getYButton() && intake.hasNote()) {
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

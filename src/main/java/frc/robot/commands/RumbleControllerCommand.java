package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RumbleControllerCommand extends Command {
    private final CommandXboxController controller;

    public RumbleControllerCommand(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

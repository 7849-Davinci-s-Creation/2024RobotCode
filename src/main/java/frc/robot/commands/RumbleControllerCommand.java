package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RumbleControllerCommand extends Command {
    private final CommandXboxController controller;
    private final double rumble;

    public RumbleControllerCommand(CommandXboxController controller, double rumble) {
        this.controller = controller;
        this.rumble = rumble;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, rumble);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

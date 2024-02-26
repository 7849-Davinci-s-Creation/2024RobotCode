package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private final CommandXboxController xboxController;
    private final Shooter shoot;

    public ShootCommand(CommandXboxController xboxController, Shooter shoot) {
        this.xboxController = xboxController;
        this.shoot = shoot;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (xboxController.getHID().getAButton()) {
            shoot.shoot(10);
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

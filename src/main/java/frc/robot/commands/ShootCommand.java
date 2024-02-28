package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {

    private final Shooter shoot;

    public ShootCommand(Shooter shoot) {
        this.shoot = shoot;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shoot.shoot(1);
    }

    @Override
    public void end(boolean interuppted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {

    private final Shooter shooter;
    private final double power;

    public ShootCommand(Shooter shoot, double power) {
        this.shooter = shoot;
        this.power = power;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.shoot(power);
    }

    @Override
    public void end(boolean interuppted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunBottomFlywheel extends Command {

    private final Shooter shooter;

    public RunBottomFlywheel(Shooter shoot) {
        this.shooter = shoot;
        addRequirements(shoot);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.getBottomSparkMax().set(0.5);
    }

    @Override
    public void end(boolean interuppted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

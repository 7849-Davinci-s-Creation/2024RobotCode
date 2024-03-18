package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {

    private final Shooter shooter;
    private final double rpm;
    private final Intake intake;

    public ShootCommand(Shooter shoot, double rpm, Intake intake) {
        this.shooter = shoot;
        this.rpm = rpm;
        this.intake = intake;
        addRequirements(shoot,intake);
    }

    @Override
    public void initialize() {
        intake.setNoteState(false);

    }

    @Override
    public void execute() {
        shooter.shoot(rpm);
    }

    @Override
    public void end(boolean interuppted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

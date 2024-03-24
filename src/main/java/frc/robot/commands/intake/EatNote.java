package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class EatNote extends Command {
    private final Intake intake;
    private final Shooter shooter;

    public EatNote(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.eatNote();
        intake.intake(-0.5);
    }

    @Override
    public void end(boolean interuppted) {
        shooter.murder();
        intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        return intake.getNoteState(); // check if beam breaker reads we have note
    }
}

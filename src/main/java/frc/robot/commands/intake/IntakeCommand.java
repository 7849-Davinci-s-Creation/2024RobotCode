package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
    private final Intake intake;
    private final Shooter shooter;
    private final CommandXboxController controller;

    public IntakeCommand(Intake intake, Shooter shooter, CommandXboxController controller) {
        this.intake = intake;
        this.shooter = shooter;
        this.controller = controller;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.intake(Constants.IntakeConstants.INTAKE_GENERAL_PERCENT_OUTPUT);
        shooter.getBottomSparkMax().set(0.5);
    }

    @Override
    public void end(boolean interuppted) {
        intake.intake(0);
        shooter.getBottomSparkMax().set(0);

        if (controller != null) { // if we specify a controller

            if (isFinished()) { // if we have a note, rumble controller, else stop rumbling it
                controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
            } else {
                controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
            }

        }
    }

    @Override
    public boolean isFinished() {
        return intake.getNoteState(); // check if beam breaker reads we have note
    }

}

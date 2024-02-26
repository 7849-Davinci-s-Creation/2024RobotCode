package frc.robot.commands.timedcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public  class RunIntakeSeconds extends Command {
    private Intake intake;
    private double seconds;
    private double power;

    private double startingtime;

    public RunIntakeSeconds(Intake intake, double seconds, double power) {
       addRequirements(intake);
       this.intake = intake;
        this.seconds = seconds;
       this.power = power;  
    }

    @Override
    public void initialize() {
      startingtime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        intake.intake(power);
    }

    @Override
    public void end(boolean interuppted) {
      intake.intake(0);
    }

    @Override 
    public boolean isFinished() {
      return System.currentTimeMillis() - startingtime > (seconds * 1000);
    }
}

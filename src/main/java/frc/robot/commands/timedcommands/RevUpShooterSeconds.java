package frc.robot.commands.timedcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public  class RevUpShooterSeconds extends Command {
    private Shooter shooter;
    private double seconds;
    private double power;

    private double startingtime;

    public RevUpShooterSeconds(Shooter shooter, double seconds, double power) {
       addRequirements(shooter);
       this.shooter = shooter;
        this.seconds = seconds;
       this.power = power;  
    }

    @Override
    public void initialize() {
      startingtime = System.currentTimeMillis();
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
      return System.currentTimeMillis() - startingtime > (seconds * 1000);
    }
}

package frc.robot.commands.timedcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public  class RevUpShooterSeconds extends Command {
    private final Shooter shooter;
    private final double seconds;
    private final double power;

    private double startingTime;

    public RevUpShooterSeconds(Shooter shooter, double seconds, double power) {
       addRequirements(shooter);
       this.shooter = shooter;
       this.seconds = seconds;
       this.power = power;  
    }

    @Override
    public void initialize() {
      startingTime = System.currentTimeMillis();
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
      return System.currentTimeMillis() - startingTime > (seconds * 1000);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends Command {
    private final DriveTrain driveTrain;
    private final CommandPS4Controller ps4Controller;

    public Drive(DriveTrain driveTrain, CommandPS4Controller ps4Controller) {
        this.driveTrain = driveTrain;
        this.ps4Controller = ps4Controller;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.arcadeDrive(0, 0);
        driveTrain.setBoosted(false);
        driveTrain.setCreeping(false);
        driveTrain.setNormal(true);
    }

    @Override
    public void execute() {
        double drive = driveTrain.handleDeadzone(
                ps4Controller.getLeftY(), Constants.DriveTrainConstants.JOYSTICK_DEADZONE_DRIVE);
        double rotate = driveTrain.handleDeadzone(
                ps4Controller.getRightX(), Constants.DriveTrainConstants.JOYSTICK_DEADZONE_ROTATE);

        if (ps4Controller.getHID().getL1Button() && !(ps4Controller.getHID().getCrossButton())) {
            driveTrain.arcadeDrive(driveTrain.applyCurve(rotate), driveTrain.applyCurve(drive));
            // this is boost mode
            driveTrain.setBoosted(true);
            driveTrain.setCreeping(false);
            driveTrain.setNormal(false);
        } else if (ps4Controller.getHID().getR1Button()&& !(ps4Controller.getHID().getCrossButton())) {
            driveTrain.arcadeDrive(driveTrain.applyCurve(rotate) * 1 / 4, driveTrain.applyCurve(drive) * 1 / 4);
            // this is creep mode
            driveTrain.setBoosted(false);
            driveTrain.setCreeping(true);
            driveTrain.setNormal(false);
        
        } else if (ps4Controller.getHID().getCrossButton()){
            driveTrain.arcadeDrive(0, 0);
       
       
        } else {
            driveTrain.arcadeDrive(driveTrain.applyCurve(rotate) * 1 / 2, driveTrain.applyCurve(drive) * 1 / 2);
            // this is regular drive
            driveTrain.setBoosted(false);
            driveTrain.setCreeping(false);
            driveTrain.setNormal(true);
        }

    }

    @Override
    public void end(boolean interuppted) {
        driveTrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

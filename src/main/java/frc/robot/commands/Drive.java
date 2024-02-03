package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends Command {
    DriveTrain driveTrain;
    CommandPS4Controller ps4Controller;

    public Drive(DriveTrain driveTrain, CommandPS4Controller ps4Controller) {
        this.driveTrain = driveTrain;
        this.ps4Controller = ps4Controller;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.arcadeDrive(0, 0);
    }

    @Override
    public void execute() {
        double drive = driveTrain.handleDeadzone(
                ps4Controller.getLeftY(), Constants.DriveTrainConstants.JOYSTICK_DEADZONE_DRIVE);
        double rotate = driveTrain.handleDeadzone(
                ps4Controller.getRightX(), Constants.DriveTrainConstants.JOYSTICK_DEADZONE_ROTATE);
        driveTrain.arcadeDrive(driveTrain.applyCurve(rotate), driveTrain.applyCurve(drive));
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

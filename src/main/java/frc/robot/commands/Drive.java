package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends Command {
    private final DriveTrain driveTrain;

    private final PS4Controller ps4Controller;

    public Drive(PS4Controller ps4Controller, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.ps4Controller = ps4Controller;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.arcadeDrive(0, 0);

        driveTrain.setNormalDriving();
    }

    @Override
    public void execute() {
        double drive = driveTrain.handleDeadzone(
                ps4Controller.getLeftY(), Constants.DriveTrainConstants.JOYSTICK_DEADZONE_DRIVE);

        double rotate = driveTrain.handleDeadzone(
                ps4Controller.getRightX(), Constants.DriveTrainConstants.JOYSTICK_DEADZONE_ROTATE);

        // if we press either of the triggers then we are wanting to invert robots
        // driving
        driveTrain.setInverted(ps4Controller.getL2Button() || ps4Controller.getR2Button());

        // if boost button is pressed and we arent braking.
        if (ps4Controller.getL1Button() && !(ps4Controller.getCrossButton())) {
            drive(driveTrain.applyCurve(rotate), driveTrain.applyCurve(drive));

            driveTrain.setBoostedDriving();

            // if creep button is pressed and we arent braking.
        } else if (ps4Controller.getR1Button() && !(ps4Controller.getCrossButton())) {
            drive(driveTrain.applyCurve(rotate) * Constants.DriveTrainConstants.CREEP_ROTATE_NERF,
                    driveTrain.applyCurve(drive) * Constants.DriveTrainConstants.CREEP_DRIVE_NERF);

            driveTrain.setCreepedDriving();

            // if brake button is pressed.
        } else if (ps4Controller.getCrossButton()) {
            driveTrain.arcadeDrive(0, 0);

            // else just drive normal
        } else {
            drive(driveTrain.applyCurve(rotate) * Constants.DriveTrainConstants.NORMAL_ROTATE_NERF,
                    driveTrain.applyCurve(drive) * Constants.DriveTrainConstants.NORMAL_DRIVE_NERF);

            driveTrain.setNormalDriving();
        }
    }

    @Override
    public void end(boolean interuppted) {
        driveTrain.setNormalDriving();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // method for handling inversion of driving.
    private void drive(double rotate, double drive) {
        double d = drive;

        // if we are inverting the drive then change drive to opposite
        if (driveTrain.isInverted()) {
            d = -d;
        }

        driveTrain.arcadeDrive(rotate, d);
    }
}

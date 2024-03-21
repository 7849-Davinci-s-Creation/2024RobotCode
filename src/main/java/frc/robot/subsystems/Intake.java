package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import lib.DashboardConfiguration;

public class Intake extends SubsystemBase implements DashboardConfiguration {
    private final TalonSRX intakeMotor = new TalonSRX(Constants.MotorConstants.INTAKE_MOTOR);
    private final DigitalInput hasNote = new DigitalInput(Constants.IntakeConstants.BEAM_BREAKER_PORT);

    public Intake() {
        intakeMotor.setInverted(true);
    }

    public void intake(double current) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, current);
    }

    public double getCurrent() {
        return this.intakeMotor.getSupplyCurrent();
    }

    public boolean getNoteState() {
        return !this.hasNote.get();
    }

    @Override
    public void periodic() {
        this.configureDashboard();
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void configureDashboard() {
        SmartDashboard.putBoolean("Note State", getNoteState());

        if (getNoteState()) { // if we have a note, rumble controller, else stop rumbling it
            RobotContainer.operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else {
            RobotContainer.operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        }
    }
}

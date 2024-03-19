package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.DashboardConfiguration;

public class Intake extends SubsystemBase implements DashboardConfiguration {
    private final TalonSRX intakeMotor = new TalonSRX(Constants.MotorConstants.INTAKE_MOTOR);
    // DigitalInout hasNote = new DigitalInput(Constants.IntakeConstants.BEAM_BREAKER_PORT);
    boolean hasNote = false; // TODO: change this to digital input when beam break is implemented

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
        return this.hasNote;
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
        SmartDashboard.putNumber("Intake Current", intakeMotor.getSupplyCurrent());
        SmartDashboard.putBoolean("Note State", getNoteState());
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.DashboardConfiguration;

public class Intake extends SubsystemBase implements DashboardConfiguration {
    private final TalonSRX intakeMotor = new TalonSRX(Constants.MotorConstants.INTAKE_MOTOR);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.IntakeConstants.INTAKE_SWITCH);

    public Intake() {
        intakeMotor.setInverted(true);
    }

    public boolean hasNote() {
        return this.limitSwitch.get();
    }

    public DigitalInput getLimitSwitch() {
        return this.limitSwitch;
    }

    public void intake(double current) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, current);
    }

    public double getCurrent() {
        return this.intakeMotor.getSupplyCurrent();
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
        SmartDashboard.putBoolean("Has Note", hasNote());
        SmartDashboard.putNumber("Intake Current", intakeMotor.getSupplyCurrent());
    }
}

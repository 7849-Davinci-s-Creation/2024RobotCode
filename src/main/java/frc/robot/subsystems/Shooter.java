package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.DashboardConfiguration;

public class Shooter extends SubsystemBase implements DashboardConfiguration {
    private final TalonSRX shootMotor = new TalonSRX(Constants.MotorConstants.FLY_WHEEL);
    private final TalonSRX intakeMotor = new TalonSRX(Constants.MotorConstants.INTAKE_MOTOR);
    
    public Shooter() {
    }

    public void shoot() {
         shootMotor.set(TalonSRXControlMode.Current, SmartDashboard.getNumber("shooter voltage", 0));
    }

    public void feed() {
        intakeMotor.set(TalonSRXControlMode.Current, 10);
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
        SmartDashboard.putNumber("shooter voltage", 0);
    }
}

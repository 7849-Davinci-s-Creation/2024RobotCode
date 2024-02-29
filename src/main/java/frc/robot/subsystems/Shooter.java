package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.DashboardConfiguration;

public class Shooter extends SubsystemBase implements DashboardConfiguration {
    private final TalonSRX topFlyWheel = new TalonSRX(Constants.MotorConstants.FLY_WHEEL_TOP);
    private final TalonSRX bottomFlyWheel = new TalonSRX(Constants.MotorConstants.FLY_WHEEL_BOTTOM);
    
    public Shooter() {
        bottomFlyWheel.follow(topFlyWheel);
        topFlyWheel.setInverted(true);
        bottomFlyWheel.setInverted(true);
    }

    public void shoot(double current) {
        topFlyWheel.set(ControlMode.PercentOutput, current);
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

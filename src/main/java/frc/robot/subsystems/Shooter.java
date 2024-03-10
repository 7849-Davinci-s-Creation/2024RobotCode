package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.DashboardConfiguration;

public class Shooter extends SubsystemBase implements DashboardConfiguration {
    private final CANSparkMax topFlyWheel = new CANSparkMax(Constants.MotorConstants.FLY_WHEEL_TOP, MotorType.kBrushless);
    private final CANSparkMax bottomFlyWheel = new CANSparkMax(Constants.MotorConstants.FLY_WHEEL_BOTTOM, MotorType.kBrushless);

    private final RelativeEncoder shooterEncoder = topFlyWheel.getEncoder();

    private final PIDController shooterPID = new PIDController(Constants.ShooterConstants.P,
            Constants.ShooterConstants.I, Constants.ShooterConstants.D);

    public Shooter() {
        bottomFlyWheel.follow(topFlyWheel);
    }

    public void murder() {
        topFlyWheel.setVoltage(0);
    }

    public void shoot(double wantedRPM) {
        topFlyWheel.set(shooterPID.calculate(shooterEncoder.getVelocity(),wantedRPM));
    }

    public PIDController getController() {
        return this.shooterPID;
    }

    public RelativeEncoder getShooterEncoder() {
        return this.shooterEncoder;
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
        SmartDashboard.putNumber("shooter RPM", shooterEncoder.getVelocity());
    }
}

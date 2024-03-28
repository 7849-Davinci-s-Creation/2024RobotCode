package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import lib.DashboardConfiguration;

public class Shooter extends SubsystemBase implements DashboardConfiguration {
    private final CANSparkMax topFlyWheel = new CANSparkMax(Constants.MotorConstants.FLY_WHEEL_TOP, MotorType.kBrushless);
    private final CANSparkMax bottomFlyWheel = new CANSparkMax(Constants.MotorConstants.FLY_WHEEL_BOTTOM, MotorType.kBrushless);

    private final RelativeEncoder shooterEncoder = topFlyWheel.getEncoder();

    private final PIDController shooterPID = new PIDController(Constants.ShooterConstants.P,
            Constants.ShooterConstants.I, Constants.ShooterConstants.D);

    public Shooter() {
        topFlyWheel.setInverted(true);
        bottomFlyWheel.follow(topFlyWheel);
    }

    public RelativeEncoder getEncoder() {
        return this.shooterEncoder;
    }

    public void murder() {
        topFlyWheel.set(0);
    }

    public void shoot(double wantedRPM) {
        topFlyWheel.set(shooterPID.calculate(shooterEncoder.getVelocity(),wantedRPM));
    }

    public void eatNote() {
        topFlyWheel.set(-1);
    }

    public CANSparkMax getBottomSparkMax() {
        return this.bottomFlyWheel;
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
        if (RobotContainer.isDebugMode()) {
            SmartDashboard.putNumber("shooter RPM", shooterEncoder.getVelocity());
        }
    }
}

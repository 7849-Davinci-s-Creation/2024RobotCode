package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.DashboardConfiguration;

public class Vision extends SubsystemBase implements DashboardConfiguration {
    public Vision() {

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
    }
}

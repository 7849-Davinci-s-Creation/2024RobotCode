package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.DashboardConfiguration;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class Vision extends SubsystemBase implements DashboardConfiguration {

    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html useful link for target info

    private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

    private final PIDController turnController = new PIDController(
            Constants.VisionConstants.AIM_ANGULAR_P,
            0,
            Constants.VisionConstants.AIM_ANGULAR_D
    );
    private final PIDController forwardController = new PIDController(
            Constants.VisionConstants.AIM_LINEAR_P,
            0,
            Constants.VisionConstants.AIM_LINEAR_D
    );

    public Vision() {

    }

    public List<PhotonTrackedTarget> getVisionTargets() {
        return camera.getLatestResult().getTargets();
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

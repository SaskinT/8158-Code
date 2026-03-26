package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private boolean targetVisible = false;
    private double targetYaw = 0.0;

    public VisionSubsystem(PhotonCamera camera) {
        this.camera = camera;
    }

    @Override
    public void periodic() {
        targetVisible = false;
        targetYaw = 0.0;

        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }
    }

    public boolean isTargetVisible() { return targetVisible; }
    public double getTargetYaw() { return targetYaw; }
}


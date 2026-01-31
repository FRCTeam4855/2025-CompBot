package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    private PhotonCamera camera;
    public boolean hasTargets;
    public double[]  targetCoords; //0 = X, 1 = Y, 2 = Z
    public int targetID;

    public Camera(String cameraName) { //Camera name in the PhotonClient
        camera = new PhotonCamera(cameraName);
    }

    @Override
    public void periodic() {
        gatherData();
    }
    
    private void gatherData() {
        var result = camera.getLatestResult();
        var target = result.getBestTarget();
        var translation = target.getBestCameraToTarget().getTranslation();

        hasTargets = result.hasTargets();
        targetCoords[0] = translation.getX();
        targetCoords[1] = translation.getY();
        targetCoords[2] = translation.getZ();
        targetID = target.getFiducialId();

        SmartDashboard.putBoolean(camera.getName() + "Has Target", hasTargets);
        SmartDashboard.putNumberArray(camera.getName() + "XYZ", targetCoords);
    }
}

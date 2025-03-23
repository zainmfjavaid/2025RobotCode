// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PhotonVision {
    private PhotonCamera camera = new PhotonCamera("HQ_Camera");
    private boolean hasResults = false;
    PhotonTrackedTarget target = new PhotonTrackedTarget();

    public PhotonVision() {}

    public void updateCameraResults() {
        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
        if (pipelineResults.size() > 0) {
            PhotonPipelineResult latestResult = pipelineResults.get(pipelineResults.size() - 1);

            if (latestResult.hasTargets()) {
                target = latestResult.getBestTarget();
                hasResults = true;
            } else {
                hasResults = false;
            }
        }
    }

    public double getXOffsetDegrees() {
        return target.getYaw();
    }

    public double getPitch() {
        return target.getPitch();
    }

    public boolean hasResults() {
        return hasResults;
    }

    public boolean isTargetVisible() {
        return hasResults && target != null && target.getFiducialId() != 0;
    }

    public int getTargetID() {
        return target.getFiducialId();
    }
    
    public double getTargetYaw() {
        return target.getYaw();
    }    

    public double getDistanceInches() {
        return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(Constants.cameraHeightInches), Units.inchesToMeters(Constants.aprilTagHeightInches), 22, -getPitch()));
    }

    public double getXOffsetInches() {
        return getDistanceInches() * Math.cos(getXOffsetDegrees());
    }

}

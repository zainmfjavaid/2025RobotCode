// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;

public class PhotonVision {
    private PhotonCamera camera = new PhotonCamera("CANmera");
    private boolean hasResults = false;
    PhotonTrackedTarget target = new PhotonTrackedTarget();

    public PhotonVision() {}

    public void updateCameraResults() {
        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
        PhotonPipelineResult latestResult = pipelineResults.get(pipelineResults.size() - 1);

        if (latestResult.hasTargets()) {
            target = latestResult.getBestTarget();
            hasResults = true;
        } else {
            hasResults = false;
        }
    }

    public double getXOffsetDegrees() {
        return target.getYaw();
    }

    public double getPitch() {
        return target.getPitch();
    }

    public double getSkew() {
        return target.getSkew();
    }

    public boolean hasResults() {
        return hasResults;
    }

    public double getDistanceInches() {
        double heightDifference = Math.abs(Constants.aprilTagHeightInches - Constants.cameraHeightInches);

        return Math.abs(Math.tan(getPitch()) * heightDifference);
    }

    public double getXOffsetInches() {
        return getDistanceInches() * Math.cos(getXOffsetDegrees());
    }

}

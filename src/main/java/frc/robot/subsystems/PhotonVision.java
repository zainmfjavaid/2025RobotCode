// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PhotonVision {
    private PhotonCamera camera = new PhotonCamera("HQ_Camera");
    private boolean hasResults = false;
    PhotonTrackedTarget target = new PhotonTrackedTarget();
    private Pigeon2 gyro = new Pigeon2(20, "CANivore2158");
    private int targetID = 0;
    private double yaw = 0;
    private double pitch = 0;

    public PhotonVision() {}

    public double getGyroAngle() {
        return gyro.getYaw().getValueAsDouble();
    }
    
    public void updateCameraResults() {
        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();
        if (pipelineResults.size() > 0) {
            PhotonPipelineResult latestResult = pipelineResults.get(pipelineResults.size() - 1);

            if (latestResult.hasTargets()) {
                target = latestResult.getBestTarget();
                pitch = target.getPitch();
                yaw = target.getYaw();
                targetID = target.getFiducialId();
                hasResults = true;
            } else {
                hasResults = false;
            }
        }
    }

    public double getXOffsetDegrees() {
        return target.getYaw() - 17;
    }

    public double getPitch() {
        return pitch;
    }

    public boolean hasResults() {
        return hasResults;
    }

    public boolean isTargetVisible() {
        return hasResults && target != null && target.getFiducialId() != 0;
    }

    public int getTargetID() {
        return targetID;
    }
    
    public double getTargetYaw() {
        return yaw;
    }    

    public double getDistanceInches() {
        return Units.metersToInches(PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(Constants.cameraHeightInches), Units.inchesToMeters(Constants.aprilTagHeightInches), Units.degreesToRadians(-22.0), Units.degreesToRadians(getPitch())));
    }

    public double getXOffsetInches() {
        return getDistanceInches() * Math.cos(getXOffsetDegrees());
    }

    public boolean alignedToTarget() {
        boolean rightReefYawGood = Constants.rightReefYaw - 2 < yaw && Constants.rightReefYaw + 2 > yaw;
        boolean rightReefPitchGood = Constants.rightReefPitch - 1.5 < pitch && Constants.rightReefPitch + 1.5 > pitch;

        boolean leftReefYawGood = Constants.leftReefYaw - 2 < yaw && Constants.leftReefYaw + 2 > yaw;
        boolean leftReefPitchGood = Constants.leftReefPitch - 1.5 < pitch && Constants.leftReefPitch + 1.5 > pitch;

        return (rightReefPitchGood && rightReefYawGood) || (leftReefPitchGood && leftReefYawGood);
    }

}

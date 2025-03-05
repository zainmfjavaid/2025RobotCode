// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import au.grapplerobotics.LaserCan;

/** Add your docs here. */
public class LaserCAN {
    LaserCan lc = new LaserCan(22);

    public LaserCAN() {
    }

    public void getLaserCANDistance() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            System.out.println("The target is " + measurement.distance_mm + "mm away!");
          } else {
            System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
            // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
          }
    }

}

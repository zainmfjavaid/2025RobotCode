package frc.robot.hardware;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;



public class LaserCan {
  private LaserCan lc;

    public LaserCan () {
    }
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
     lc = new LaserCan(0);
 // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
 try {
   lc.setRangingMode(LaserCan.RangingMode.SHORT);
   lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
   lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
 } catch (ConfigurationFailedException e) {
   System.out.println("Configuration failed! " + e);
 }
}

 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
     LaserCan.Measurement measurement = lc.getMeasurement();
 if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
   System.out.println("The target is " + measurement.distance_mm + "mm away!");
 } else {
   System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
   // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
 }
 }

 // Called once the command ends or is interrupted.
 @Override
 public void end() {

}

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
}




}
   
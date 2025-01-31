package frc.robot.sensors;

import au.grapplerobotics.LaserCan;
import frc.robot.Constants.CAN;
import frc.robot.util.ReefscapeUtils;
import frc.robot.util.ReefscapeUtils.BranchSide;

import java.util.function.Supplier;

import au.grapplerobotics.ConfigurationFailedException;

public class ProximitySensor {

    private LaserCan outerLeftLaserCan;
    private LaserCan innerLeftLaserCan;
    
    public ProximitySensor() {
        outerLeftLaserCan = new LaserCan(CAN.LEFT_LASER_CAN);
        try {
            outerLeftLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            outerLeftLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            outerLeftLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          } //i copied this whole thing
        innerLeftLaserCan = new LaserCan(CAN.RIGHT_LASER_CAN);
        try {
            innerLeftLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            innerLeftLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            innerLeftLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          } //i copied this whole thing
    }

    // returns in millimters
    public int getSensorDistance(ProxSensor sensor) {
        LaserCan laserCan = getLaserCan(sensor);
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        }
        return Integer.MAX_VALUE;
    }

    private LaserCan getLaserCan(ProxSensor sensor) {
        switch (sensor) {
            case OUTER_LEFT_SENSOR:
                return outerLeftLaserCan;
            case INNER_LEFT_SENSOR:
                return innerLeftLaserCan;
            default:
                return outerLeftLaserCan;
        }
    }

    public boolean isTriggered(ProxSensor sensor) {
        return getSensorDistance(sensor) > 100; //put value
    }

    public boolean isAtReefSide() {
        ProxSensor outerSensor = ReefscapeUtils.branchSide() == BranchSide.LEFT ?
            ProxSensor.OUTER_LEFT_SENSOR:
            ProxSensor.OUTER_RIGHT_SENSOR;

        ProxSensor innerSensor = ReefscapeUtils.branchSide() == BranchSide.LEFT ?
            ProxSensor.INNER_LEFT_SENSOR:
            ProxSensor.INNER_RIGHT_SENSOR;
        return getSensorDistance(outerSensor) > 250;// && getSensorDistance(innerSensor) < 150; // add sensor to reef number
    }

    public enum ProxSensor {
        OUTER_LEFT_SENSOR,
        INNER_LEFT_SENSOR,
        INNER_RIGHT_SENSOR,
        OUTER_RIGHT_SENSOR;
    }
}

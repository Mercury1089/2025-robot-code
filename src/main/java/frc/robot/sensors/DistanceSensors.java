package frc.robot.sensors;

import au.grapplerobotics.LaserCan;
import frc.robot.Constants.CAN;
import frc.robot.util.ReefscapeUtils;
import frc.robot.util.ReefscapeUtils.BranchSide;
import frc.robot.util.ReefscapeUtils.RobotZone;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.RobotEnableValue;

import au.grapplerobotics.ConfigurationFailedException;

public class DistanceSensors {

    private LaserCan innerSensor;
    private LaserCan outerSensor;

    private double outerTrigger = 380;
    private double innerTrigger = 340;

    private double awayFromReefError = 135;

    // private double awayFromReefError = innerTrigger;
    
    public DistanceSensors(int innerCANID, int outerCANID) {
        outerSensor = new LaserCan(outerCANID);
        try {
            outerSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            outerSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            outerSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          } //i copied this whole thing
        innerSensor = new LaserCan(innerCANID);
        try {
            innerSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            innerSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            innerSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          } //i copied this whole thing
    }

    public LaserCan getInnerSensor() {
        return innerSensor;
    }

    public LaserCan getOuterSensor() {
        return outerSensor;
    }

    // returns in millimters
    public int getSensorDistance(LaserCan sensor) {
        LaserCan.Measurement measurement = sensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        }
        return Integer.MAX_VALUE;
    }

    // private LaserCan getLaserCan(ProxSensor sensor) {
    //     switch (sensor) {
    //         case OUTER_LEFT_SENSOR:
    //             return outerLeftLaserCan;
    //         case INNER_LEFT_SENSOR:
    //             return innerLeftLaserCan;
    //         case OUTER_RIGHT_SENSOR:
    //             return outerRightLaserCan;
    //         case INNER_RIGHT_SENSOR:
    //             return innerRightLaserCan;
    //         default:
    //             return outerLeftLaserCan;
    //     }
    // }

    public boolean isOuterSensorTriggered() {
        // ProxSensor outerSensor = ReefscapeUtils.branchSide() == BranchSide.LEFT ?
        //     ProxSensor.OUTER_LEFT_SENSOR:
        //     ProxSensor.OUTER_RIGHT_SENSOR;
        
        return getSensorDistance(outerSensor) > outerTrigger;
    }

    public boolean isInnerSensorTriggered() {
        // ProxSensor innerSensor = ReefscapeUtils.branchSide() == BranchSide.LEFT ?
        //     ProxSensor.INNER_LEFT_SENSOR:
        //     ProxSensor.INNER_RIGHT_SENSOR;
        
        return getSensorDistance(innerSensor) < innerTrigger;
    }

    public boolean isAtReefSide() {
        return isInnerSensorTriggered() && isOuterSensorTriggered();
    }

    public boolean isTooFarLeft() {
        boolean tooLeft = false;

        if (isInnerSensorTriggered() && !isOuterSensorTriggered()) {
            tooLeft = false;
        } else if (!isInnerSensorTriggered() && isOuterSensorTriggered()) {
            tooLeft = true;
        }

        // RobotZone currentPref = ReefscapeUtils.preferredZone();

        // if (currentPref == RobotZone.BARGE || currentPref == RobotZone.BARGE_LEFT || currentPref == RobotZone.BARGE_RIGHT) {
        //     tooLeft = !tooLeft;
        // }

        return tooLeft;
    }

    public boolean isTooFarAwayFromReef() {
        double yComp = Math.sin(60.0) * getSensorDistance(innerSensor);

        return getSensorDistance(innerSensor) > awayFromReefError;
    }   

    public enum ProxSensor {
        OUTER_LEFT_SENSOR,
        INNER_LEFT_SENSOR,
        INNER_RIGHT_SENSOR,
        OUTER_RIGHT_SENSOR;
    }
}

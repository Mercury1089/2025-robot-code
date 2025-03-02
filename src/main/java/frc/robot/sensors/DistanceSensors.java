package frc.robot.sensors;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.LinearFilter;
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
    private LaserCan backSensor;

    private LinearFilter innerFilter;
    private LinearFilter outerFilter;
    private LinearFilter backFilter;

    private double outerTrigger;
    private double innerTrigger;

    private double awayFromReefError;

    // private double awayFromReefError = innerTrigger;
    
    public DistanceSensors(int innerCANID, int outerCANID, int innerTrigger, int outerTrigger, int awayFromReefError) {
        outerSensor = new LaserCan(outerCANID);
        this.innerTrigger = innerTrigger;
        this.outerTrigger = outerTrigger;
        this.awayFromReefError = awayFromReefError;
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
        
        innerFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        outerFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        backFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    }

    public DistanceSensors(int backCANID, double error) {
        outerSensor = new LaserCan(backCANID);
        awayFromReefError = error;
        try {
            outerSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            outerSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            outerSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
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

    public LaserCan getBackSensor() {
        return backSensor;
    }

    // returns in millimters
    private int getSensorDistance(LaserCan sensor) {
        LaserCan.Measurement measurement = sensor.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        }
        return Integer.MAX_VALUE;
    }

    public int getInnerSensorDistance() {
        return (int)Math.round(innerFilter.calculate((float)getSensorDistance(innerSensor)));
    }
    public int getOuterSensorDistance() {
        return (int)Math.round(outerFilter.calculate((float)getSensorDistance(outerSensor)));
    }
    public int getBackSensorDistance() {
        return (int)Math.round(backFilter.calculate((float)getSensorDistance(backSensor)));
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
        
        return getOuterSensorDistance() < outerTrigger;
    }

    public boolean isInnerSensorTriggered() {
        // ProxSensor innerSensor = ReefscapeUtils.branchSide() == BranchSide.LEFT ?
        //     ProxSensor.INNER_LEFT_SENSOR:
        //     ProxSensor.INNER_RIGHT_SENSOR;
        
        return getInnerSensorDistance() < innerTrigger;
    }

    public boolean isBackSensorTriggered() {
        return getBackSensorDistance() < awayFromReefError;
    }

    public boolean isAtReefSide() {
        return isInnerSensorTriggered() && !isOuterSensorTriggered();
    }

    public boolean isTooFarInside() {
        return isInnerSensorTriggered() && isOuterSensorTriggered();
    }

    public boolean isTooFarOutside() {
        return !isInnerSensorTriggered() && !isOuterSensorTriggered();
    }

    public boolean isTooFarLeft(Supplier<RobotZone> zone, Supplier<BranchSide> side) {
        return side.get() == BranchSide.LEFT ? isTooFarOutside() : isTooFarInside();
    }

    public boolean isTooFarAway() {
        return getSensorDistance(innerSensor) > awayFromReefError;
    }  

    public boolean isTooFarAwayFromReefBack() {
        return isBackSensorTriggered();
    }
}

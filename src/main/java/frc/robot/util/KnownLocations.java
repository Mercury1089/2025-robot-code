// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.UncheckedIOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Absolute (X, Y) of certain field locations
 * (thanks design :)
 */
public class KnownLocations {

    // The KnownLocations singleton instance returned by getKnownLocaitons()
    private static KnownLocations knownLocations = null;
    private static AprilTagFieldLayout fieldLayout = null;

    public static Pose2d REEF;

    // The alliance of the current KnownLocations
    public final Alliance alliance;

    public static Pose2d 
        rightBranchInRightZone,
        leftBranchInRightZone,
        rightBranchInBottomRightZone,
        leftBranchInBottomRightZone,
        rightBranchInBottomLeftZone,
        leftBranchInBottomLeftZone,
        rightBranchInLeftZone,
        leftBranchInLeftZone,
        rightBranchInTopLeftZone,
        leftBranchInTopLeftZone,
        rightBranchInTopRightZone,
        leftBranchInTopRightZone;

    /**
     * Load the field layout for the current year (currently CRESCENDO).
     * 
     * <p>
     * This is a singleton instance of the field layout for any class that needs it.
     * 
     * @return The current AprilTagFieldLayout or null if there is an UncheckedIOException loading the file.
     *         A warning is reported on the Driver Station if an exception is encountered
     */
    synchronized public static AprilTagFieldLayout getFieldLayout() {
        if (fieldLayout == null) {
            try {
                fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            } catch (UncheckedIOException e) {
                DriverStation.reportWarning("Failed to load AprilTagFieldLayout: " + e.getMessage(), true);
                fieldLayout = null;
            }
        }
        return fieldLayout;
    }

    /**
     * Get an updated KnownLocations based on the current alliance.
     * 
     * <p>
     * This will generate a new KnownLocations instance based on the current
     * alliance.
     * Do not save the returned instance. Always call this method to get the latest
     * copy.
     * 
     * @return The current KnownLocations instance. Never save this instance as it
     *         can change.
     */
    synchronized public static KnownLocations getKnownLocations() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (knownLocations == null || knownLocations.alliance != alliance) {
            knownLocations = new KnownLocations(alliance);
        }
        return knownLocations;
    }

    private KnownLocations(Alliance alliance) {
        this.alliance = alliance;

        //TODO: check headings
        if (alliance == Alliance.Blue) {
            rightBranchInBottomLeftZone = PathPointInch(159.522, 112.559, 60.0);
            leftBranchInBottomLeftZone = PathPointInch(146.522,120.065,60.0);
            rightBranchInBottomRightZone = PathPointInch(198.267, 112.559, 120);
            leftBranchInBottomRightZone = PathPointInch(211.267,120.065,120.0);
            rightBranchInLeftZone = PathPointInch(200.189,154.625,0.0);
            leftBranchInLeftZone = PathPointInch(127.149,167.625,0.0);
            rightBranchInRightZone = PathPointInch(230.640,167.625,180.0);
            leftBranchInRightZone = PathPointInch(230.640,154.625,180.0);
            leftBranchInTopLeftZone = PathPointInch(158.685,209.208,-60.0);
            rightBranchInTopLeftZone = PathPointInch(146.482,202.162,-60.0);
            leftBranchInTopRightZone = PathPointInch(199.103,209.208,-120.0);
            rightBranchInTopRightZone = PathPointInch(211.307, 202.162, -120.0);

            REEF = PathPointInch(176.75, 158.5, 0);
        } else {
            rightBranchInBottomLeftZone = PathPointInch(483.907, 120.0650, 60.0);
            leftBranchInBottomLeftZone = PathPointInch(496.907,112.559,60.0);
            rightBranchInBottomRightZone = PathPointInch(548.652, 120.065, 120);
            leftBranchInBottomRightZone = PathPointInch(535.652,112.559,120.0);
            rightBranchInLeftZone = PathPointInch(464.534,167.625,0.0);
            leftBranchInLeftZone = PathPointInch(464.534,154.625,0.0);
            rightBranchInRightZone = PathPointInch(568.025,167.625,180.0);
            leftBranchInRightZone = PathPointInch(568.025,154.625,180.0);
            leftBranchInTopLeftZone = PathPointInch(483.907,202.185,-60.0);
            rightBranchInTopLeftZone = PathPointInch(496.907,209.691,-60);
            leftBranchInTopRightZone = PathPointInch(548.652,202.185,-120.0);
            rightBranchInTopRightZone = PathPointInch(535.652, 209.691, -120.0);

            REEF = PathPointInch(514.14, 158.5, 0);
        }

    }

    /** Convenience method to create PathPoint from inches */
    private static Pose2d PathPointInch(double xInches, double yInches, double headingDegrees) {
        return new Pose2d(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)),
                Rotation2d.fromDegrees(headingDegrees));
    }
}
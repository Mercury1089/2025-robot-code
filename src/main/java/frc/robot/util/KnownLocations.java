// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.UncheckedIOException;

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

    // The alliance of the current KnownLocations
    public final Alliance alliance;

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
                fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
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

    }

    /** Convenience method to create PathPoint from inches */
    private static Pose2d PathPointInch(double xInches, double yInches, double headingDegrees) {
        return new Pose2d(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)),
                Rotation2d.fromDegrees(headingDegrees));
    }
}
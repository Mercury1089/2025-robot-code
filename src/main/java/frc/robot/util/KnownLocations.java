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

    public static Pose2d
        rightZone,
        bottomRightZone,
        bottomLeftZone,
        leftZone,
        topLeftZone,
        topRightZone;

    public static Pose2d
        leftCoralStationInside,
        leftCoralStationOutside,
        rightCoralStationInside,
        rightCoralStationOutside;

    public static Pose2d
        topMostStart,
        middleStart,
        bottomMostStart;

    public static Pose2d
        rightZoneAlgaeSafePoint,
        bottomRightZoneAlgaeSafePoint,
        bottomLeftZoneAlgaeSafePoint,
        leftZoneAlgaeSafePoint,
        topLeftZoneAlgaeSafePoint,
        topRightZoneAlgaeSafePoint;

    public static Pose2d
        rightZoneAlgaeScorePoint,
        bottomRightZoneAlgaeScorePoint,
        bottomLeftZoneAlgaeScorePoint,
        leftZoneAlgaeScorePoint,
        topLeftZoneAlgaeScorePoint,
        topRightZoneAlgaeScorePoint;

    public static final double ALGAE_SAFE_DISTANCE = 34.5;
    public static final double ALGAE_SCORE_DISTANCE = 16.5;

    /**
     * Load the field layout for the current year (currently CHARGED UP).
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
            leftBranchInBottomRightZone = PathPointInch(198.267, 112.559, 120);
            rightBranchInBottomRightZone = PathPointInch(211.267,120.065,120.0);
            rightBranchInLeftZone = PathPointInch(127.149,154.625,0.0);
            leftBranchInLeftZone = PathPointInch(127.149,167.625,0.0);
            leftBranchInRightZone = PathPointInch(230.640,154.625,180.0);
            rightBranchInRightZone = PathPointInch(230.640,167.625,180.0);
            leftBranchInTopLeftZone = PathPointInch(158.685,209.208,-60.0);
            rightBranchInTopLeftZone = PathPointInch(146.482,202.162,-60.0);
            rightBranchInTopRightZone = PathPointInch(199.103,209.208,-120.0);
            leftBranchInTopRightZone = PathPointInch(211.307, 202.162, -120.0);

            leftZone = PathPointInch(123.149,161.125,0);
            rightZone = PathPointInch(234.640, 161.125, 180);
            topLeftZone = PathPointInch(151.022, 209.402, -60);
            bottomLeftZone = PathPointInch(151.022, 112.848, 60);
            topRightZone = PathPointInch(206.767, 209.402, -120);
            bottomRightZone = PathPointInch(206.767, 112.848, 120);

            rightCoralStationInside = PathPointInch(30.783, 54.988, -126 + 180);
            rightCoralStationOutside = PathPointInch(62.743, 32.206, -126  + 180);

            leftCoralStationInside = PathPointInch(30.783, 267.262, 126 - 180);
            leftCoralStationOutside = PathPointInch(62.743, 290.044, 126 - 180);

            Rotation2d bottomLeftRotation = getFieldLayout().getTagPose(17).get().getRotation().toRotation2d();
            bottomLeftZoneAlgaeSafePoint = PathPointInch(160.39 + ALGAE_SAFE_DISTANCE*bottomLeftRotation.getCos(),
                                                        130.17 + ALGAE_SAFE_DISTANCE*bottomLeftRotation.getSin(),
                                                        60);
            bottomLeftZoneAlgaeScorePoint = PathPointInch(160.39 + ALGAE_SCORE_DISTANCE*bottomLeftRotation.getCos(),
                                                        130.17 + ALGAE_SCORE_DISTANCE*bottomLeftRotation.getSin(),
                                                        60);

            Rotation2d bottomRightRotation = getFieldLayout().getTagPose(22).get().getRotation().toRotation2d();
            bottomRightZoneAlgaeSafePoint = PathPointInch(193.10 + ALGAE_SAFE_DISTANCE*bottomRightRotation.getCos(),
                                                        130.17 + ALGAE_SAFE_DISTANCE*bottomRightRotation.getSin(),
                                                        120);
            bottomRightZoneAlgaeScorePoint = PathPointInch(193.10 + ALGAE_SCORE_DISTANCE*bottomRightRotation.getCos(),
                                                        130.17 + ALGAE_SCORE_DISTANCE*bottomRightRotation.getSin(),
                                                        120);    

            Rotation2d topLeftRotation = getFieldLayout().getTagPose(19).get().getRotation().toRotation2d();
            topLeftZoneAlgaeSafePoint = PathPointInch(160.39 + ALGAE_SAFE_DISTANCE*topLeftRotation.getCos(),
                                                        186.83 + ALGAE_SAFE_DISTANCE*topLeftRotation.getSin(),
                                                        -60);
            topLeftZoneAlgaeScorePoint = PathPointInch(160.39 + ALGAE_SCORE_DISTANCE*topLeftRotation.getCos(),
                                                        186.83 + ALGAE_SCORE_DISTANCE*topLeftRotation.getSin(),
                                                        -60);

            Rotation2d topRightRotation = getFieldLayout().getTagPose(20).get().getRotation().toRotation2d();
            topRightZoneAlgaeSafePoint = PathPointInch(193.10 + ALGAE_SAFE_DISTANCE*topRightRotation.getCos(),
                                                        186.83 + ALGAE_SAFE_DISTANCE*topRightRotation.getSin(),
                                                        -120);
            topRightZoneAlgaeScorePoint = PathPointInch(193.10 + ALGAE_SCORE_DISTANCE*topRightRotation.getCos(),
                                                        186.83 + ALGAE_SCORE_DISTANCE*topRightRotation.getSin(),
                                                        -120); 

            rightZoneAlgaeScorePoint = PathPointInch(209.49 + ALGAE_SCORE_DISTANCE, 158.50, 180);
            leftZoneAlgaeScorePoint = PathPointInch(144.00 - ALGAE_SCORE_DISTANCE, 158.50, 0);
            
            rightZoneAlgaeSafePoint = PathPointInch(209.49 + ALGAE_SAFE_DISTANCE, 158.50, 180);
            leftZoneAlgaeSafePoint = PathPointInch(144.00 - ALGAE_SAFE_DISTANCE, 158.50, 0);
                
            REEF = PathPointInch(176.75, 158.5, 0);

            topMostStart = PathPointInch(290.0,246.0,180.0);
            middleStart = PathPointInch(290.0,158.5,180.0);
            bottomMostStart = PathPointInch(290.0,76.5,180.0);

        } else {
            rightBranchInBottomLeftZone = PathPointInch(483.907, 120.0650, 60.0);
            leftBranchInBottomLeftZone = PathPointInch(496.907,112.559,60.0);
            leftBranchInBottomRightZone = PathPointInch(548.652, 120.065, 120);
            rightBranchInBottomRightZone = PathPointInch(535.652,112.559,120.0);
            rightBranchInLeftZone = PathPointInch(464.534,167.625,0.0);
            leftBranchInLeftZone = PathPointInch(464.534,154.625,0.0);
            leftBranchInRightZone = PathPointInch(568.025,167.625,180.0);
            rightBranchInRightZone = PathPointInch(568.025,154.625,180.0);
            leftBranchInTopLeftZone = PathPointInch(483.907,202.185,-60.0);
            rightBranchInTopLeftZone = PathPointInch(496.907,209.691,-60);
            rightBranchInTopRightZone = PathPointInch(548.652,202.185,-120.0);
            leftBranchInTopRightZone = PathPointInch(535.652, 209.691, -120.0);

            leftZone = PathPointInch(460.534, 161.125, 0);
            rightZone = PathPointInch(572.025, 161.125, 180);
            topLeftZone = PathPointInch(488.407, 209.402, -60);
            bottomLeftZone = PathPointInch(488.407, 112.848, 60);
            topRightZone = PathPointInch(544.152, 209.402, -120);
            bottomRightZone = PathPointInch(544.152, 112.848, 120);

            
            rightCoralStationInside = PathPointInch(664.391, 267.262, 54 - 180);
            rightCoralStationOutside = PathPointInch(632.431, 290.044, 54 - 180);

            leftCoralStationInside = PathPointInch(664.391, 54.988, -54 + 180);
            leftCoralStationOutside = PathPointInch(632.431, 54.988, -54 + 180);

            Rotation2d bottomLeftRotation = getFieldLayout().getTagPose(11).get().getRotation().toRotation2d();
            bottomLeftZoneAlgaeSafePoint = PathPointInch(497.77 + ALGAE_SAFE_DISTANCE*bottomLeftRotation.getCos(),
                                                        130.17 + ALGAE_SAFE_DISTANCE*bottomLeftRotation.getSin(),
                                                        60);
            bottomLeftZoneAlgaeScorePoint = PathPointInch(497.77 + ALGAE_SCORE_DISTANCE*bottomLeftRotation.getCos(),
                                                        130.17 + ALGAE_SCORE_DISTANCE*bottomLeftRotation.getSin(),
                                                        60);
            Rotation2d bottomRightRotation = getFieldLayout().getTagPose(6).get().getRotation().toRotation2d();
            bottomRightZoneAlgaeSafePoint = PathPointInch(530.49 + ALGAE_SAFE_DISTANCE*bottomRightRotation.getCos(),
                                                        530.49 + ALGAE_SAFE_DISTANCE*bottomRightRotation.getSin(),
                                                        120);
            bottomRightZoneAlgaeScorePoint = PathPointInch(193.10 + ALGAE_SCORE_DISTANCE*bottomRightRotation.getCos(),
                                                        130.17 + ALGAE_SCORE_DISTANCE*bottomRightRotation.getSin(),
                                                        120);    
            Rotation2d topLeftRotation = getFieldLayout().getTagPose(9).get().getRotation().toRotation2d();
            topLeftZoneAlgaeSafePoint = PathPointInch(497.77 + ALGAE_SAFE_DISTANCE*topLeftRotation.getCos(),
                                                        186.83 + ALGAE_SAFE_DISTANCE*topLeftRotation.getSin(),
                                                        -60);
            topLeftZoneAlgaeScorePoint = PathPointInch(497.77 + ALGAE_SCORE_DISTANCE*topLeftRotation.getCos(),
                                                        186.83 + ALGAE_SCORE_DISTANCE*topLeftRotation.getSin(),
                                                        -60);
            Rotation2d topRightRotation = getFieldLayout().getTagPose(8).get().getRotation().toRotation2d();
            topRightZoneAlgaeSafePoint = PathPointInch(530.49 + ALGAE_SAFE_DISTANCE*topRightRotation.getCos(),
                                                        186.83 + ALGAE_SAFE_DISTANCE*topRightRotation.getSin(),
                                                        -120);
            topRightZoneAlgaeScorePoint = PathPointInch(530.49 + ALGAE_SCORE_DISTANCE*topRightRotation.getCos(),
                                                        186.83 + ALGAE_SCORE_DISTANCE*topRightRotation.getSin(),
                                                        -120); 
            
            rightZoneAlgaeScorePoint = PathPointInch(546.87 + ALGAE_SCORE_DISTANCE, 158.50, 180);
            leftZoneAlgaeScorePoint = PathPointInch(481.39 - ALGAE_SCORE_DISTANCE, 158.50, 0);
                                                        
            rightZoneAlgaeSafePoint = PathPointInch(546.87 + ALGAE_SAFE_DISTANCE, 158.50, 180);
            leftZoneAlgaeSafePoint = PathPointInch(481.390 - ALGAE_SAFE_DISTANCE, 158.50, 0);

            REEF = PathPointInch(514.14, 158.5, 0);
            
            topMostStart = PathPointInch(403,246,0);
            middleStart = PathPointInch(403,158.5,0);
            bottomMostStart = PathPointInch(403,76.5,0);
        }

    }

    /** Convenience method to create PathPoint from inches */
    private static Pose2d PathPointInch(double xInches, double yInches, double headingDegrees) {
        return new Pose2d(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)),
                Rotation2d.fromDegrees(headingDegrees));
    }
}
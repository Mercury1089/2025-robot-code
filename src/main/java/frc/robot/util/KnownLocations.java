// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOError;
import java.io.IOException;
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
        bargeSideLeftBranch,
        bargeSideRightBranch,
        rightBargeSideLeftBranch,
        rightBargeSideRightBranch,
        closeRightSideRightBranch,
        closeRightSideLeftBranch,
        closeSideRightBranch,
        closeSideLeftBranch,
        leftCloseSideRightBranch,
        leftCloseSideLeftBranch,
        leftBargeSideLeftBranch,
        leftBargeSideRightBranch;

    public static Pose2d
        bargeSide,
        rightBargeSide,
        rightCloseSide,
        closeSide,
        leftCloseSide,
        leftBargeSide;

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
        bargeSideAlgaeSafePoint,
        rightBargeSideAlgaeSafePoint,
        rightCloseSideAlgaeSafePoint,
        closeSideAlgaeSafePoint,
        leftCloseSideAlgaeSafePoint,
        leftBargeSideAlgaeSafePoint;

    public static Pose2d
        bargeSideAlgaeScorePoint,
        rightBargeSideALgaeScorePoint,
        rightCloseSideAlgaeScorePoint,
        closeSideAlgaeScorePoint,
        leftCloseSideAlgaeScorePoint,
        leftBargeSideAlgaeScorePoint;

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
            try {//C:/Users/Mercury1089/git/
                fieldLayout = AprilTagFieldLayout.loadFromResource("2025-robot-code/src/main/java/frc/robot/util/combined_calibration.json");
            } catch (IOException e) {
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
            closeRightSideRightBranch = PathPointInch(159.522, 112.559, 60.0);
            closeRightSideLeftBranch = PathPointInch(146.522,120.065,60.0);
            rightBargeSideRightBranch = PathPointInch(198.267, 112.559, 120);
            rightBargeSideLeftBranch = PathPointInch(211.267,120.065,120.0);
            closeSideRightBranch = PathPointInch(127.149,154.625,0.0);
            closeSideLeftBranch = PathPointInch(127.149,167.625,0.0);
            bargeSideRightBranch = PathPointInch(230.640,154.625,180.0);
            bargeSideLeftBranch = PathPointInch(230.640,167.625,180.0);
            leftCloseSideLeftBranch = PathPointInch(158.685,209.208,-60.0);
            leftCloseSideRightBranch = PathPointInch(146.482,202.162,-60.0);
            leftBargeSideLeftBranch = PathPointInch(199.103,209.208,-120.0);
            leftBargeSideRightBranch = PathPointInch(211.307, 202.162, -120.0);

            closeSide = PathPointInch(123.149,161.125,0);
            bargeSide = PathPointInch(234.640, 161.125, 180);
            leftCloseSide = PathPointInch(151.022, 209.402, -60);
            rightCloseSide = PathPointInch(151.022, 112.848, 60);
            leftBargeSide = PathPointInch(206.767, 209.402, -120);
            rightBargeSide = PathPointInch(206.767, 112.848, 120);

            rightCoralStationInside = PathPointInch(30.783, 54.988, -126 + 180);
            rightCoralStationOutside = PathPointInch(62.743, 32.206, -126  + 180);

            leftCoralStationInside = PathPointInch(30.783, 267.262, 126 - 180);
            leftCoralStationOutside = PathPointInch(62.743, 290.044, 126 - 180);

            Rotation2d bottomLeftRotation = getFieldLayout().getTagPose(17).get().getRotation().toRotation2d();
            rightCloseSideAlgaeSafePoint = PathPointInch(160.39 + ALGAE_SAFE_DISTANCE*bottomLeftRotation.getCos(),
                                                        130.17 + ALGAE_SAFE_DISTANCE*bottomLeftRotation.getSin(),
                                                        60);
            rightCloseSideAlgaeScorePoint = PathPointInch(160.39 + ALGAE_SCORE_DISTANCE*bottomLeftRotation.getCos(),
                                                        130.17 + ALGAE_SCORE_DISTANCE*bottomLeftRotation.getSin(),
                                                        60);

            Rotation2d bottomRightRotation = getFieldLayout().getTagPose(22).get().getRotation().toRotation2d();
            rightBargeSideAlgaeSafePoint = PathPointInch(193.10 + ALGAE_SAFE_DISTANCE*bottomRightRotation.getCos(),
                                                        130.17 + ALGAE_SAFE_DISTANCE*bottomRightRotation.getSin(),
                                                        120);
            rightBargeSideALgaeScorePoint = PathPointInch(193.10 + ALGAE_SCORE_DISTANCE*bottomRightRotation.getCos(),
                                                        130.17 + ALGAE_SCORE_DISTANCE*bottomRightRotation.getSin(),
                                                        120);    

            Rotation2d topLeftRotation = getFieldLayout().getTagPose(19).get().getRotation().toRotation2d();
            leftCloseSideAlgaeSafePoint = PathPointInch(160.39 + ALGAE_SAFE_DISTANCE*topLeftRotation.getCos(),
                                                        186.83 + ALGAE_SAFE_DISTANCE*topLeftRotation.getSin(),
                                                        -60);
            leftCloseSideAlgaeScorePoint = PathPointInch(160.39 + ALGAE_SCORE_DISTANCE*topLeftRotation.getCos(),
                                                        186.83 + ALGAE_SCORE_DISTANCE*topLeftRotation.getSin(),
                                                        -60);

            Rotation2d topRightRotation = getFieldLayout().getTagPose(20).get().getRotation().toRotation2d();
            leftBargeSideAlgaeSafePoint = PathPointInch(193.10 + ALGAE_SAFE_DISTANCE*topRightRotation.getCos(),
                                                        186.83 + ALGAE_SAFE_DISTANCE*topRightRotation.getSin(),
                                                        -120);
            leftBargeSideAlgaeScorePoint = PathPointInch(193.10 + ALGAE_SCORE_DISTANCE*topRightRotation.getCos(),
                                                        186.83 + ALGAE_SCORE_DISTANCE*topRightRotation.getSin(),
                                                        -120); 

            bargeSideAlgaeScorePoint = PathPointInch(209.49 + ALGAE_SCORE_DISTANCE, 158.50, 180);
            closeSideAlgaeScorePoint = PathPointInch(144.00 - ALGAE_SCORE_DISTANCE, 158.50, 0);
            
            bargeSideAlgaeSafePoint = PathPointInch(209.49 + ALGAE_SAFE_DISTANCE, 158.50, 180);
            closeSideAlgaeSafePoint = PathPointInch(144.00 - ALGAE_SAFE_DISTANCE, 158.50, 0);
                
            REEF = PathPointInch(176.75, 158.5, 0);

            topMostStart = PathPointInch(290.0,246.0,180.0);
            middleStart = PathPointInch(290.0,158.5,180.0);
            bottomMostStart = PathPointInch(290.0,76.5,180.0);

        } else {
            closeRightSideRightBranch = PathPointInch(483.907, 120.0650, 60.0);
            closeRightSideLeftBranch = PathPointInch(496.907,112.559,60.0);
            rightBargeSideRightBranch = PathPointInch(548.652, 120.065, 120);
            rightBargeSideLeftBranch = PathPointInch(535.652,112.559,120.0);
            closeSideRightBranch = PathPointInch(464.534,167.625,0.0);
            closeSideLeftBranch = PathPointInch(464.534,154.625,0.0);
            bargeSideRightBranch = PathPointInch(568.025,167.625,180.0);
            bargeSideLeftBranch = PathPointInch(568.025,154.625,180.0);
            leftCloseSideLeftBranch = PathPointInch(483.907,202.185,-60.0);
            leftCloseSideRightBranch = PathPointInch(496.907,209.691,-60);
            leftBargeSideLeftBranch = PathPointInch(548.652,202.185,-120.0);
            leftBargeSideRightBranch = PathPointInch(535.652, 209.691, -120.0);

            closeSide = PathPointInch(460.534, 161.125, 0);
            bargeSide = PathPointInch(572.025, 161.125, 180);
            leftCloseSide = PathPointInch(488.407, 209.402, -60);
            rightCloseSide = PathPointInch(488.407, 112.848, 60);
            leftBargeSide = PathPointInch(544.152, 209.402, -120);
            rightBargeSide = PathPointInch(544.152, 112.848, 120);

            
            rightCoralStationInside = PathPointInch(664.391, 267.262, 54 - 180);
            rightCoralStationOutside = PathPointInch(632.431, 290.044, 54 - 180);

            leftCoralStationInside = PathPointInch(664.391, 54.988, -54 + 180);
            leftCoralStationOutside = PathPointInch(632.431, 54.988, -54 + 180);

            Rotation2d bottomLeftRotation = getFieldLayout().getTagPose(11).get().getRotation().toRotation2d();
            rightCloseSideAlgaeSafePoint = PathPointInch(497.77 + ALGAE_SAFE_DISTANCE*bottomLeftRotation.getCos(),
                                                        130.17 + ALGAE_SAFE_DISTANCE*bottomLeftRotation.getSin(),
                                                        60);
            rightCloseSideAlgaeScorePoint = PathPointInch(497.77 + ALGAE_SCORE_DISTANCE*bottomLeftRotation.getCos(),
                                                        130.17 + ALGAE_SCORE_DISTANCE*bottomLeftRotation.getSin(),
                                                        60);
            Rotation2d bottomRightRotation = getFieldLayout().getTagPose(6).get().getRotation().toRotation2d();
            rightBargeSideAlgaeSafePoint = PathPointInch(530.49 + ALGAE_SAFE_DISTANCE*bottomRightRotation.getCos(),
                                                        530.49 + ALGAE_SAFE_DISTANCE*bottomRightRotation.getSin(),
                                                        120);
            rightBargeSideALgaeScorePoint = PathPointInch(193.10 + ALGAE_SCORE_DISTANCE*bottomRightRotation.getCos(),
                                                        130.17 + ALGAE_SCORE_DISTANCE*bottomRightRotation.getSin(),
                                                        120);    
            Rotation2d topLeftRotation = getFieldLayout().getTagPose(9).get().getRotation().toRotation2d();
            leftCloseSideAlgaeSafePoint = PathPointInch(497.77 + ALGAE_SAFE_DISTANCE*topLeftRotation.getCos(),
                                                        186.83 + ALGAE_SAFE_DISTANCE*topLeftRotation.getSin(),
                                                        -60);
            leftCloseSideAlgaeScorePoint = PathPointInch(497.77 + ALGAE_SCORE_DISTANCE*topLeftRotation.getCos(),
                                                        186.83 + ALGAE_SCORE_DISTANCE*topLeftRotation.getSin(),
                                                        -60);
            Rotation2d topRightRotation = getFieldLayout().getTagPose(8).get().getRotation().toRotation2d();
            leftBargeSideAlgaeSafePoint = PathPointInch(530.49 + ALGAE_SAFE_DISTANCE*topRightRotation.getCos(),
                                                        186.83 + ALGAE_SAFE_DISTANCE*topRightRotation.getSin(),
                                                        -120);
            leftBargeSideAlgaeScorePoint = PathPointInch(530.49 + ALGAE_SCORE_DISTANCE*topRightRotation.getCos(),
                                                        186.83 + ALGAE_SCORE_DISTANCE*topRightRotation.getSin(),
                                                        -120); 
            
            bargeSideAlgaeScorePoint = PathPointInch(546.87 + ALGAE_SCORE_DISTANCE, 158.50, 180);
            closeSideAlgaeScorePoint = PathPointInch(481.39 - ALGAE_SCORE_DISTANCE, 158.50, 0);
                                                        
            bargeSideAlgaeSafePoint = PathPointInch(546.87 + ALGAE_SAFE_DISTANCE, 158.50, 180);
            closeSideAlgaeSafePoint = PathPointInch(481.390 - ALGAE_SAFE_DISTANCE, 158.50, 0);

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
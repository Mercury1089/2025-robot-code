// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Rotation;

import java.io.IOError;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Absolute (X, Y) of certain field locations
 * (thanks design :)
 */
public class KnownLocations {

    // The KnownLocations singleton instance returned by getKnownLocaitons()
    private static KnownLocations knownLocations = null;
    private static AprilTagFieldLayout fieldLayout = null;

    public final Pose2d REEF;

    // The alliance of the current KnownLocations
    public final Alliance alliance;

    public final Pose2d 
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

    public final Pose2d
        bargeSide,
        rightBargeSide,
        rightCloseSide,
        closeSide,
        leftCloseSide,
        leftBargeSide;

    public final Pose2d
        leftCoralStationInside,
        leftCoralStationOutside,
        rightCoralStationInside,
        rightCoralStationOutside;

    public final Pose2d
        topMostStart,
        middleStart,
        bottomMostStart;

    public final Pose2d
        bargeSideAlgaeSafePoint,
        rightBargeSideAlgaeSafePoint,
        rightCloseSideAlgaeSafePoint,
        closeSideAlgaeSafePoint,
        leftCloseSideAlgaeSafePoint,
        leftBargeSideAlgaeSafePoint;

    public final Pose2d
        bargeSideAlgaeScorePoint,
        rightBargeSideALgaeScorePoint,
        rightCloseSideAlgaeScorePoint,
        closeSideAlgaeScorePoint,
        leftCloseSideAlgaeScorePoint,
        leftBargeSideAlgaeScorePoint;

    public static final double ALGAE_SAFE_DISTANCE = 34.5;
    public static final double ALGAE_SCORE_DISTANCE = 16.5;

    public final double X_OFFSET = 16;
    public final double Y_OFFSET = 8;

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
                fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/combined_calibration.json");
                // fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
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

        Transform2d posXposYOffsets = new Transform2d(Units.inchesToMeters(X_OFFSET), Units.inchesToMeters(Y_OFFSET), Rotation2d.fromDegrees(0.0));
        Transform2d posXnegYOffsets = new Transform2d(Units.inchesToMeters(X_OFFSET), Units.inchesToMeters(-Y_OFFSET), Rotation2d.fromDegrees(0.0));
        Rotation2d rotate180 = Rotation2d.fromDegrees(180);

        Transform2d algaeSafe = new Transform2d(Units.inchesToMeters(ALGAE_SAFE_DISTANCE), Units.inchesToMeters(0), Rotation2d.fromDegrees(0.0));
        Transform2d algaeScore = new Transform2d(Units.inchesToMeters(ALGAE_SCORE_DISTANCE), Units.inchesToMeters(0), Rotation2d.fromDegrees(0.0));

        //TODO: check headings
        if (alliance == Alliance.Blue) {
            // closeRightSideRightBranch = PathPointInch(159.522, 112.559, 60.0);
            Pose2d closeRightTag = getFieldLayout().getTagPose(17).get().toPose2d();
            closeRightSideRightBranch = new Pose2d(closeRightTag.transformBy(posXposYOffsets).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));
            closeRightSideLeftBranch = new Pose2d(closeRightTag.transformBy(posXnegYOffsets).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));

            Pose2d bargeRightTag = getFieldLayout().getTagPose(22).get().toPose2d();
            rightBargeSideRightBranch = new Pose2d(bargeRightTag.transformBy(posXnegYOffsets).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));
            rightBargeSideLeftBranch = new Pose2d(bargeRightTag.transformBy(posXposYOffsets).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));

            Pose2d closeTag = getFieldLayout().getTagPose(18).get().toPose2d();
            closeSideRightBranch = new Pose2d(closeTag.transformBy(posXposYOffsets).getTranslation(), closeTag.getRotation().rotateBy(rotate180));
            closeSideLeftBranch = new Pose2d(closeTag.transformBy(posXnegYOffsets).getTranslation(), closeTag.getRotation().rotateBy(rotate180));

            Pose2d bargeTag = getFieldLayout().getTagPose(21).get().toPose2d();
            bargeSideRightBranch = new Pose2d(bargeTag.transformBy(posXnegYOffsets).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));
            bargeSideLeftBranch = new Pose2d(bargeTag.transformBy(posXposYOffsets).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));

            Pose2d closeLeftTag = getFieldLayout().getTagPose(19).get().toPose2d();
            leftCloseSideLeftBranch = new Pose2d(closeLeftTag.transformBy(posXnegYOffsets).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));
            leftCloseSideRightBranch = new Pose2d(closeLeftTag.transformBy(posXposYOffsets).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));

            Pose2d leftBargeTag = getFieldLayout().getTagPose(20).get().toPose2d();
            leftBargeSideLeftBranch = new Pose2d(leftBargeTag.transformBy(posXposYOffsets).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));
            leftBargeSideRightBranch = new Pose2d(leftBargeTag.transformBy(posXnegYOffsets).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));

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

            rightCloseSideAlgaeSafePoint = new Pose2d(closeRightTag.transformBy(algaeSafe).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));
            rightCloseSideAlgaeScorePoint = new Pose2d(closeRightTag.transformBy(algaeScore).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));

            rightBargeSideAlgaeSafePoint = new Pose2d(bargeRightTag.transformBy(algaeSafe).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));
            rightBargeSideALgaeScorePoint = new Pose2d(bargeRightTag.transformBy(algaeScore).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));
    
            leftCloseSideAlgaeSafePoint = new Pose2d(closeLeftTag.transformBy(algaeSafe).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));
            leftCloseSideAlgaeScorePoint = new Pose2d(closeLeftTag.transformBy(algaeScore).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));

            leftBargeSideAlgaeSafePoint = new Pose2d(leftBargeTag.transformBy(algaeSafe).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));
            leftBargeSideAlgaeScorePoint = new Pose2d(leftBargeTag.transformBy(algaeScore).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));

            bargeSideAlgaeSafePoint = new Pose2d(bargeTag.transformBy(algaeSafe).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));
            bargeSideAlgaeScorePoint = new Pose2d(bargeTag.transformBy(algaeScore).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));
            
            closeSideAlgaeSafePoint = new Pose2d(closeTag.transformBy(algaeSafe).getTranslation(), closeTag.getRotation().rotateBy(rotate180));
            closeSideAlgaeScorePoint = new Pose2d(closeTag.transformBy(algaeScore).getTranslation(), closeTag.getRotation().rotateBy(rotate180));
                
            REEF = PathPointInch(176.75, 158.5, 0);

            topMostStart = PathPointInch(290.0,246.0,180.0);
            middleStart = PathPointInch(290.0,158.5,180.0);
            bottomMostStart = PathPointInch(290.0,76.5,180.0);

        } else {
            Pose2d closeRightTag = getFieldLayout().getTagPose(8).get().toPose2d();
            closeRightSideRightBranch = new Pose2d(closeRightTag.transformBy(posXposYOffsets).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));
            closeRightSideLeftBranch = new Pose2d(closeRightTag.transformBy(posXnegYOffsets).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));

            Pose2d bargeRightTag = getFieldLayout().getTagPose(9).get().toPose2d();
            rightBargeSideRightBranch = new Pose2d(bargeRightTag.transformBy(posXnegYOffsets).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));
            rightBargeSideLeftBranch = new Pose2d(bargeRightTag.transformBy(posXposYOffsets).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));

            Pose2d closeTag = getFieldLayout().getTagPose(7).get().toPose2d();
            closeSideRightBranch = new Pose2d(closeTag.transformBy(posXposYOffsets).getTranslation(), closeTag.getRotation().rotateBy(rotate180));
            closeSideLeftBranch = new Pose2d(closeTag.transformBy(posXnegYOffsets).getTranslation(), closeTag.getRotation().rotateBy(rotate180));

            Pose2d bargeTag = getFieldLayout().getTagPose(10).get().toPose2d();
            bargeSideRightBranch = new Pose2d(bargeTag.transformBy(posXnegYOffsets).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));
            bargeSideLeftBranch = new Pose2d(bargeTag.transformBy(posXposYOffsets).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));

            Pose2d closeLeftTag = getFieldLayout().getTagPose(6).get().toPose2d();
            leftCloseSideLeftBranch = new Pose2d(closeLeftTag.transformBy(posXnegYOffsets).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));
            leftCloseSideRightBranch = new Pose2d(closeLeftTag.transformBy(posXposYOffsets).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));

            Pose2d leftBargeTag = getFieldLayout().getTagPose(11).get().toPose2d();
            leftBargeSideLeftBranch = new Pose2d(leftBargeTag.transformBy(posXposYOffsets).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));
            leftBargeSideRightBranch = new Pose2d(leftBargeTag.transformBy(posXnegYOffsets).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));

            closeSide = PathPointInch(460.534, 161.125, 0);
            bargeSide = PathPointInch(572.025, 161.125, 180);
            leftCloseSide = PathPointInch(488.407, 209.402, -60);
            rightCloseSide = PathPointInch(488.407, 112.848, 60);
            leftBargeSide = PathPointInch(544.152, 209.402, -120);
            rightBargeSide = PathPointInch(544.152, 112.848, 120);

            
            rightCoralStationInside = PathPointInch(664.391, 267.262, 54 - 180);
            rightCoralStationOutside = PathPointInch(632.431, 290.044, 54 - 180);

            leftCoralStationInside = PathPointInch(664.391, 54.988, -54 + 180);
            leftCoralStationOutside = PathPointInch(632.431, 32.206, -54 + 180);

            rightCloseSideAlgaeSafePoint = new Pose2d(closeRightTag.transformBy(algaeSafe).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));
            rightCloseSideAlgaeScorePoint = new Pose2d(closeRightTag.transformBy(algaeScore).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));

            rightBargeSideAlgaeSafePoint = new Pose2d(bargeRightTag.transformBy(algaeSafe).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));
            rightBargeSideALgaeScorePoint = new Pose2d(bargeRightTag.transformBy(algaeScore).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));
    
            leftCloseSideAlgaeSafePoint = new Pose2d(closeLeftTag.transformBy(algaeSafe).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));
            leftCloseSideAlgaeScorePoint = new Pose2d(closeLeftTag.transformBy(algaeScore).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));

            leftBargeSideAlgaeSafePoint = new Pose2d(leftBargeTag.transformBy(algaeSafe).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));
            leftBargeSideAlgaeScorePoint = new Pose2d(leftBargeTag.transformBy(algaeScore).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));

            bargeSideAlgaeSafePoint = new Pose2d(bargeTag.transformBy(algaeSafe).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));
            bargeSideAlgaeScorePoint = new Pose2d(bargeTag.transformBy(algaeScore).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));
            
            closeSideAlgaeSafePoint = new Pose2d(closeTag.transformBy(algaeSafe).getTranslation(), closeTag.getRotation().rotateBy(rotate180));
            closeSideAlgaeScorePoint = new Pose2d(closeTag.transformBy(algaeScore).getTranslation(), closeTag.getRotation().rotateBy(rotate180)); 
            
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
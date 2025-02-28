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
        rightBargeSideAlgaeScorePoint,
        rightCloseSideAlgaeScorePoint,
        closeSideAlgaeScorePoint,
        leftCloseSideAlgaeScorePoint,
        leftBargeSideAlgaeScorePoint;

    public final Pose2d
        processor;

    public static final double ALGAE_SAFE_DISTANCE = 34.5;
    public static final double ALGAE_SCORE_DISTANCE = 21.0;

    public static final double CORAL_STATION_DISTANCE_X = 18;
    public static final double CORAL_STATION_DISTANCE_Y = 13;

    public final double X_OFFSET = 20;
    public final double Y_OFFSET = 8;
    public final double BIGGER_Y_OFFSET = 10;

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
                // fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/combined_calibration.json");
                fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            } catch (Exception e) {
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

        Transform2d posXposBiggerYOffsets = new Transform2d(Units.inchesToMeters(X_OFFSET), Units.inchesToMeters(BIGGER_Y_OFFSET), Rotation2d.fromDegrees(0.0));
        Transform2d posXnegYBiggerOffsets = new Transform2d(Units.inchesToMeters(X_OFFSET), Units.inchesToMeters(-BIGGER_Y_OFFSET), Rotation2d.fromDegrees(0.0));

        Rotation2d rotate180 = Rotation2d.fromDegrees(180);
        Rotation2d rotate0 = Rotation2d.fromDegrees(0);

        Transform2d algaeSafe = new Transform2d(Units.inchesToMeters(ALGAE_SAFE_DISTANCE), Units.inchesToMeters(0), Rotation2d.fromDegrees(0.0));
        Transform2d algaeScore = new Transform2d(Units.inchesToMeters(ALGAE_SCORE_DISTANCE), Units.inchesToMeters(0), Rotation2d.fromDegrees(0.0));

        Transform2d coralStationPosY = new Transform2d(Units.inchesToMeters(CORAL_STATION_DISTANCE_X), Units.inchesToMeters(CORAL_STATION_DISTANCE_Y), Rotation2d.fromDegrees(0.0));
        Transform2d coralStationNegY = new Transform2d(Units.inchesToMeters(CORAL_STATION_DISTANCE_X), Units.inchesToMeters(-CORAL_STATION_DISTANCE_Y), Rotation2d.fromDegrees(0.0));
        
        //TODO: check headings
        if (alliance == Alliance.Blue) {
            // closeRightSideRightBranch = PathPointInch(159.522, 112.559, 60.0);
            Pose2d closeRightTag = getFieldLayout().getTagPose(17).get().toPose2d();
            closeRightSideRightBranch = new Pose2d(closeRightTag.transformBy(posXposBiggerYOffsets).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));
            closeRightSideLeftBranch = new Pose2d(closeRightTag.transformBy(posXnegYOffsets).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));

            Pose2d bargeRightTag = getFieldLayout().getTagPose(22).get().toPose2d();
            rightBargeSideRightBranch = new Pose2d(bargeRightTag.transformBy(posXnegYOffsets).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));
            rightBargeSideLeftBranch = new Pose2d(bargeRightTag.transformBy(posXposBiggerYOffsets).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));

            Pose2d closeTag = getFieldLayout().getTagPose(18).get().toPose2d();
            closeSideRightBranch = new Pose2d(closeTag.transformBy(posXposBiggerYOffsets).getTranslation(), closeTag.getRotation().rotateBy(rotate180));
            closeSideLeftBranch = new Pose2d(closeTag.transformBy(posXnegYOffsets).getTranslation(), closeTag.getRotation().rotateBy(rotate180));

            Pose2d bargeTag = getFieldLayout().getTagPose(21).get().toPose2d();
            bargeSideRightBranch = new Pose2d(bargeTag.transformBy(posXnegYOffsets).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));
            bargeSideLeftBranch = new Pose2d(bargeTag.transformBy(posXposBiggerYOffsets).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));

            Pose2d closeLeftTag = getFieldLayout().getTagPose(19).get().toPose2d();
            leftCloseSideLeftBranch = new Pose2d(closeLeftTag.transformBy(posXnegYOffsets).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));
            leftCloseSideRightBranch = new Pose2d(closeLeftTag.transformBy(posXposBiggerYOffsets).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));

            Pose2d leftBargeTag = getFieldLayout().getTagPose(20).get().toPose2d();
            leftBargeSideLeftBranch = new Pose2d(leftBargeTag.transformBy(posXposBiggerYOffsets).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));
            leftBargeSideRightBranch = new Pose2d(leftBargeTag.transformBy(posXnegYOffsets).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));

            Pose2d rightCoralStationTag = getFieldLayout().getTagPose(12).get().toPose2d();
            rightCoralStationOutside = new Pose2d(rightCoralStationTag.transformBy(coralStationNegY).getTranslation(), rightCoralStationTag.getRotation().rotateBy(rotate0));
            rightCoralStationInside = new Pose2d(rightCoralStationTag.transformBy(coralStationPosY).getTranslation(), rightCoralStationTag.getRotation().rotateBy(rotate0));

            Pose2d leftCoralStationTag = getFieldLayout().getTagPose(13).get().toPose2d();
            leftCoralStationOutside = new Pose2d(leftCoralStationTag.transformBy(coralStationNegY).getTranslation(), leftCoralStationTag.getRotation().rotateBy(rotate0));
            leftCoralStationInside = new Pose2d(leftCoralStationTag.transformBy(coralStationPosY).getTranslation(), leftCoralStationTag.getRotation().rotateBy(rotate0));

            rightCloseSideAlgaeSafePoint = new Pose2d(closeRightTag.transformBy(algaeSafe).getTranslation(), closeRightTag.getRotation().rotateBy(rotate0));
            rightCloseSideAlgaeScorePoint = new Pose2d(closeRightTag.transformBy(algaeScore).getTranslation(), closeRightTag.getRotation().rotateBy(rotate0));

            rightBargeSideAlgaeSafePoint = new Pose2d(bargeRightTag.transformBy(algaeSafe).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate0));
            rightBargeSideAlgaeScorePoint = new Pose2d(bargeRightTag.transformBy(algaeScore).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate0));
    
            leftCloseSideAlgaeSafePoint = new Pose2d(closeLeftTag.transformBy(algaeSafe).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate0));
            leftCloseSideAlgaeScorePoint = new Pose2d(closeLeftTag.transformBy(algaeScore).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate0));

            leftBargeSideAlgaeSafePoint = new Pose2d(leftBargeTag.transformBy(algaeSafe).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate0));
            leftBargeSideAlgaeScorePoint = new Pose2d(leftBargeTag.transformBy(algaeScore).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate0));

            bargeSideAlgaeSafePoint = new Pose2d(bargeTag.transformBy(algaeSafe).getTranslation(), bargeTag.getRotation().rotateBy(rotate0));
            bargeSideAlgaeScorePoint = new Pose2d(bargeTag.transformBy(algaeScore).getTranslation(), bargeTag.getRotation().rotateBy(rotate0));
            
            closeSideAlgaeSafePoint = new Pose2d(closeTag.transformBy(algaeSafe).getTranslation(), closeTag.getRotation().rotateBy(rotate0));
            closeSideAlgaeScorePoint = new Pose2d(closeTag.transformBy(algaeScore).getTranslation(), closeTag.getRotation().rotateBy(rotate0));
                
            double reefX = (bargeTag.getX() + closeTag.getX()) / 2.0;
            double reefY = (bargeTag.getY() + closeTag.getY()) / 2.0;
            REEF = new Pose2d(reefX, reefY, Rotation2d.fromDegrees(0.0));

            Pose2d topTagPose = getFieldLayout().getTagPose(14).get().toPose2d();
            Pose2d bottomTagPose = getFieldLayout().getTagPose(15).get().toPose2d();
            double averageX = (topTagPose.getX() + bottomTagPose.getX()) / 2.0;
            double averageY = (topTagPose.getY() + bottomTagPose.getY()) / 2.0;
            Pose2d middleFictionalTagPose = new Pose2d(averageX, averageY, Rotation2d.fromDegrees(180.0));
            
            topMostStart = new Pose2d(topTagPose.transformBy(new Transform2d(Units.inchesToMeters(37.5), 0.0, Rotation2d.fromDegrees(0.0))).getTranslation(), topTagPose.getRotation().rotateBy(rotate0));
            middleStart = new Pose2d(middleFictionalTagPose.transformBy(new Transform2d(Units.inchesToMeters(37.5), 0.0, Rotation2d.fromDegrees(0.0))).getTranslation(), topTagPose.getRotation().rotateBy(rotate0));
            bottomMostStart = new Pose2d(bottomTagPose.transformBy(new Transform2d(Units.inchesToMeters(37.5), 0.0, Rotation2d.fromDegrees(0.0))).getTranslation(), topTagPose.getRotation().rotateBy(rotate0));

            Pose2d processorTagPose = getFieldLayout().getTagPose(16).get().toPose2d();
            processor = new Pose2d(processorTagPose.getTranslation(), processorTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0)));


        } else {
            Pose2d closeRightTag = getFieldLayout().getTagPose(8).get().toPose2d();
            closeRightSideRightBranch = new Pose2d(closeRightTag.transformBy(posXposBiggerYOffsets).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));
            closeRightSideLeftBranch = new Pose2d(closeRightTag.transformBy(posXnegYOffsets).getTranslation(), closeRightTag.getRotation().rotateBy(rotate180));

            Pose2d bargeRightTag = getFieldLayout().getTagPose(9).get().toPose2d();
            rightBargeSideRightBranch = new Pose2d(bargeRightTag.transformBy(posXnegYOffsets).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));
            rightBargeSideLeftBranch = new Pose2d(bargeRightTag.transformBy(posXposBiggerYOffsets).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate180));

            Pose2d closeTag = getFieldLayout().getTagPose(7).get().toPose2d();
            closeSideRightBranch = new Pose2d(closeTag.transformBy(posXposBiggerYOffsets).getTranslation(), closeTag.getRotation().rotateBy(rotate180));
            closeSideLeftBranch = new Pose2d(closeTag.transformBy(posXnegYOffsets).getTranslation(), closeTag.getRotation().rotateBy(rotate180));

            Pose2d bargeTag = getFieldLayout().getTagPose(10).get().toPose2d();
            bargeSideRightBranch = new Pose2d(bargeTag.transformBy(posXnegYOffsets).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));
            bargeSideLeftBranch = new Pose2d(bargeTag.transformBy(posXposBiggerYOffsets).getTranslation(), bargeTag.getRotation().rotateBy(rotate180));

            Pose2d closeLeftTag = getFieldLayout().getTagPose(6).get().toPose2d();
            leftCloseSideLeftBranch = new Pose2d(closeLeftTag.transformBy(posXnegYOffsets).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));
            leftCloseSideRightBranch = new Pose2d(closeLeftTag.transformBy(posXposBiggerYOffsets).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate180));

            Pose2d leftBargeTag = getFieldLayout().getTagPose(11).get().toPose2d();
            leftBargeSideLeftBranch = new Pose2d(leftBargeTag.transformBy(posXposBiggerYOffsets).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));
            leftBargeSideRightBranch = new Pose2d(leftBargeTag.transformBy(posXnegYOffsets).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate180));
            

            Pose2d rightCoralStationTag = getFieldLayout().getTagPose(2).get().toPose2d();
            rightCoralStationOutside = new Pose2d(rightCoralStationTag.transformBy(coralStationNegY).getTranslation(), rightCoralStationTag.getRotation().rotateBy(rotate0));
            rightCoralStationInside = new Pose2d(rightCoralStationTag.transformBy(coralStationPosY).getTranslation(), rightCoralStationTag.getRotation().rotateBy(rotate0));

            Pose2d leftCoralStationTag = getFieldLayout().getTagPose(1).get().toPose2d();
            leftCoralStationOutside = new Pose2d(leftCoralStationTag.transformBy(coralStationNegY).getTranslation(), leftCoralStationTag.getRotation().rotateBy(rotate0));
            leftCoralStationInside = new Pose2d(leftCoralStationTag.transformBy(coralStationPosY).getTranslation(), leftCoralStationTag.getRotation().rotateBy(rotate0));

            rightCloseSideAlgaeSafePoint = new Pose2d(closeRightTag.transformBy(algaeSafe).getTranslation(), closeRightTag.getRotation().rotateBy(rotate0));
            rightCloseSideAlgaeScorePoint = new Pose2d(closeRightTag.transformBy(algaeScore).getTranslation(), closeRightTag.getRotation().rotateBy(rotate0));

            rightBargeSideAlgaeSafePoint = new Pose2d(bargeRightTag.transformBy(algaeSafe).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate0));
            rightBargeSideAlgaeScorePoint = new Pose2d(bargeRightTag.transformBy(algaeScore).getTranslation(), bargeRightTag.getRotation().rotateBy(rotate0));
    
            leftCloseSideAlgaeSafePoint = new Pose2d(closeLeftTag.transformBy(algaeSafe).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate0));
            leftCloseSideAlgaeScorePoint = new Pose2d(closeLeftTag.transformBy(algaeScore).getTranslation(), closeLeftTag.getRotation().rotateBy(rotate0));

            leftBargeSideAlgaeSafePoint = new Pose2d(leftBargeTag.transformBy(algaeSafe).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate0));
            leftBargeSideAlgaeScorePoint = new Pose2d(leftBargeTag.transformBy(algaeScore).getTranslation(), leftBargeTag.getRotation().rotateBy(rotate0));

            bargeSideAlgaeSafePoint = new Pose2d(bargeTag.transformBy(algaeSafe).getTranslation(), bargeTag.getRotation().rotateBy(rotate0));
            bargeSideAlgaeScorePoint = new Pose2d(bargeTag.transformBy(algaeScore).getTranslation(), bargeTag.getRotation().rotateBy(rotate0));
            
            closeSideAlgaeSafePoint = new Pose2d(closeTag.transformBy(algaeSafe).getTranslation(), closeTag.getRotation().rotateBy(rotate0));
            closeSideAlgaeScorePoint = new Pose2d(closeTag.transformBy(algaeScore).getTranslation(), closeTag.getRotation().rotateBy(rotate0)); 
            
            double reefX = (bargeTag.getX() + closeTag.getX()) / 2.0;
            double reefY = (bargeTag.getY() + closeTag.getY()) / 2.0;
            REEF = new Pose2d(reefX, reefY, Rotation2d.fromDegrees(0.0));

            Pose2d topTagPose = getFieldLayout().getTagPose(4).get().toPose2d();
            Pose2d bottomTagPose = getFieldLayout().getTagPose(5).get().toPose2d();
            double averageX = (topTagPose.getX() + bottomTagPose.getX()) / 2.0;
            double averageY = (topTagPose.getY() + bottomTagPose.getY()) / 2.0;
            Pose2d middleFictionalTagPose = new Pose2d(averageX, averageY, Rotation2d.fromDegrees(0.0));
            
            topMostStart = new Pose2d(topTagPose.transformBy(new Transform2d(Units.inchesToMeters(37.5), 0.0, Rotation2d.fromDegrees(0.0))).getTranslation(), topTagPose.getRotation().rotateBy(rotate0));
            middleStart = new Pose2d(middleFictionalTagPose.transformBy(new Transform2d(Units.inchesToMeters(37.5), 0.0, Rotation2d.fromDegrees(0.0))).getTranslation(), topTagPose.getRotation().rotateBy(rotate0));
            bottomMostStart = new Pose2d(bottomTagPose.transformBy(new Transform2d(Units.inchesToMeters(37.5), 0.0, Rotation2d.fromDegrees(0.0))).getTranslation(), topTagPose.getRotation().rotateBy(rotate0));

            Pose2d processorTagPose = getFieldLayout().getTagPose(3).get().toPose2d();
            processor = new Pose2d(processorTagPose.getTranslation(), processorTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0)));
        }

    }

    /** Convenience method to create PathPoint from inches */
    private static Pose2d PathPointInch(double xInches, double yInches, double headingDegrees) {
        return new Pose2d(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)),
                Rotation2d.fromDegrees(headingDegrees));
    }
}
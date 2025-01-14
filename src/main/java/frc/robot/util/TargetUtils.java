package frc.robot.util;

import java.util.Optional;

import javax.lang.model.util.ElementScanner14;
import javax.management.openmbean.OpenType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.Constants.APRILTAGS;
import frc.robot.commands.Autons;
import frc.robot.commands.Autons.AutonTypes;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;

public class TargetUtils {

    public static RobotZone robotZone = RobotZone.LEFT;

    public static double getDistanceToFieldPos(Pose2d robotPose, int apriltag) {
        double distance = 0.0;
        Optional<Pose3d> tagPose = KnownLocations.getFieldLayout().getTagPose(apriltag);

        if (tagPose.isPresent()) {
            distance = robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
        }

        return distance;
    }

    public static double getDistanceToSpeaker(Pose2d robotPose) {
        Alliance alliance = KnownLocations.getKnownLocations().alliance;
        if (alliance == Alliance.Blue) {
            return getDistanceToFieldPos(robotPose, APRILTAGS.MIDDLE_BLUE_SPEAKER);
        } else {
            return getDistanceToFieldPos(robotPose, APRILTAGS.MIDDLE_RED_SPEAKER);
        }
    }

    // Following is based off https://www.chiefdelphi.com/t/is-there-a-builtin-function-to-find-the-angle-needed-to-get-one-pose2d-to-face-another-pose2d/455972
    public static double getTargetHeadingToAprilTag(Pose2d robotPose, int tagId) {
        double heading = 0.0;
        Optional<Pose3d> tagPose = KnownLocations.getFieldLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
            Translation2d tagPoint = tagPose.get().getTranslation().toTranslation2d();
            Rotation2d targetRotation = tagPoint.minus(robotPose.getTranslation()).getAngle();
            heading = targetRotation.rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
        }
        return heading;
    }

    public static double getTargetHeadingToPoint(Pose2d robotPose, Translation2d point) {
        Rotation2d targetRotation = point.minus(robotPose.getTranslation()).getAngle();
        return  targetRotation.rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
    }

    public static double getTargetHeadingToReef(Pose2d robotPose) {
        double rawHeading = getTargetHeadingToPoint(robotPose, KnownLocations.REEF.getTranslation());
        double finalHeading = 0.0;

        if (rawHeading >= -30.0 && rawHeading <= 30.0) {
            finalHeading = 0.0;
            robotZone = RobotZone.LEFT;
        } else if (rawHeading > 30.0 && rawHeading <= 90.0) {
            finalHeading = 60.0;
            robotZone = RobotZone.BOTTOM_LEFT;
        } else if (rawHeading > 90.0 && rawHeading <= 150.0) {
            finalHeading = 120.0;
            robotZone = RobotZone.BOTTOM_RIGHT;
        } else if (rawHeading > 150.0 && rawHeading <= 180.0) { 
            finalHeading = 180.0;
            robotZone = RobotZone.RIGHT;
        } else if (rawHeading >= -180.0 && rawHeading < -150.0) { 
            finalHeading = 180.0;
            robotZone = RobotZone.RIGHT;
        } else if (rawHeading >= -150.0 && rawHeading < -90.0) {
            finalHeading = -120.0;
            robotZone = RobotZone.TOP_RIGHT;
        } else if (rawHeading >= -90.0 && rawHeading < -30.0) {
            finalHeading = -60.0;
            robotZone = RobotZone.TOP_LEFT;
        }        

        return finalHeading;
    }

    public static Pose2d getLeftBranch() {
        switch (robotZone) {
            case RIGHT:
                return KnownLocations.leftBranchInRightZone;
            case BOTTOM_RIGHT:
                return KnownLocations.leftBranchInBottomRightZone;
            case BOTTOM_LEFT:
                return KnownLocations.leftBranchInBottomLeftZone;
            case LEFT:
                return KnownLocations.leftBranchInLeftZone;
            case TOP_LEFT:
                return KnownLocations.leftBranchInTopLeftZone;
            case TOP_RIGHT:
                return KnownLocations.leftBranchInTopRightZone;        
        }
        return new Pose2d(); //should not get here
    }

    public static Pose2d getRightBranch() {
        switch (robotZone) {
            case RIGHT:
                return KnownLocations.rightBranchInRightZone;
            case BOTTOM_RIGHT:
                return KnownLocations.rightBranchInBottomRightZone;
            case BOTTOM_LEFT:
                return KnownLocations.rightBranchInBottomLeftZone;
            case LEFT:
                return KnownLocations.rightBranchInLeftZone;
            case TOP_LEFT:
                return KnownLocations.rightBranchInTopLeftZone;
            case TOP_RIGHT:
                return KnownLocations.rightBranchInTopRightZone;        
        }
        return new Pose2d(); //should not get here
    }

    public enum RobotZone {
        LEFT("left"),
        BOTTOM_LEFT("bottomLeft"),
        BOTTOM_RIGHT("bottomRight"),
        TOP_LEFT("topLeft"),
        TOP_RIGHT("topRight"),
        RIGHT("right");
      
        public final String robotZone;
          RobotZone(String zone) {
            this.robotZone = zone;
          }
    }
}



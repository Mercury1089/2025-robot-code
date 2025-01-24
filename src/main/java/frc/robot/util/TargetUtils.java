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
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.FieldPosition;

public class TargetUtils {


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

    public static Rotation2d getTargetHeadingToPoint(Pose2d robotPose, Translation2d point) {
        Rotation2d targetRotation = point.minus(robotPose.getTranslation()).getAngle();
        return  targetRotation.rotateBy(Rotation2d.fromDegrees(180.0));
    }
}



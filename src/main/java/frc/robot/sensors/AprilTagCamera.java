// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.KnownLocations;

/** Wrapper for PhotonCamera class */
public class AprilTagCamera extends PhotonCamera {

    // AprilTagCamera 3d Pose on robot
    // Uses coordinates described here: https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html#camera-coordinate-frame
    // private static final String DEFAULT_CAM_NAME = "AprilTagCamera";
    // private static final double DEFAULT_CAM_X = Units.inchesToMeters(13.5); // 14.75in behind center
    // private static final double DEFAULT_CAM_Y = 0.0; // centered in robot Y
    // private static final double DEFAULT_CAM_Z = Units.inchesToMeters(6.5); // 21.25in up from center

    // private static final double DEFAULT_CAM_ROTATION = Rotation2d.fromDegrees(0).getRadians(); // rotation relative to robot front (radians)
    //private static final double DEFAULT_CAM_TILT = Rotation2d.fromDegrees(20).getRadians(); // tilt relative to floor (raians)

    // private static final double TARGET_HEIGHT = 0.36; // may need to change - DO WE NEED THIS?
    // private static final double CAMERA_HEIGHT = DEFAULT_CAM_Z; // height on robot (meters) - DO WE NEED THIS?


    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator estimator;

    public AprilTagCamera(String name, Transform3d robotToCam) {
        super(name);
        fieldLayout = KnownLocations.getFieldLayout();
        // Transform3d robotToCam = new Transform3d(
        //     new Translation3d(DEFAULT_CAM_X, DEFAULT_CAM_Y, DEFAULT_CAM_Z), new Rotation3d(0.0, DEFAULT_CAM_TILT, DEFAULT_CAM_ROTATION)
        // );
        // Uncomment the following to silence missing camera errors
        // PhotonCamera.setVersionCheckEnabled(false);
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }

    public Optional<EstimatedRobotPose> getGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : getAllUnreadResults()) {
            visionEst = estimator.update(change);
            // TODO: See example: https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
            //updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        return getLatestResult().getBestTarget().getYaw();
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        return getLatestResult().getBestTarget().getPitch();
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        return getLatestResult().getBestTarget().getSkew();
    }

    public double getApriltagID() {
        return getLatestResult().getBestTarget().getFiducialId();
    }
}


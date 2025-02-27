// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.opencv.core.RotatedRect;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.path.PathPlannerPath;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.sensors.DistanceSensors;
import frc.robot.sensors.ProximitySensor;
import frc.robot.util.KnownLocations;
import frc.robot.util.PathUtils;
import frc.robot.util.SwerveUtils;
import frc.robot.util.TargetUtils;
import frc.robot.util.ReefscapeUtils;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class Drivetrain extends SubsystemBase {

  private MAXSwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  private Pigeon2 pigeon;
  private SwerveDrivePoseEstimator odometry;
  private SwerveDriveKinematics swerveKinematics;
  private AprilTagCamera photonCam, photonCam2;
  private Field2d smartdashField;
  private ProfiledPIDController rotationPIDController, xPIDController, yPIDController;
  private Pose2d startingPosition;

  private static final double ROTATION_P = 1.0 / 90.0, DIRECTION_P = 1 / 1.125, I = 0.0, D = 0.0;
  private final double THRESHOLD_DEGREES = 3.0;
  private final double THRESHOLD_SPEED = 0.5;

  private double targetHeadingToReef = 0.0;
  private double targetHeadingToStation = 0.0;

  private Transform3d frontCam = new Transform3d(
    new Translation3d(Units.inchesToMeters(13.5), Units.inchesToMeters(0.0), Units.inchesToMeters(6.5)), 
    new Rotation3d(0.0, Rotation2d.fromDegrees(20).getRadians(), Rotation2d.fromDegrees(0).getRadians())
  );
    
  private Transform3d backCam = new Transform3d(
    new Translation3d(Units.inchesToMeters(9.5), Units.inchesToMeters(0.0), Units.inchesToMeters(18.25)), 
    new Rotation3d(0.0, Rotation2d.fromDegrees(20).getRadians(), Rotation2d.fromDegrees(180).getRadians())
  );

  // 2024 - Autotune
  //private final double WHEEL_WIDTH = 23.5; // distance between front/back wheels (in inches)
  //private final double WHEEL_LENGTH = 28.5; // distance between left/right wheels (in inches)

  // // distance between wheels
  private final double WHEEL_WIDTH = 23.5; // distance between front/back wheels (in inches)
  private final double WHEEL_LENGTH = 23.5; // distance between left/right wheels (in inches)

  private Rotation2d gyroOffset = Rotation2d.fromDegrees(0); // Offset to apply to gyro for field oriented

  // Slew rate filter variables for controlling lateral acceleration
  private double currentAngularSpeed = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(SWERVE.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter angularSpeedLimiter = new SlewRateLimiter(SWERVE.ROTATIONAL_SLEW_RATE);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  private double laserCanMeasurement = 0.0;
  private double laserCanMeasurement2 = 0.0;
  private boolean ignoreBackCamera = false;
  private DistanceSensors leftSensors, rightSensors, backSensor;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // configure swerve modules
    /**
    * Configuring Driving and Turning motors for each Swerve Module
    */
    frontLeftModule = new MAXSwerveModule(CAN.DRIVING_FRONT_LEFT, CAN.TURNING_FRONT_LEFT, -Math.PI / 2);
    frontRightModule = new MAXSwerveModule(CAN.DRIVING_FRONT_RIGHT, CAN.TURNING_FRONT_RIGHT, 0);
    backLeftModule = new MAXSwerveModule(CAN.DRIVING_BACK_LEFT, CAN.TURNING_BACK_LEFT, Math.PI);
    backRightModule = new MAXSwerveModule(CAN.DRIVING_BACK_RIGHT, CAN.TURNING_BACK_RIGHT, Math.PI / 2);

    leftSensors = new DistanceSensors(CAN.LEFT_INNER_LASER_CAN, CAN.LEFT_OUTER_LASER_CAN);
    rightSensors = new DistanceSensors(CAN.RIGHT_INNER_LASER_CAN, CAN.RIGHT_OUTER_LASER_CAN);
    backSensor = new DistanceSensors(CAN.BACK_LASER_CAN, 135.0); // check this error

    //configure gyro
    pigeon = new Pigeon2(CAN.PIGEON_DRIVETRAIN);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getYaw().setUpdateFrequency(10);

    rotationPIDController = new ProfiledPIDController(ROTATION_P, I, D, new TrapezoidProfile.Constraints(Units.radiansToDegrees(4.75),720));
    rotationPIDController.enableContinuousInput(-180, 180);
    rotationPIDController.setTolerance(1.0);

    xPIDController = new ProfiledPIDController(DIRECTION_P, I, D, new TrapezoidProfile.Constraints(SWERVE.MAX_DIRECTION_SPEED,SWERVE.MAX_ACCELERATION));
    xPIDController.setTolerance(0.0254);
    
    yPIDController = new ProfiledPIDController(DIRECTION_P, I, D, new TrapezoidProfile.Constraints(SWERVE.MAX_DIRECTION_SPEED,SWERVE.MAX_ACCELERATION));
    yPIDController.setTolerance(0.0254);

    startingPosition = new Pose2d();

    // photonvision wrapper
    photonCam = new AprilTagCamera("AprilTagCamera" , frontCam);

    photonCam2 = new AprilTagCamera("BackTagCamera" , backCam);


    smartdashField = new Field2d();
    SmartDashboard.putData("Swerve Odometry", smartdashField);

    /*
    * swerve modules relative to robot center --> kinematics object --> odometry object 
    */

    double widthFromCenter = Units.inchesToMeters(WHEEL_WIDTH) / 2;
    double lengthFromCenter = Units.inchesToMeters(WHEEL_LENGTH) / 2;

    swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(lengthFromCenter, widthFromCenter),
      new Translation2d(lengthFromCenter, -widthFromCenter),
      new Translation2d(-lengthFromCenter, widthFromCenter),
      new Translation2d(-lengthFromCenter, -widthFromCenter)
    );

    // Initialize the robot odometry to the the field origin.
    // This will be updated by the selected Auton and DriveTrain.periodic()
    odometry = new SwerveDrivePoseEstimator(
      swerveKinematics, 
      getRotation(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      },
      new Pose2d(0, 0, getRotation())
    );
  }

  public ProfiledPIDController getRotationalController() {
    return rotationPIDController;
  }

  public ProfiledPIDController getXController() {
    return xPIDController;
  }

  public ProfiledPIDController getYController() {
    return yPIDController;
  }

  public DistanceSensors getLeftSensors() {
    return leftSensors;
  }

  public DistanceSensors getRightSensors() {
    return rightSensors;
  }

  public DistanceSensors getBackSensor() {
    return backSensor;
  }

  public void setStartingPosition(Pose2d startPos) {
    this.startingPosition = startPos;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backRightModule.resetEncoders();
  }
  /**
  * Resets the Gyro of robot, or facing to 0
  */
  public void resetGyro() {
    pigeon.reset();
    gyroOffset = Rotation2d.fromDegrees(0.0);
  }
  /**
  * @param : Double as (xSpeed, ySpeed, angularSpeed)
  * @return : Assigning values to drive
  */
  public void drive(double xSpeed, double ySpeed, double angularSpeed) {
    drive(xSpeed, ySpeed, angularSpeed, true);
  }
  /**
  * @param : Double as (xSpeed, ySpeed, angularSpeed), fieldRelative
  * @return : Assigning Values to drive
  */
  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative) {
    drive(xSpeed, ySpeed, angularSpeed, fieldRelative, false);
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative, boolean rateLimit) {
    drive(xSpeed, ySpeed, angularSpeed, fieldRelative, rateLimit, () -> this.getRotation(this.gyroOffset));
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative, boolean rateLimit, Supplier<Rotation2d> rotationSupplier) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(SWERVE.DIRECTION_SLEW_RATE / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
      
      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentAngularSpeed = angularSpeedLimiter.calculate(angularSpeed);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentAngularSpeed = angularSpeed;
    }

    double xSpeedDelivered = xSpeedCommanded * SWERVE.MAX_DIRECTION_SPEED;
    double ySpeedDelivered = ySpeedCommanded * SWERVE.MAX_DIRECTION_SPEED;
    double angularSpeedDelivered = currentAngularSpeed * SWERVE.MAX_ROTATIONAL_SPEED;

    ChassisSpeeds fieldRelativeSpeeds;

    if (fieldRelative) {
      fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, angularSpeedDelivered, rotationSupplier.get());
    } else {
      fieldRelativeSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, angularSpeedDelivered);
    }
    
    drive(fieldRelativeSpeeds);
  }

  public void drive(ChassisSpeeds fieldRelativeSpeeds) {
    // general swerve speeds --> speed per module
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(fieldRelativeSpeeds);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SWERVE.MAX_DIRECTION_SPEED);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  public void lockSwerve() {
    // set wheels into X formation
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(- Math.PI / 4)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
  }

  /** update smartdash with trajectory */
  public void setTrajectorySmartdash(Trajectory trajectory, String type) {
    smartdashField.getObject(type).setTrajectory(trajectory);
  }

  public void setPoseSmartdash(Pose2d pose, String type) {
    smartdashField.getObject(type).setPose(pose);
  }
  
  /**
   * Set the odometry object to a predetermined pose
   * No need to reset gyro as it auto-applies offset
   * 
   * Used to set initial pose from an auton trajectory
   */
  public void resetPose(Pose2d pose) {
    // Set gyro offset for field orieneted rotatioon (zero faces away from alliance station wall)
    // gyroOffset = KnownLocations.getKnownLocations().ZERO_GYRO_ROTAION.plus(pose.getRotation()); *TODO: ADD THIS OFFSET BACK*

    odometry.resetPosition(
    getRotation(), 
    new SwerveModulePosition[] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      backRightModule.getPosition()
      },
    pose
    );
  }

  public ChassisSpeeds getFieldRelativSpeeds() {
    return swerveKinematics.toChassisSpeeds(new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    });
  }

  // meters/second
  //up is pos
  public double getXSpeeds() {
    return getFieldRelativSpeeds().vxMetersPerSecond;
  }

  // left is pos
  public double getYSpeeds() {
    return getFieldRelativSpeeds().vyMetersPerSecond;
  }

  /**
   * Get the robot relative rotation reported by the gyro (pigeon). Remember that this should be CCW positive.
   * @return The rotation as read from the gyro.
   */
  public Rotation2d getRotation() {
    // Note: Unlike getAngle(), getRotation2d is CCW positive.
    return pigeon.getRotation2d();
  }

  /**
   * Get the robot relative rotation reported by the gyro with an applied offset.
   * @param offset Offset to add to the gyro-supplied rotation.
   */
  public Rotation2d getRotation(Rotation2d offset) {
    return getRotation().plus(offset);
  }

  public AprilTagCamera getAprilTagCamera() {
    return this.photonCam;
  }

  public double getTargetHeadingToReef() {
    return targetHeadingToReef;
  }

  public double getTargetHeadingToStation() {
    return targetHeadingToStation;
  }

  public boolean isTargetPresent() {
    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    return result.isPresent();
  }

  public boolean isNotMoving() {
    return Math.abs(getXSpeeds()) < THRESHOLD_SPEED && Math.abs(getYSpeeds()) < THRESHOLD_SPEED;
  }

  public boolean isAtPose(Pose2d target) {
    return isAtPose(target, getXController().getPositionTolerance(), getYController().getPositionTolerance());
  }
  
  public boolean isAtPose(Pose2d target, double xTolerance, double yTolerance) {
    return Math.abs(target.getX() - getPose().getX()) < xTolerance
        && Math.abs(target.getY() - getPose().getY()) < yTolerance;
  }

  public boolean isAtPose(Pose2d target, double tolerance) {
    return isAtPose(target, tolerance, tolerance);
  }


  public boolean isAtPreferredCoralStation() {
    return isAtPose(ReefscapeUtils.getPreferredCoralStation());
  }

  public boolean isAtPreferredBranch() {
    return isAtPose(ReefscapeUtils.getPreferredBranch());
  }

  public double getMinimumAbiguity(List<PhotonTrackedTarget> targetsUsed) {
    double min = 1.0;
      
    for (PhotonTrackedTarget photonTrackedTarget : targetsUsed) {
      min = Math.min(min, photonTrackedTarget.getPoseAmbiguity());
    }
      
    return min;
  }

  public void setIgnoreBackCam(boolean ignore){
    ignoreBackCamera = ignore;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      getRotation(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    });
    
    Optional<EstimatedRobotPose> result = photonCam.getGlobalPose();
    if (result.isPresent() && !photonCam.rejectUpdate(result.get())) {
      // Uncomment the following to check camera position on robot
      // Pose3d estimatedPose = result.get().estimatedPose;
      // SmartDashboard.putNumber("Cam/Yaw", estimatedPose.getRotation().getZ());
      // SmartDashboard.putNumber("Cam/Pitch", estimatedPose.getRotation().getY());
      // SmartDashboard.putNumber("Cam/Roll", estimatedPose.getRotation().getX());
      odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
    }

    Optional<EstimatedRobotPose> backResult = photonCam2.getGlobalPose();
    if (backResult.isPresent() && !ignoreBackCamera && !photonCam2.rejectUpdate(backResult.get())){
      // Uncomment the following to check camera position on robot
      // Pose3d estimatedPose = result.get().estimatedPose;
      // SmartDashboard.putNumber("Cam/Yaw", estimatedPose.getRotation().getZ());
      // SmartDashboard.putNumber("Cam/Pitch", estimatedPose.getRotation().getY());
      // SmartDashboard.putNumber("Cam/Roll", estimatedPose.getRotation().getX());
      odometry.addVisionMeasurement(backResult.get().estimatedPose.toPose2d(), backResult.get().timestampSeconds);
    }

    smartdashField.setRobotPose(getPose());

  
    // KnownLocations knownLocations = KnownLocations.getKnownLocations();
    // pathToAmp = PathUtils.generatePath(Rotation2d.fromDegrees(-90.0), getPose(), knownLocations.AMP);
    // setTrajectorySmartdash(PathUtils.TrajectoryFromPath(pathToAmp), "pathToAmp");

    targetHeadingToReef = ReefscapeUtils.getTargetHeadingToReef(getPose());
    targetHeadingToStation = ReefscapeUtils.getTargetHeadingToStation(getPose());

    SmartDashboard.putNumber("Drivetrain/CurrentPose X", getPose().getX());
    SmartDashboard.putNumber("Drivetrain/CurrentPose Y", getPose().getY());
    SmartDashboard.putNumber("Drivetrain/getRotation", getRotation().getDegrees());
    SmartDashboard.putBoolean("Drivetrain/isNotMoving", isNotMoving());
    SmartDashboard.putNumber("Drivetrain/CurrentPose Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Drivetrain/Drive Angle", getRotation().getDegrees());
    SmartDashboard.putString("Drivetrain/robotZone", ReefscapeUtils.getCurrentRobotZone().robotZone);
    SmartDashboard.putString("Drivetrain/preferredZone", ReefscapeUtils.preferredZone().robotZone);
    SmartDashboard.putString("Drivetrain/branchSide", ReefscapeUtils.branchSide().side);
    SmartDashboard.putString("Drivetrain/coralStation", ReefscapeUtils.preferredCoralStation().coralStation);
    SmartDashboard.putBoolean("Drivetrain/isAtPreferredStation", isAtPreferredCoralStation());
    SmartDashboard.putBoolean("Drivetrain/isAtPreferredBranch", isAtPreferredBranch());
    SmartDashboard.putString("Drivetrain/preferredLevel", ReefscapeUtils.getPreferredLevel().lev);
    SmartDashboard.putNumber("Drivetrain/leftInner", leftSensors.getSensorDistance(leftSensors.getInnerSensor()));
    SmartDashboard.putNumber("Drivetrain/leftOuter", leftSensors.getSensorDistance(leftSensors.getOuterSensor()));
    SmartDashboard.putBoolean("Drivetrain/isAlignedWithSensorsLEFT", leftSensors.isAtReefSide());
    SmartDashboard.putBoolean("Drivetrain/isAtScoreCoralPoint",isAtPose(ReefscapeUtils.getCurrentZoneScoreAlgaePoint()));
    SmartDashboard.putNumber("Drivetrain/rightInner", rightSensors.getSensorDistance(rightSensors.getInnerSensor()));
    SmartDashboard.putNumber("Drivetrain/rightOuter", rightSensors.getSensorDistance(rightSensors.getOuterSensor()));
    SmartDashboard.putNumber("Drivetrain/rotationToStation", ReefscapeUtils.getTargetHeadingToStation(getPose()));
    SmartDashboard.putBoolean("Drivetrain/isAtStartingPosition", isAtPose(startingPosition, Units.inchesToMeters(2.0), Units.inchesToMeters(2.0)));
  }
}

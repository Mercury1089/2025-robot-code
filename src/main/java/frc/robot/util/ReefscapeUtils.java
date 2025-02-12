package frc.robot.util;

import java.nio.file.Path;
import java.util.Currency;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class ReefscapeUtils {
    private static RobotZone robotZone = RobotZone.LEFT; // this is your current robotZone

    private static RobotZone preferredZone = RobotZone.LEFT; // this is your intended robot zone to go to while in FIDO
    private static CoralStation preferredCoralStation = CoralStation.INSIDERIGHT;
    private static ElevatorPosition preferredLevel = ElevatorPosition.LEVEL1;
    private static BranchSide preferredBranchSide = BranchSide.LEFT;

    
    public static RobotZone preferredZone() {
        return preferredZone;
    }

    public static CoralStation preferredCoralStation() {
        return preferredCoralStation;
    }

    
    public static BranchSide branchSide() {
        return preferredBranchSide;
    }

    public static Pose2d getCurrentZoneLeftBranch() {
        return getLeftBranch(robotZone);
    }

    public static Pose2d getCurrentZoneRightBranch() {
        return getRightBranch(robotZone);
    }

    public static Pose2d getPreferredBranch() {
       if (preferredBranchSide == BranchSide.LEFT) {
            return getLeftBranch(preferredZone);
       } else {
            return getRightBranch(preferredZone);
       }
    }

    public static ElevatorPosition getPreferredLevel() {
        return preferredLevel;
    }

    public static Pose2d getPreferredCoralStation() {
        Pose2d stationPose2d = new Pose2d();

        switch (preferredCoralStation) {
            case INSIDELEFT:
                stationPose2d = KnownLocations.leftCoralStationInside;
                break;
            case OUTSIDELEFT:
                stationPose2d = KnownLocations.leftCoralStationOutside;
                break;
            case INSIDERIGHT:
                stationPose2d = KnownLocations.rightCoralStationInside;
                break;
            case OUTSIDERIGHT:
                stationPose2d = KnownLocations.rightCoralStationOutside;
                break;
        }

        return stationPose2d;
    }

    public static Pose2d getpreferredZone() {
        Pose2d zone = new Pose2d();

        switch (preferredZone) {
            case RIGHT:
                zone = KnownLocations.rightZone;
                break;
            case BOTTOM_RIGHT:
                zone = KnownLocations.bottomRightZone;
                break;
            case BOTTOM_LEFT:
                zone = KnownLocations.bottomLeftZone;
                break;
            case LEFT:
                zone = KnownLocations.leftZone;
                break;
            case TOP_LEFT:
                zone = KnownLocations.topLeftZone;
                break;
            case TOP_RIGHT:
                zone = KnownLocations.topRightZone;
                break;    
        }

        return zone;
    }

    public static Pose2d getCurrentZoneSafeAlgaePoint() {
        Pose2d result;
        switch (robotZone) {
            case LEFT:
                result = KnownLocations.leftZoneAlgaeSafePoint;
                break;
            case BOTTOM_LEFT:
                result = KnownLocations.bottomLeftZoneAlgaeSafePoint;
                break;
            case BOTTOM_RIGHT:
                result = KnownLocations.bottomRightZoneAlgaeSafePoint;
                break;
            case RIGHT:
                result = KnownLocations.rightZoneAlgaeSafePoint;
                break;
            case TOP_RIGHT:
                result = KnownLocations.topRightZoneAlgaeSafePoint;
                break;
            case TOP_LEFT:
                result = KnownLocations.topLeftZoneAlgaeSafePoint;
                break;
            default:
                result = new Pose2d();
                break;
        }
        return result;
    }

    public static Pose2d getCurrentZoneScoreAlgaePoint() {
        Pose2d result;
        switch (robotZone) {
            case LEFT:
                result = KnownLocations.leftZoneAlgaeScorePoint;
                break;
            case BOTTOM_LEFT:
                result = KnownLocations.bottomLeftZoneAlgaeScorePoint;
                break;
            case BOTTOM_RIGHT:
                result = KnownLocations.bottomRightZoneAlgaeScorePoint;
                break;
            case RIGHT:
                result = KnownLocations.rightZoneAlgaeScorePoint;
                break;
            case TOP_RIGHT:
                result = KnownLocations.topRightZoneAlgaeScorePoint;
                break;
            case TOP_LEFT:
                result = KnownLocations.topLeftZoneAlgaeScorePoint;
                break;
            default:
                result = new Pose2d();
                break;
        }
        return result;
    }

    public static int getCurrentRobotZoneAprilTag() {
        int tag = 0;

        if (KnownLocations.getKnownLocations().alliance == Alliance.Blue) {
            switch (robotZone) {
                case LEFT:
                    tag = 18;
                    break;
                case BOTTOM_LEFT:
                    tag = 17;
                    break;
                case BOTTOM_RIGHT:
                    tag = 22;
                    break;
                case RIGHT:
                    tag = 21;
                    break;
                case TOP_RIGHT:
                    tag = 20;
                    break;
                case TOP_LEFT:
                    tag = 19;
                    break;
                default:
                    break;
            }
        } else {
            switch (robotZone) {
                case LEFT:
                    tag = 10;
                    break;
                case BOTTOM_LEFT:
                    tag = 11;
                    break;
                case BOTTOM_RIGHT:
                    tag = 6;
                    break;
                case RIGHT:
                    tag = 7;
                    break;
                case TOP_RIGHT:
                    tag = 8;
                    break;
                case TOP_LEFT:
                    tag = 9;
                    break;
                default:
                    break;
            }
        }

        return tag;
    }
    /**
    * @return : Returns a ComditionalCommand
    */
    public static Command getPathToPreferredBranch() {
            Supplier<Double> goalEndSupplier = () -> 0.5;
    
            return new ConditionalCommand(
                PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.LEFT), goalEndSupplier), 
                new ConditionalCommand(
                    PathUtils.getPathToPose(() -> getRightBranch(RobotZone.LEFT), goalEndSupplier),
                    new ConditionalCommand(
                        PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.BOTTOM_LEFT), goalEndSupplier),
                        new ConditionalCommand(
                            PathUtils.getPathToPose(() -> getRightBranch(RobotZone.BOTTOM_LEFT), goalEndSupplier),
                            new ConditionalCommand(
                                PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.BOTTOM_RIGHT), goalEndSupplier),
                                new ConditionalCommand(
                                    PathUtils.getPathToPose(() -> getRightBranch(RobotZone.BOTTOM_RIGHT), goalEndSupplier),
                                    new ConditionalCommand(
                                        PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.RIGHT), goalEndSupplier),
                                        new ConditionalCommand(
                                            PathUtils.getPathToPose(() -> getRightBranch(RobotZone.RIGHT), goalEndSupplier),
                                            new ConditionalCommand(
                                                PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.TOP_RIGHT), goalEndSupplier),
                                                new ConditionalCommand(
                                                    PathUtils.getPathToPose(() -> getRightBranch(RobotZone.TOP_RIGHT), goalEndSupplier),
                                                    new ConditionalCommand(
                                                        PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.TOP_LEFT), goalEndSupplier),
                                                        new ConditionalCommand(
                                                            PathUtils.getPathToPose(() -> getRightBranch(RobotZone.TOP_LEFT), goalEndSupplier),
                                                            PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.LEFT), goalEndSupplier), 
                                                            () -> preferredZone == RobotZone.TOP_LEFT && preferredBranchSide == BranchSide.RIGHT),
                                                        () -> preferredZone == RobotZone.TOP_LEFT && preferredBranchSide == BranchSide.LEFT),
                                                    () -> preferredZone == RobotZone.TOP_RIGHT && preferredBranchSide == BranchSide.RIGHT),
                                                () -> preferredZone == RobotZone.TOP_RIGHT && preferredBranchSide == BranchSide.LEFT),
                                            () -> preferredZone == RobotZone.RIGHT && preferredBranchSide == BranchSide.RIGHT),
                                        () -> preferredZone == RobotZone.RIGHT && preferredBranchSide == BranchSide.LEFT),
                                    () -> preferredZone == RobotZone.BOTTOM_RIGHT && preferredBranchSide == BranchSide.RIGHT),
                                () -> preferredZone == RobotZone.BOTTOM_RIGHT && preferredBranchSide == BranchSide.LEFT),
                            () -> preferredZone == RobotZone.BOTTOM_LEFT && preferredBranchSide == BranchSide.RIGHT),
                        () -> preferredZone == RobotZone.BOTTOM_LEFT && preferredBranchSide == BranchSide.LEFT),
                    () -> preferredZone == RobotZone.LEFT && preferredBranchSide == BranchSide.RIGHT), 
                () -> preferredZone == RobotZone.LEFT && preferredBranchSide == BranchSide.LEFT);
    }

    public static Command getPathToPreferredCoralStation() {
        Supplier<Double> goalEndSupplier = () -> 0.5;
        return new ConditionalCommand(
            PathUtils.getPathToPose(() -> KnownLocations.leftCoralStationInside, goalEndSupplier), 
            new ConditionalCommand(
                PathUtils.getPathToPose(() -> KnownLocations.leftCoralStationOutside, goalEndSupplier), 
                new ConditionalCommand(
                    PathUtils.getPathToPose(() -> KnownLocations.rightCoralStationInside, goalEndSupplier), 
                    new ConditionalCommand(
                        PathUtils.getPathToPose(() -> KnownLocations.rightCoralStationOutside, goalEndSupplier), 
                            PathUtils.getPathToPose(() -> KnownLocations.rightCoralStationInside, goalEndSupplier), 
                        () -> preferredCoralStation == CoralStation.OUTSIDERIGHT),
                    () -> preferredCoralStation == CoralStation.INSIDERIGHT),
                () -> preferredCoralStation == CoralStation.OUTSIDELEFT), 
            () -> preferredCoralStation == CoralStation.INSIDELEFT);
    }

    public static void changepreferredBranch(BranchSide side) {
        preferredBranchSide = side;
    }

    public static void changepreferredLevel(ElevatorPosition level) {
        preferredLevel = level;
    }

    public static void changepreferredCoralStation(CoralStation station) {
        preferredCoralStation = station;
    }

    public static void changepreferredZone(RobotZone zone) {
        preferredZone = zone;
    }
    /**
    * @param : robotPose
    * @return : finalHeading
    */
    public static double getTargetHeadingToReef(Pose2d robotPose) {
        double rawHeading = TargetUtils.getTargetHeadingToPoint(robotPose, KnownLocations.REEF.getTranslation()).rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
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
    /**
    * @param : current robot zone
    * @return : left branch
    */
    public static Pose2d getLeftBranch(RobotZone zone) {
        Pose2d leftBranch = new Pose2d();
        switch (zone) {
            case RIGHT:
                leftBranch = KnownLocations.leftBranchInRightZone;
                break;
            case BOTTOM_RIGHT:
                leftBranch = KnownLocations.leftBranchInBottomRightZone;
                break;
            case BOTTOM_LEFT:
                leftBranch = KnownLocations.leftBranchInBottomLeftZone;
                break;
            case LEFT:
                leftBranch = KnownLocations.leftBranchInLeftZone;
                break;
            case TOP_LEFT:
                leftBranch = KnownLocations.leftBranchInTopLeftZone;
                break;
            case TOP_RIGHT:
                leftBranch = KnownLocations.leftBranchInTopRightZone;
                break;    
        }
        return leftBranch; //should get here (:
    }

    public static Pose2d getRightBranch(RobotZone zone)  {
        Pose2d rightBranch = new Pose2d();
        switch (zone) {
            case RIGHT:
                rightBranch = KnownLocations.rightBranchInRightZone;
                break;
            case BOTTOM_RIGHT:
                rightBranch = KnownLocations.rightBranchInBottomRightZone;
                break;
            case BOTTOM_LEFT:
                rightBranch = KnownLocations.rightBranchInBottomLeftZone;
                break;
            case LEFT:
                rightBranch = KnownLocations.rightBranchInLeftZone;
                break;
            case TOP_LEFT:
                rightBranch = KnownLocations.rightBranchInTopLeftZone;
                break;
            case TOP_RIGHT:
                rightBranch = KnownLocations.rightBranchInTopRightZone;  
                break;
                     
        }
        return rightBranch; //should not get here
    }

    public enum RobotZone {
        LEFT("left"),
        BOTTOM_LEFT("bottomLeft"),
        BOTTOM_RIGHT("bottomRight"),
        TOP_LEFT("topLeft"),
        TOP_RIGHT("topRight"),
        RIGHT("right");
     
        public String robotZone;
          RobotZone(String zone) {
            this.robotZone = zone;
          }
    }

    public enum CoralStation {
        INSIDELEFT("insideLeft"),
        OUTSIDELEFT("outsideLeft"),
        INSIDERIGHT("insideRight"),
        OUTSIDERIGHT("outsideRight");
     
        public String coralStation;
          CoralStation(String station) {
            this.coralStation = station;
          }
    }

    public enum BranchSide {
        LEFT("left"),
        RIGHT("right");
     
        public String side;
          BranchSide(String side) {
            this.side = side;
          }
    }


    public static RobotZone getCurrentRobotZone() {
        return robotZone;
    }
}

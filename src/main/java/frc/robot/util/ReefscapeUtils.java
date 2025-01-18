package frc.robot.util;

import java.nio.file.Path;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class ReefscapeUtils {
    private static RobotZone robotZone = RobotZone.LEFT; // this is your current robotZone

    private static RobotZone preferredZone = RobotZone.LEFT; // this is your intended robot zone to go to while in FIDO
    private static CoralStation preferredCoralStation = CoralStation.INSIDERIGHT;
    private static Level preferredLevel = Level.L4;
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

    public static Pose2d getpreferredBranch() {
       if (preferredBranchSide == BranchSide.LEFT) {
            return getLeftBranch(preferredZone);
       } else {
            return getRightBranch(preferredZone);
       }
    }

    public static Level getPreferredLevel() {
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

    public static Command getPathToPreferredZone() {
        Supplier<Double> goalEndSupplier = () -> 0.5;
        return new ConditionalCommand(
            PathUtils.getPathToPose(() -> KnownLocations.rightZone, goalEndSupplier),
            new ConditionalCommand(
                PathUtils.getPathToPose(() -> KnownLocations.leftZone, goalEndSupplier), 
                new ConditionalCommand(
                    PathUtils.getPathToPose(() -> KnownLocations.bottomLeftZone, goalEndSupplier), 
                    new ConditionalCommand(
                        PathUtils.getPathToPose(() -> KnownLocations.bottomRightZone, goalEndSupplier), 
                        new ConditionalCommand(
                            PathUtils.getPathToPose(() -> KnownLocations.topRightZone, goalEndSupplier), 
                            new ConditionalCommand(
                                PathUtils.getPathToPose(() -> KnownLocations.topLeftZone, goalEndSupplier), 
                                    PathUtils.getPathToPose(() -> KnownLocations.leftZone, goalEndSupplier), 
                                () -> preferredZone == RobotZone.TOP_LEFT),
                            () -> preferredZone == RobotZone.TOP_RIGHT),
                        () -> preferredZone == RobotZone.BOTTOM_RIGHT), 
                    () -> preferredZone == RobotZone.BOTTOM_LEFT), 
                () -> preferredZone == RobotZone.LEFT), 
            () -> preferredZone == RobotZone.RIGHT);
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
                        () -> preferredCoralStation == CoralStation.OUTSIDELEFT),
                    () -> preferredCoralStation == CoralStation.OUTSIDELEFT),
                () -> preferredCoralStation == CoralStation.OUTSIDELEFT), 
            () -> preferredCoralStation == CoralStation.INSIDELEFT);
    }

    public static void changepreferredBranch(BranchSide side) {
        preferredBranchSide = side;
    }

    public static void changepreferredLevel(Level level) {
        preferredLevel = level;
    }

    public static void changepreferredCoralStation(CoralStation station) {
        preferredCoralStation = station;
    }

    public static void changepreferredZone(RobotZone zone) {
        preferredZone = zone;
    }

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

    public enum Level {
        L1("level1"),
        L2("level2"),
        L3("level3"),
        L4("level4");
     
        public String level;
          Level(String lev) {
            this.level = lev;
          }
    }

    public static RobotZone getCurrentRobotZone() {
        return robotZone;
    }
}
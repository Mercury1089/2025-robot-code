package frc.robot.util;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Currency;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class ReefscapeUtils {
    private static RobotZone robotZone = RobotZone.CLOSE; // this is your current robotZone

    private static RobotZone preferredZone = RobotZone.CLOSE; // this is your intended robot zone to go to while in FIDO
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
        preferredBranchSide = BranchSide.LEFT;
        preferredZone = getCurrentRobotZone();
        return getLeftBranch(robotZone);
    }

    public static Pose2d getCurrentZoneRightBranch() {
        preferredBranchSide = BranchSide.RIGHT;
        preferredZone = getCurrentRobotZone();
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
        KnownLocations locs = KnownLocations.getKnownLocations();
        Pose2d stationPose2d = new Pose2d();

        switch (preferredCoralStation) {
            case INSIDELEFT:
                stationPose2d = locs.leftCoralStationInside;
                break;
            case OUTSIDELEFT:
                stationPose2d = locs.leftCoralStationOutside;
                break;
            case INSIDERIGHT:
                stationPose2d = locs.rightCoralStationInside;
                break;
            case OUTSIDERIGHT:
                stationPose2d = locs.rightCoralStationOutside;
                break;
        }

        return stationPose2d;
    }

    public static Pose2d getpreferredZone() {
        KnownLocations locs = KnownLocations.getKnownLocations();
        Pose2d zone = new Pose2d();

        switch (preferredZone) {
            case BARGE:
                zone = locs.bargeSide;
                break;
            case BARGE_RIGHT:
                zone = locs.rightBargeSide;
                break;
            case CLOSE_RIGHT:
                zone = locs.rightCloseSide;
                break;
            case CLOSE:
                zone = locs.closeSide;
                break;
            case CLOSE_LEFT:
                zone = locs.leftCloseSide;
                break;
            case BARGE_LEFT:
                zone = locs.leftBargeSide;
                break;    
        }

        return zone;
    }

    public static Pose2d getCurrentZoneSafeAlgaePoint() {
        KnownLocations locs = KnownLocations.getKnownLocations();
        Pose2d result;
        switch (robotZone) {
            case CLOSE:
                result = locs.closeSideAlgaeSafePoint;
                break;
            case CLOSE_RIGHT:
                result = locs.rightCloseSideAlgaeSafePoint;
                break;
            case BARGE_RIGHT:
                result = locs.rightBargeSideAlgaeSafePoint;
                break;
            case BARGE:
                result = locs.bargeSideAlgaeSafePoint;
                break;
            case BARGE_LEFT:
                result = locs.leftBargeSideAlgaeSafePoint;
                break;
            case CLOSE_LEFT:
                result = locs.leftCloseSideAlgaeSafePoint;
                break;
            default:
                result = new Pose2d();
                break;
        }
        return result;
    }

    public static Pose2d getCurrentZoneScoreAlgaePoint() {
        KnownLocations locs = KnownLocations.getKnownLocations();
        Pose2d result;
        switch (robotZone) {
            case CLOSE:
                result = locs.closeSideAlgaeScorePoint;
                break;
            case CLOSE_RIGHT:
                result = locs.rightCloseSideAlgaeScorePoint;
                break;
            case BARGE_RIGHT:
                result = locs.rightBargeSideALgaeScorePoint;
                break;
            case BARGE:
                result = locs.bargeSideAlgaeScorePoint;
                break;
            case BARGE_LEFT:
                result = locs.leftBargeSideAlgaeScorePoint;
                break;
            case CLOSE_LEFT:
                result = locs.leftCloseSideAlgaeScorePoint;
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
                case CLOSE:
                    tag = 18;
                    break;
                case CLOSE_RIGHT:
                    tag = 17;
                    break;
                case BARGE_RIGHT:
                    tag = 22;
                    break;
                case BARGE:
                    tag = 21;
                    break;
                case BARGE_LEFT:
                    tag = 20;
                    break;
                case CLOSE_LEFT:
                    tag = 19;
                    break;
                default:
                    break;
            }
        } else {
            switch (robotZone) {
                case CLOSE:
                    tag = 10;
                    break;
                case CLOSE_RIGHT:
                    tag = 11;
                    break;
                case BARGE_RIGHT:
                    tag = 6;
                    break;
                case BARGE:
                    tag = 7;
                    break;
                case BARGE_LEFT:
                    tag = 8;
                    break;
                case CLOSE_LEFT:
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
                PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.CLOSE), goalEndSupplier), 
                new ConditionalCommand(
                    PathUtils.getPathToPose(() -> getRightBranch(RobotZone.CLOSE), goalEndSupplier),
                    new ConditionalCommand(
                        PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.CLOSE_RIGHT), goalEndSupplier),
                        new ConditionalCommand(
                            PathUtils.getPathToPose(() -> getRightBranch(RobotZone.CLOSE_RIGHT), goalEndSupplier),
                            new ConditionalCommand(
                                PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.BARGE_RIGHT), goalEndSupplier),
                                new ConditionalCommand(
                                    PathUtils.getPathToPose(() -> getRightBranch(RobotZone.BARGE_RIGHT), goalEndSupplier),
                                    new ConditionalCommand(
                                        PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.BARGE), goalEndSupplier),
                                        new ConditionalCommand(
                                            PathUtils.getPathToPose(() -> getRightBranch(RobotZone.BARGE), goalEndSupplier),
                                            new ConditionalCommand(
                                                PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.BARGE_LEFT), goalEndSupplier),
                                                new ConditionalCommand(
                                                    PathUtils.getPathToPose(() -> getRightBranch(RobotZone.BARGE_LEFT), goalEndSupplier),
                                                    new ConditionalCommand(
                                                        PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.CLOSE_LEFT), goalEndSupplier),
                                                        new ConditionalCommand(
                                                            PathUtils.getPathToPose(() -> getRightBranch(RobotZone.CLOSE_LEFT), goalEndSupplier),
                                                            PathUtils.getPathToPose(() -> getLeftBranch(RobotZone.CLOSE), goalEndSupplier), 
                                                            () -> preferredZone == RobotZone.CLOSE_LEFT && preferredBranchSide == BranchSide.RIGHT),
                                                        () -> preferredZone == RobotZone.CLOSE_LEFT && preferredBranchSide == BranchSide.LEFT),
                                                    () -> preferredZone == RobotZone.BARGE_LEFT && preferredBranchSide == BranchSide.RIGHT),
                                                () -> preferredZone == RobotZone.BARGE_LEFT && preferredBranchSide == BranchSide.LEFT),
                                            () -> preferredZone == RobotZone.BARGE && preferredBranchSide == BranchSide.RIGHT),
                                        () -> preferredZone == RobotZone.BARGE && preferredBranchSide == BranchSide.LEFT),
                                    () -> preferredZone == RobotZone.BARGE_RIGHT && preferredBranchSide == BranchSide.RIGHT),
                                () -> preferredZone == RobotZone.BARGE_RIGHT && preferredBranchSide == BranchSide.LEFT),
                            () -> preferredZone == RobotZone.CLOSE_RIGHT && preferredBranchSide == BranchSide.RIGHT),
                        () -> preferredZone == RobotZone.CLOSE_RIGHT && preferredBranchSide == BranchSide.LEFT),
                    () -> preferredZone == RobotZone.CLOSE && preferredBranchSide == BranchSide.RIGHT), 
                () -> preferredZone == RobotZone.CLOSE && preferredBranchSide == BranchSide.LEFT);
    }

    public static Command getPathToPreferredCoralStation() {
        KnownLocations locs = KnownLocations.getKnownLocations();
        Supplier<Double> goalEndSupplier = () -> 0.5;
        return new ConditionalCommand(
            PathUtils.getPathToPose(() -> locs.leftCoralStationInside, goalEndSupplier), 
            new ConditionalCommand(
                PathUtils.getPathToPose(() -> locs.leftCoralStationOutside, goalEndSupplier), 
                new ConditionalCommand(
                    PathUtils.getPathToPose(() -> locs.rightCoralStationInside, goalEndSupplier), 
                    new ConditionalCommand(
                        PathUtils.getPathToPose(() -> locs.rightCoralStationOutside, goalEndSupplier), 
                            PathUtils.getPathToPose(() -> locs.rightCoralStationInside, goalEndSupplier), 
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
        double rawHeading = TargetUtils.getTargetHeadingToPoint(robotPose, KnownLocations.getKnownLocations().REEF.getTranslation()).rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
        double finalHeading = 0.0;
        boolean isBlue = KnownLocations.getKnownLocations().alliance == Alliance.Blue;

        if (rawHeading >= -30.0 && rawHeading <= 30.0) {
            finalHeading = 0.0;
            robotZone = isBlue ? RobotZone.CLOSE : RobotZone.BARGE;
        } else if (rawHeading > 30.0 && rawHeading <= 90.0) {
            finalHeading = 60.0;
            robotZone = isBlue ? RobotZone.CLOSE_RIGHT : RobotZone.BARGE_LEFT;
        } else if (rawHeading > 90.0 && rawHeading <= 150.0) {
            finalHeading = 120.0;
            robotZone = isBlue ? RobotZone.BARGE_RIGHT : RobotZone.CLOSE_LEFT;
        } else if (rawHeading > 150.0 && rawHeading <= 180.0) {
            finalHeading = 180.0;
            robotZone = isBlue ? RobotZone.BARGE : RobotZone.CLOSE;
        } else if (rawHeading >= -180.0 && rawHeading < -150.0) {
            finalHeading = 180.0;
            robotZone = isBlue ? RobotZone.BARGE : RobotZone.CLOSE;
        } else if (rawHeading >= -150.0 && rawHeading < -90.0) {
            finalHeading = -120.0;
            robotZone = isBlue ? RobotZone.BARGE_LEFT : RobotZone.CLOSE_RIGHT;
        } else if (rawHeading >= -90.0 && rawHeading < -30.0) {
            finalHeading = -60.0;
            robotZone = isBlue ? RobotZone.CLOSE_LEFT : RobotZone.BARGE_RIGHT;
        }        

        return finalHeading;
    }
    /**
    * @param : robotPose
    * @return : finalHeading
    */
    public static double getTargetHeadingToStation(Pose2d robotPose) {
        KnownLocations locs = KnownLocations.getKnownLocations();
        List<Pose2d> stations = new ArrayList<>();
        stations.add(locs.leftCoralStationOutside);
        stations.add(locs.rightCoralStationOutside);
        stations.add(locs.leftCoralStationInside);
        stations.add(locs.rightCoralStationInside);
        return robotPose.nearest(stations).getRotation().rotateBy(new Rotation2d(180.0)).getDegrees();
    }
    /**
    * @param : current robot zone
    * @return : left branch
    */
    public static Pose2d getLeftBranch(RobotZone zone) {
        KnownLocations locs = KnownLocations.getKnownLocations();
        Pose2d leftBranch = new Pose2d();
        switch (zone) {
            case BARGE:
                leftBranch = locs.bargeSideLeftBranch;
                break;
            case BARGE_RIGHT:
                leftBranch = locs.rightBargeSideLeftBranch;
                break;
            case CLOSE_RIGHT:
                leftBranch = locs.closeRightSideLeftBranch;
                break;
            case CLOSE:
                leftBranch = locs.closeSideLeftBranch;
                break;
            case CLOSE_LEFT:
                leftBranch = locs.leftCloseSideLeftBranch;
                break;
            case BARGE_LEFT:
                leftBranch = locs.leftBargeSideLeftBranch;
                break;    
        }
        return leftBranch; //should get here (:
    }

    public static Pose2d getRightBranch(RobotZone zone)  {
        KnownLocations locs = KnownLocations.getKnownLocations();
        Pose2d rightBranch = new Pose2d();
        switch (zone) {
            case BARGE:
                rightBranch = locs.bargeSideRightBranch;
                break;
            case BARGE_RIGHT:
                rightBranch = locs.rightBargeSideRightBranch;
                break;
            case CLOSE_RIGHT:
                rightBranch = locs.closeRightSideRightBranch;
                break;
            case CLOSE:
                rightBranch = locs.closeSideRightBranch;
                break;
            case CLOSE_LEFT:
                rightBranch = locs.leftCloseSideRightBranch;
                break;
            case BARGE_LEFT:
                rightBranch = locs.leftBargeSideRightBranch;  
                break;
                     
        }
        return rightBranch; //should not get here
    }

    public enum RobotZone {
        CLOSE("close"),
        CLOSE_RIGHT("closeRight"),
        BARGE_RIGHT("bargeRight"),
        CLOSE_LEFT("closeLeft"),
        BARGE_LEFT("bargeLeft"),
        BARGE("barge");
     
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

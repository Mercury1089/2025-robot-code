package frc.robot.commands;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.sensors.DistanceSensors;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.util.KnownLocations;
import frc.robot.util.PathUtils;
import frc.robot.util.ReefscapeUtils;
import frc.robot.util.TargetUtils;
import frc.robot.util.ReefscapeUtils.CoralStation;

public class Autons {

    private SendableChooser<AutonType> autonTypeChooser;
    // private SendableChooser<Pose2d> startingPoseChooser;
    // private SendableChooser<AutonLocations> firstBranchChooser;
    // private SendableChooser<CoralStation> firstStationChooser;
    // private SendableChooser<AutonLocations> secondBranchChooser;
    // private SendableChooser<CoralStation> secondStationChooser;
    // private SendableChooser<AutonLocations> thirdBranchChooser;
    // private SendableChooser<CoralStation> thirdStationChooser;
    // private SendableChooser<AutonLocations> fourthBranchChooser;
    private Pose2d startingPose;
    private AutonLocations firstBranch;
    private AutonLocations secondBranch;
    private AutonLocations thirdBranch;
    // private AutonLocations fourthBranch;
    // private CoralStation firstStation;
    // private CoralStation secondStation;
    // private CoralStation thirdStation;
    // private CoralStation fourthStation;



    private AutonType autoType = AutonType.RIGHT;

    private Command autonCommand;

    private Alliance alliance;

    private final double ROTATION_P = 3.0;
    private final double TRANSLATION_P = 5.0;

    private final Command DO_NOTHING = new PrintCommand("Do Nothing Auton");
    private Drivetrain drivetrain;
    private CoralIntake coralIntake;
    private Elevator elevator;
    // private DistanceSensors proximitySensor;
    private RobotConfig config;

    public Autons(Drivetrain drivetrain, CoralIntake coralIntake, Elevator elevator) {

        this.drivetrain = drivetrain;
        // this.proximitySensor = proximitySensor;
        this.elevator = elevator;
        this.coralIntake = coralIntake;

        KnownLocations knownLocations = KnownLocations.getKnownLocations();
        this.alliance = knownLocations.alliance;

        setChoosers(knownLocations);
        updateAutonLocations();

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> drivetrain.getPose(), // Robot pose supplier
                (pose) -> drivetrain.resetPose(pose), // Method to reset odometry (will be called if your auto has starting pose)
                () -> drivetrain.getFieldRelativSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (chassisSpeeds) -> drivetrain.drive(chassisSpeeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(TRANSLATION_P, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(ROTATION_P, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                }, // Never flip a path - all paths use absolute coordinates
                drivetrain // Reference to this subsystem to set requirements
        );
    }

    public Command getAutonCommand() {
        return this.autonCommand;
    }


    public Command buildAutonCommand(KnownLocations knownLocations) {
        PathPlannerPath firstZonePath = getBasePath(), 
            firstStationPath= getBasePath(), 
            secondZonePath= getBasePath(),
            secondStationPath= getBasePath(),
            thirdZonePath = getBasePath();

        int traj = 0;

        switch (autoType) {
            case LEFT:
                startingPose = knownLocations.leftStart;
                firstBranch = AutonLocations.LEFTINLEFTBARGEZONE;
                secondBranch = AutonLocations.LEFTINLEFTCLOSEZONE;
                thirdBranch = AutonLocations.RIGHTINLEFTCLOSEZONE;

                try {
                    firstZonePath = PathPlannerPath.fromPathFile("leftStartToleftBargeSide");
                    firstStationPath = PathPlannerPath.fromPathFile("leftBargeSideToleftStation");
                    secondZonePath = PathPlannerPath.fromPathFile("leftStationToleftCloseSide");
                    secondStationPath = PathPlannerPath.fromPathFile("leftCloseSideToleftStation");
                    thirdZonePath = PathPlannerPath.fromPathFile("leftStationToleftCloseSide");
                } catch (Exception e) {
                    // TODO: handle exception
                }

                break;
            case RIGHT:
                startingPose = knownLocations.rightStart;
                firstBranch = AutonLocations.LEFTINRIGHTBARGEZONE;
                secondBranch = AutonLocations.LEFTINRIGHTCLOSEZONE;
                thirdBranch = AutonLocations.RIGHTINRIGHTCLOSEZONE;
                
                try {
                    firstZonePath = PathPlannerPath.fromPathFile("rightStartTorightBargeSide");
                    firstStationPath = PathPlannerPath.fromPathFile("rightBargeSideTorightStation");
                    secondZonePath = PathPlannerPath.fromPathFile("rightStationTorightCloseSide");
                    secondStationPath = PathPlannerPath.fromPathFile("rightCloseSideTorightStation");
                    thirdZonePath = PathPlannerPath.fromPathFile("rightStationTorightCloseSide");
                } catch (Exception e) {
                    // TODO: handle exception
                }

                break;
            case MIDDLE:
                startingPose = knownLocations.middleStart;
                firstBranch = AutonLocations.LEFTINBARGEZONE;
                try {
                    firstZonePath = PathPlannerPath.fromPathFile("middleStartTobargeSide");
                } catch (Exception e) {
                    // TODO: handle exception
                }
                break;
            default:
                break;
        }
        
        boolean isRedAlliance = (KnownLocations.getKnownLocations().alliance == Alliance.Red);
        boolean isMiddleAuton = (autoType == AutonType.MIDDLE);
        int trajIndex = 0;

        PathPlannerPath[] paths = {firstZonePath, firstStationPath, secondZonePath, secondStationPath, thirdZonePath};

        for (int i = 0; i < paths.length; i++) {
            if (isMiddleAuton && i > 0) {
                drivetrain.setTrajectorySmartdash(new Trajectory(), trajIndex + "");
            } else {
                PathPlannerPath path = isRedAlliance ? paths[i].flipPath() : paths[i];
                drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path, config), trajIndex + "");
            }

            trajIndex++;
        }
        
        
        

        // SET OUR INITIAL POSE
        drivetrain.resetPose(startingPose);
        drivetrain.setStartingPosition(startingPose);

        SequentialCommandGroup autonCommandGroup = new SequentialCommandGroup();
        autonCommandGroup.addCommands(new InstantCommand(() -> ReefscapeUtils.changePreferredLevel(ElevatorPosition.LEVEL4)));

        //giant mess sponsored by lucas and megh
        switch (autoType) {
            case MIDDLE:
                autonCommandGroup.addCommands(
                    AutoBuilder.followPath(firstZonePath).until(() -> TargetUtils.getDistanceToPoint(drivetrain.getPose(), firstBranch.getBranch().getTranslation()) < 1),
                    DriveCommands.driveAndScoreAtBranchAuton(drivetrain, () -> firstBranch.getBranch(), elevator, coralIntake),
                    new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.HOME), elevator)
                );
                break;
            default:
                autonCommandGroup.addCommands(
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(firstZonePath),
                        new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.LEVEL4))
                        )
                    ).until(() -> TargetUtils.getDistanceToPoint(drivetrain.getPose(), firstBranch.getBranch().getTranslation()) < 1),
                    DriveCommands.driveAndScoreAtBranchAuton(drivetrain, () -> firstBranch.getBranch(), elevator, coralIntake),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(firstStationPath),
                        new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.HOME), elevator)
                    ).until(() -> coralIntake.hasCoralEntered()),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(secondZonePath),
                        new SequentialCommandGroup(
                            new WaitCommand(0.75),
                            new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.LEVEL4))
                        )
                    ).until(() -> TargetUtils.getDistanceToPoint(drivetrain.getPose(), secondBranch.getBranch().getTranslation()) < 1),
                    DriveCommands.driveAndScoreAtBranchAuton(drivetrain, () -> secondBranch.getBranch(), elevator, coralIntake),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(secondStationPath),
                        new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.HOME), elevator)
                    ).until(() -> coralIntake.hasCoralEntered()),

                    new ParallelCommandGroup(
                        AutoBuilder.followPath(thirdZonePath),
                        new SequentialCommandGroup(
                            new WaitCommand(0.75),
                            new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.LEVEL4))
                        )
                    ).until(() -> TargetUtils.getDistanceToPoint(drivetrain.getPose(), thirdBranch.getBranch().getTranslation()) < 1),
                    DriveCommands.driveAndScoreAtBranchAuton(drivetrain, () -> thirdBranch.getBranch(), elevator, coralIntake)
                );
                break;
        }

        return autonCommandGroup;
    }

    public PathPlannerPath getBasePath() {
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new Waypoint(new Translation2d(), new Translation2d(), new Translation2d()));
        waypoints.add(new Waypoint(new Translation2d(), new Translation2d(), new Translation2d()));

        PathConstraints constraints = new PathConstraints(0.0,0.0,0.0,0.0,0.0, true);
        IdealStartingState idealStartingState = new IdealStartingState(0.0, Rotation2d.fromDegrees(0.0));
        GoalEndState goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(0.0));
        return new PathPlannerPath(waypoints, constraints, idealStartingState, goalEndState);
    }


    /**
     * Rebuilds the autonCommand when ONE of the following conditions changes:
     * - Alliance Color
     * - Starting Pose
     * - like all the things we do
     */

    public void updateDash() {

        boolean rebuildAutonCommand = false;

        KnownLocations knownLocations = KnownLocations.getKnownLocations();

        if (knownLocations.alliance != this.alliance) {
            this.alliance = knownLocations.alliance;
            SmartDashboard.putString("alliance color!", this.alliance.toString());
            setChoosers(knownLocations);
            rebuildAutonCommand = true;
        }

        AutonType type = autonTypeChooser.getSelected();

        if (type != this.autoType) {
            this.autoType = type;
            rebuildAutonCommand = true;
        }

        // Pose2d startingPose = startingPoseChooser.getSelected();

        // AutonLocations firstBranch = firstBranchChooser.getSelected();
        // AutonLocations secondBranch = secondBranchChooser.getSelected();
        // AutonLocations thirdBranch = thirdBranchChooser.getSelected();
        // // AutonLocations fourthBranch = fourthBranchChooser.getSelected();

        // CoralStation firstStation = firstStationChooser.getSelected();
        // CoralStation secondStation = secondStationChooser.getSelected();
        // // CoralStation thirdStation = thirdStationChooser.getSelected();

        // if (startingPose != this.startingPose) {
        //     this.startingPose = startingPose;
        //     rebuildAutonCommand = true;
        // }

        // if (firstBranch != this.firstBranch) {
        //     this.firstBranch = firstBranch;
        //     rebuildAutonCommand = true;
        // }

        // if (secondBranch != this.secondBranch) {
        //     this.secondBranch = secondBranch;
        //     rebuildAutonCommand = true;
        // }

        // if (thirdBranch != this.thirdBranch) {
        //     this.thirdBranch = thirdBranch;
        //     rebuildAutonCommand = true;
        // }
        
        // if (fourthBranch != this.fourthBranch) {
        //     this.fourthBranch = fourthBranch;
        //     rebuildAutonCommand = true;
        // }

        // if (firstStation != this.firstStation) {
        //     this.firstStation = firstStation;
        //     rebuildAutonCommand = true;
        // }

        // if (secondStation != this.secondStation) {
        //     this.secondStation = secondStation;
        //     rebuildAutonCommand = true;
        // }

        // if (thirdStation != this.thirdStation) {
        //     this.thirdStation = thirdStation;
        //     rebuildAutonCommand = true;
        // }
        

        if (rebuildAutonCommand) {
            this.autonCommand = buildAutonCommand(knownLocations);
        }
    }

    public void setChoosers(KnownLocations knownLocations) {
        autonTypeChooser = new SendableChooser<AutonType>();
        autonTypeChooser.setDefaultOption("RIGHT", AutonType.RIGHT);
        autonTypeChooser.addOption("MIDDLE", AutonType.MIDDLE);
        autonTypeChooser.addOption("LEFT", AutonType.LEFT);

        // // select the MANUAL STARTING POSITION of the robot
        // this.startingPoseChooser = new SendableChooser<Pose2d>();
        // startingPoseChooser.setDefaultOption("RIGHT", knownLocations.rightStart);
        // startingPoseChooser.addOption("Middle", knownLocations.middleStart);
        // startingPoseChooser.addOption("LEFT", knownLocations.leftStart);
        

        // firstBranchChooser = getBranchChooser();
        // secondBranchChooser = getBranchChooser();
        // thirdBranchChooser = getBranchChooser();
        // // fourthBranchChooser = getBranchChooser();
        // firstStationChooser = getCoralStationChooser();
        // secondStationChooser = getCoralStationChooser();
        // // thirdStationChooser = getCoralStationChooser();

        // SmartDashboard.putData("Starting Pose", startingPoseChooser);
        // SmartDashboard.putData("First Branch", firstBranchChooser);
        // SmartDashboard.putData("Second Branch", secondBranchChooser);
        // SmartDashboard.putData("Third Branch", thirdBranchChooser);
        // SmartDashboard.putData("First Coral Station", firstStationChooser);
        // SmartDashboard.putData("Second Coral Station", secondStationChooser);

        SmartDashboard.putData("Auton Type Chooser", autonTypeChooser);
    }

    private SendableChooser<AutonLocations> getBranchChooser() {
        SendableChooser<AutonLocations> branchChooser = new SendableChooser<AutonLocations>();
        branchChooser.setDefaultOption("D Right", AutonLocations.RIGHTINBARGEZONE);
        branchChooser.addOption("E Left", AutonLocations.LEFTINRIGHTBARGEZONE);
        branchChooser.addOption("E Right", AutonLocations.RIGHTINRIGHTBARGEZONE);
        branchChooser.addOption("F Right", AutonLocations.RIGHTINRIGHTCLOSEZONE);
        branchChooser.addOption("F Left", AutonLocations.LEFTINRIGHTCLOSEZONE);
        branchChooser.addOption("A Right", AutonLocations.RIGHTINCLOSEZONE);
        branchChooser.addOption("A Left", AutonLocations.LEFTINCLOSEZONE);
        branchChooser.addOption("B Right", AutonLocations.RIGHTINLEFTCLOSEZONE);
        branchChooser.addOption("B Left", AutonLocations.LEFTINLEFTCLOSEZONE);
        branchChooser.addOption("C Left", AutonLocations.LEFTINLEFTBARGEZONE);
        branchChooser.addOption("C Right", AutonLocations.RIGHTINLEFTBARGEZONE);
        branchChooser.addOption("D Left", AutonLocations.LEFTINBARGEZONE);
        return branchChooser;
    }

    private SendableChooser<CoralStation> getCoralStationChooser() {
        SendableChooser<CoralStation> coralStationChooser = new SendableChooser<CoralStation>();
        coralStationChooser.setDefaultOption("Right", CoralStation.RIGHT);
        coralStationChooser.addOption("Left", CoralStation.LEFT);
        return coralStationChooser;
    }

    public void updateAutonLocations() {
        KnownLocations locs = KnownLocations.getKnownLocations();
        AutonLocations.LEFTINBARGEZONE.setZonePose(locs.bargeZone);
        AutonLocations.RIGHTINBARGEZONE.setZonePose(locs.bargeZone);
        AutonLocations.LEFTINRIGHTBARGEZONE.setZonePose(locs.rightBargeZone);
        AutonLocations.RIGHTINRIGHTBARGEZONE.setZonePose(locs.rightBargeZone);
        AutonLocations.LEFTINLEFTBARGEZONE.setZonePose(locs.leftBargeZone);
        AutonLocations.RIGHTINLEFTBARGEZONE.setZonePose(locs.leftBargeZone);
        AutonLocations.LEFTINCLOSEZONE.setZonePose(locs.closeZone);
        AutonLocations.RIGHTINCLOSEZONE.setZonePose(locs.closeZone);
        AutonLocations.LEFTINRIGHTCLOSEZONE.setZonePose(locs.rightCloseZone);
        AutonLocations.RIGHTINRIGHTCLOSEZONE.setZonePose(locs.rightCloseZone);
        AutonLocations.LEFTINLEFTCLOSEZONE.setZonePose(locs.leftCloseZone);
        AutonLocations.RIGHTINLEFTCLOSEZONE.setZonePose(locs.leftCloseZone);

        AutonLocations.LEFTINBARGEZONE.setBranch(locs.bargeSideLeftBranch);
        AutonLocations.RIGHTINBARGEZONE.setBranch(locs.bargeSideRightBranch);
        AutonLocations.LEFTINRIGHTBARGEZONE.setBranch(locs.rightBargeSideLeftBranch);
        AutonLocations.RIGHTINRIGHTBARGEZONE.setBranch(locs.rightBargeSideRightBranch);
        AutonLocations.LEFTINLEFTBARGEZONE.setBranch(locs.leftBargeSideLeftBranch);
        AutonLocations.RIGHTINLEFTBARGEZONE.setBranch(locs.leftBargeSideRightBranch);
        AutonLocations.LEFTINCLOSEZONE.setBranch(locs.closeSideLeftBranch);
        AutonLocations.RIGHTINCLOSEZONE.setBranch(locs.closeSideRightBranch);
        AutonLocations.LEFTINRIGHTCLOSEZONE.setBranch(locs.rightCloseSideLeftBranch);
        AutonLocations.RIGHTINRIGHTCLOSEZONE.setBranch(locs.rightCloseSideRightBranch);
        AutonLocations.LEFTINLEFTCLOSEZONE.setBranch(locs.leftCloseSideLeftBranch);
        AutonLocations.RIGHTINLEFTCLOSEZONE.setBranch(locs.leftCloseSideRightBranch);

        CoralStation.LEFT.setStation(locs.leftCoralStation);
        CoralStation.RIGHT.setStation(locs.rightCoralStation);
    }

    //field relative
    public enum AutonLocations {
        LEFTINBARGEZONE(KnownLocations.getKnownLocations().bargeZone, KnownLocations.getKnownLocations().bargeSideLeftBranch),
        RIGHTINBARGEZONE(KnownLocations.getKnownLocations().bargeZone, KnownLocations.getKnownLocations().bargeSideRightBranch),
        LEFTINRIGHTBARGEZONE(KnownLocations.getKnownLocations().rightBargeZone, KnownLocations.getKnownLocations().rightBargeSideLeftBranch),
        RIGHTINRIGHTBARGEZONE(KnownLocations.getKnownLocations().rightBargeZone, KnownLocations.getKnownLocations().rightBargeSideRightBranch),
        LEFTINRIGHTCLOSEZONE(KnownLocations.getKnownLocations().rightCloseZone, KnownLocations.getKnownLocations().rightCloseSideLeftBranch),
        RIGHTINRIGHTCLOSEZONE(KnownLocations.getKnownLocations().rightCloseZone, KnownLocations.getKnownLocations().rightBargeSideRightBranch),
        LEFTINCLOSEZONE(KnownLocations.getKnownLocations().closeZone, KnownLocations.getKnownLocations().closeSideLeftBranch),
        RIGHTINCLOSEZONE(KnownLocations.getKnownLocations().closeZone, KnownLocations.getKnownLocations().closeSideRightBranch),
        LEFTINLEFTCLOSEZONE(KnownLocations.getKnownLocations().leftCloseZone, KnownLocations.getKnownLocations().leftCloseSideLeftBranch),
        RIGHTINLEFTCLOSEZONE(KnownLocations.getKnownLocations().leftCloseZone, KnownLocations.getKnownLocations().leftCloseSideRightBranch),
        LEFTINLEFTBARGEZONE(KnownLocations.getKnownLocations().leftBargeZone, KnownLocations.getKnownLocations().leftBargeSideLeftBranch),
        RIGHTINLEFTBARGEZONE(KnownLocations.getKnownLocations().leftBargeZone, KnownLocations.getKnownLocations().leftBargeSideRightBranch);

        private Pose2d zonePose;
        private Pose2d branch;

        AutonLocations(Pose2d zonePose, Pose2d branchPose) {
            this.zonePose = zonePose;
            this.branch = branchPose;
        }

        public Pose2d getZonePose() {
            return zonePose;
        }

        public void setZonePose(Pose2d zonePose) {
            this.zonePose = zonePose;
        }

        public Pose2d getBranch() {
            return branch;
        }

        public void setBranch(Pose2d branchPose) {
            branch = branchPose;
        }
    }

    public enum AutonType {
        LEFT,
        MIDDLE,
        RIGHT;
    }
}
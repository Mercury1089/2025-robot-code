package frc.robot.commands;

import java.nio.file.Path;
import java.sql.Driver;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SWERVE;
// import frc.robot.sensors.DistanceSensors;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.CoralIntake.IntakeSpeed;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.PathUtils;
import frc.robot.util.ReefscapeUtils;
import frc.robot.util.TargetUtils;
import frc.robot.util.ReefscapeUtils.BranchSide;
import frc.robot.util.ReefscapeUtils.RobotZone;
import frc.robot.util.ReefscapeUtils.CoralStation;

public class Autons {

    private SendableChooser<Pose2d> startingPoseChooser;
    private SendableChooser<AutonLocations> firstBranchChooser;
    private SendableChooser<CoralStation> firstStationChooser;
    private SendableChooser<AutonLocations> secondBranchChooser;
    private SendableChooser<CoralStation> secondStationChooser;
    private SendableChooser<AutonLocations> thirdBranchChooser;
    private SendableChooser<CoralStation> thirdStationChooser;
    private SendableChooser<AutonLocations> fourthBranchChooser;
    private Pose2d startingPose;
    private AutonLocations firstBranch;
    private AutonLocations secondBranch;
    private AutonLocations thirdBranch;
    private AutonLocations fourthBranch;
    private CoralStation firstStation;
    private CoralStation secondStation;
    private CoralStation thirdStation;
    private CoralStation fourthStation;
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
        // SET OUR INITIAL POSE
        drivetrain.resetPose(startingPose);
        drivetrain.setStartingPosition(startingPose);

        SequentialCommandGroup autonCommandGroup = new SequentialCommandGroup();

        autonCommandGroup.addCommands(//TODO: fix
            new InstantCommand(() -> ReefscapeUtils.changePreferredLevel(ElevatorPosition.LEVEL2)),
            PathUtils.getPathToPose(firstBranch.getPose(), () -> 0.5),
            new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.LEVEL2), elevator).until(() -> elevator.isInPosition()),
            new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.OUTTAKE)).until(() -> coralIntake.noCoralPresent()),

            new InstantCommand(() -> ReefscapeUtils.changePreferredCoralStation(firstStation)),
            DriveCommands.getCoralFromStation(drivetrain, elevator, coralIntake, () -> firstStation.stationPose),

            // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> secondBranch.getZone(), () -> secondBranch.getSide(), () -> secondBranch.getPose(), elevator, coralIntake),

            new InstantCommand(() -> ReefscapeUtils.changePreferredCoralStation(secondStation)),
            DriveCommands.getCoralFromStation(drivetrain, elevator, coralIntake, () -> secondStation.stationPose)

            // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> thirdBranch.getZone(), () -> thirdBranch.getSide(), () -> thirdBranch.getPose(), elevator, coralIntake)
        );

        return autonCommandGroup;
    }

    public void changePreferredScoringLocation(AutonLocations loc) {
        ReefscapeUtils.changePreferredZone(loc.getZone());
        ReefscapeUtils.changePreferredBranchSide(loc.getSide());
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

        Pose2d startingPose = startingPoseChooser.getSelected();

        AutonLocations firstBranch = firstBranchChooser.getSelected();
        AutonLocations secondBranch = secondBranchChooser.getSelected();
        AutonLocations thirdBranch = thirdBranchChooser.getSelected();
        // AutonLocations fourthBranch = fourthBranchChooser.getSelected();

        CoralStation firstStation = firstStationChooser.getSelected();
        CoralStation secondStation = secondStationChooser.getSelected();
        // CoralStation thirdStation = thirdStationChooser.getSelected();

        if (startingPose != this.startingPose) {
            this.startingPose = startingPose;
            rebuildAutonCommand = true;
        }

        if (firstBranch != this.firstBranch) {
            this.firstBranch = firstBranch;
            rebuildAutonCommand = true;
        }

        if (secondBranch != this.secondBranch) {
            this.secondBranch = secondBranch;
            rebuildAutonCommand = true;
        }

        if (thirdBranch != this.thirdBranch) {
            this.thirdBranch = thirdBranch;
            rebuildAutonCommand = true;
        }
        
        // if (fourthBranch != this.fourthBranch) {
        //     this.fourthBranch = fourthBranch;
        //     rebuildAutonCommand = true;
        // }

        if (firstStation != this.firstStation) {
            this.firstStation = firstStation;
            rebuildAutonCommand = true;
        }

        if (secondStation != this.secondStation) {
            this.secondStation = secondStation;
            rebuildAutonCommand = true;
        }

        // if (thirdStation != this.thirdStation) {
        //     this.thirdStation = thirdStation;
        //     rebuildAutonCommand = true;
        // }
        

        if (rebuildAutonCommand) {
            this.autonCommand = buildAutonCommand(knownLocations);
        }
    }

    public void setChoosers(KnownLocations knownLocations) {
        // select the MANUAL STARTING POSITION of the robot
        this.startingPoseChooser = new SendableChooser<Pose2d>();
        startingPoseChooser.setDefaultOption("Bottom", knownLocations.bottomMostStart);
        startingPoseChooser.addOption("Middle", knownLocations.middleStart);
        startingPoseChooser.addOption("Top", knownLocations.topMostStart);

        // startingPoseChooser.addOption("closeRightRight", knownLocations.closeRightSideRightBranch);
        // startingPoseChooser.addOption("closeRightLeft", knownLocations.closeRightSideLeftBranch);
        // startingPoseChooser.addOption("bargeLeftRight", knownLocations.leftBargeSideRightBranch);
        // startingPoseChooser.addOption("bargeLeftLeft", knownLocations.leftBargeSideLeftBranch);
        // startingPoseChooser.addOption("bargeRightLeft", knownLocations.rightBargeSideLeftBranch);
        // startingPoseChooser.addOption("bargeRightRight", knownLocations.rightBargeSideRightBranch);
        // startingPoseChooser.addOption("closeLeftLeft", knownLocations.leftCloseSideLeftBranch);
        // startingPoseChooser.addOption("closeLeftRight", knownLocations.leftCloseSideRightBranch);
        // startingPoseChooser.addOption("closeLeft", knownLocations.closeSideLeftBranch);
        // startingPoseChooser.addOption("closeRight", knownLocations.closeSideRightBranch);
        // startingPoseChooser.addOption("bargeLeft", knownLocations.bargeSideLeftBranch);
        // startingPoseChooser.addOption("bargeRight", knownLocations.bargeSideRightBranch);
        // startingPoseChooser.addOption("reef", knownLocations.REEF);

        // startingPoseChooser.addOption("BRSAFE", KnownLocations.rightBargeSideAlgaeSafePoint);
        // startingPoseChooser.addOption("BRSCORE", KnownLocations.rightBargeSideALgaeScorePoint);
        
        // startingPoseChooser.addOption("BLSAFE", KnownLocations.rightCloseSideAlgaeSafePoint);
        // startingPoseChooser.addOption("BLSCORE", KnownLocations.rightCloseSideAlgaeScorePoint);
        
        // startingPoseChooser.addOption("RSAFE", KnownLocations.bargeSideAlgaeSafePoint);
        // startingPoseChooser.addOption("RSCORE", KnownLocations.bargeSideAlgaeScorePoint);
        
        // startingPoseChooser.addOption("TRSAFE", KnownLocations.leftBargeSideAlgaeSafePoint);
        // startingPoseChooser.addOption("TRSCORE", KnownLocations.leftBargeSideAlgaeScorePoint);
        
        // startingPoseChooser.addOption("TLSAFE", KnownLocations.leftCloseSideAlgaeSafePoint);
        // startingPoseChooser.addOption("TLSCORE", KnownLocations.leftCloseSideAlgaeScorePoint);
        
        // startingPoseChooser.addOption("LSAFE", KnownLocations.closeSideAlgaeSafePoint);
        // startingPoseChooser.addOption("LSCORE", KnownLocations.closeSideAlgaeScorePoint);
        

        firstBranchChooser = getBranchChooser();
        secondBranchChooser = getBranchChooser();
        thirdBranchChooser = getBranchChooser();
        // fourthBranchChooser = getBranchChooser();
        firstStationChooser = getCoralStationChooser();
        secondStationChooser = getCoralStationChooser();
        // thirdStationChooser = getCoralStationChooser();

        SmartDashboard.putData("Starting Pose", startingPoseChooser);
        SmartDashboard.putData("First Branch", firstBranchChooser);
        SmartDashboard.putData("Second Branch", secondBranchChooser);
        SmartDashboard.putData("Third Branch", thirdBranchChooser);
        // SmartDashboard.putData("Fourth Branch", fourthBranchChooser);
        SmartDashboard.putData("First Coral Station", firstStationChooser);
        SmartDashboard.putData("Second Coral Station", secondStationChooser);
        // SmartDashboard.putData("Third Coral Station", thirdStationChooser);
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
        coralStationChooser.setDefaultOption("Outside Right", CoralStation.OUTSIDERIGHT);
        coralStationChooser.addOption("Inside Right", CoralStation.INSIDERIGHT);
        coralStationChooser.addOption("Inside Left", CoralStation.INSIDELEFT);
        coralStationChooser.addOption("Outside Left", CoralStation.OUTSIDELEFT);
        return coralStationChooser;
    }

    public void updateAutonLocations() {
        KnownLocations locs = KnownLocations.getKnownLocations();
        AutonLocations.LEFTINBARGEZONE.setPose(locs.bargeSideLeftBranch);
        AutonLocations.RIGHTINBARGEZONE.setPose(locs.bargeSideRightBranch);
        AutonLocations.LEFTINRIGHTBARGEZONE.setPose(locs.rightBargeSideLeftBranch);
        AutonLocations.RIGHTINRIGHTBARGEZONE.setPose(locs.rightBargeSideRightBranch);
        AutonLocations.LEFTINLEFTBARGEZONE.setPose(locs.leftBargeSideLeftBranch);
        AutonLocations.RIGHTINLEFTBARGEZONE.setPose(locs.leftBargeSideRightBranch);
        AutonLocations.LEFTINCLOSEZONE.setPose(locs.closeSideLeftBranch);
        AutonLocations.RIGHTINCLOSEZONE.setPose(locs.closeSideRightBranch);
        AutonLocations.LEFTINRIGHTCLOSEZONE.setPose(locs.rightCloseSideLeftBranch);
        AutonLocations.RIGHTINRIGHTCLOSEZONE.setPose(locs.rightCloseSideRightBranch);
        AutonLocations.LEFTINLEFTCLOSEZONE.setPose(locs.leftCloseSideLeftBranch);
        AutonLocations.RIGHTINLEFTCLOSEZONE.setPose(locs.leftCloseSideRightBranch);

        CoralStation.INSIDELEFT.setStation(locs.leftCoralStationInside);
        CoralStation.INSIDERIGHT.setStation(locs.rightCoralStationInside);
        CoralStation.OUTSIDELEFT.setStation(locs.leftCoralStationOutside);
        CoralStation.OUTSIDERIGHT.setStation(locs.rightCoralStationOutside);
    }

    //field relative
    public enum AutonLocations {
        LEFTINBARGEZONE(RobotZone.BARGE, BranchSide.LEFT),
        RIGHTINBARGEZONE(RobotZone.BARGE, BranchSide.RIGHT),
        LEFTINRIGHTBARGEZONE(RobotZone.BARGE_RIGHT, BranchSide.LEFT),
        RIGHTINRIGHTBARGEZONE(RobotZone.BARGE_RIGHT, BranchSide.RIGHT),
        LEFTINRIGHTCLOSEZONE(RobotZone.CLOSE_RIGHT, BranchSide.LEFT),
        RIGHTINRIGHTCLOSEZONE(RobotZone.CLOSE_RIGHT, BranchSide.RIGHT),
        LEFTINCLOSEZONE(RobotZone.CLOSE, BranchSide.LEFT),
        RIGHTINCLOSEZONE(RobotZone.CLOSE, BranchSide.RIGHT),
        LEFTINLEFTCLOSEZONE(RobotZone.CLOSE_LEFT, BranchSide.LEFT),
        RIGHTINLEFTCLOSEZONE(RobotZone.CLOSE_LEFT, BranchSide.RIGHT),
        LEFTINLEFTBARGEZONE(RobotZone.BARGE_LEFT, BranchSide.LEFT),
        RIGHTINLEFTBARGEZONE(RobotZone.BARGE_LEFT, BranchSide.RIGHT);

        private RobotZone zone;
        private BranchSide side;
        private Pose2d branchPose;

        AutonLocations(RobotZone z, BranchSide s) {
            this.zone = z;
            this.side = s;
        }

        public RobotZone getZone() {
            return zone;
        }

        public BranchSide getSide() {
            return side;
        }

        public Pose2d getPose() {
            return branchPose;
        }

        public void setPose(Pose2d pose) {
            branchPose = pose;
        }
    }
}
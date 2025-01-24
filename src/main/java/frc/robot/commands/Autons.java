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
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.PathUtils;
import frc.robot.util.TargetUtils;

public class Autons {

    private SendableChooser<Pose2d> startingPoseChooser;
    private SendableChooser<Pose2d> firstBranchChooser;
    private SendableChooser<Pose2d> firstStationChooser;
    private SendableChooser<Pose2d> secondBranchChooser;
    private SendableChooser<Pose2d> secondStationChooser;
    private SendableChooser<Pose2d> thirdBranchChooser;
    private SendableChooser<Pose2d> thirdStationChooser;
    private SendableChooser<Pose2d> fourthBranchChooser;
    private Pose2d startingPose;
    private Pose2d firstBranch;
    private Pose2d secondBranch;
    private Pose2d thirdBranch;
    private Pose2d fourthBranch;
    private Pose2d firstStation;
    private Pose2d secondStation;
    private Pose2d thirdStation;
    private Pose2d fourthStation;
    private Command autonCommand;

    private Alliance alliance;

    private final double ROTATION_P = 3.0;
    private final double TRANSLATION_P = 5.0;

    private final Command DO_NOTHING = new PrintCommand("Do Nothing Auton");
    private Drivetrain drivetrain;
    private RobotConfig config;

    public Autons(Drivetrain drivetrain) {

        this.drivetrain = drivetrain;

        KnownLocations knownLocations = KnownLocations.getKnownLocations();
        this.alliance = knownLocations.alliance;


        setChoosers(knownLocations);

        
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

        SequentialCommandGroup autonCommand = new SequentialCommandGroup();

        return autonCommand;
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

        Pose2d firstBranch = firstBranchChooser.getSelected();
        Pose2d secondBranch = secondBranchChooser.getSelected();
        Pose2d thirdBranch = thirdBranchChooser.getSelected();
        Pose2d fourthBranch = fourthBranchChooser.getSelected();

        Pose2d firstStation = firstStationChooser.getSelected();
        Pose2d secondStation = secondStationChooser.getSelected();
        Pose2d thirdStation = thirdStationChooser.getSelected();

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

        if (fourthBranch != this.fourthBranch) {
            this.fourthBranch = fourthBranch;
            rebuildAutonCommand = true;
        }

        if (firstStation != this.firstStation) {
            this.firstStation = firstStation;
            rebuildAutonCommand = true;
        }

        if (secondStation != this.secondStation) {
            this.secondBranch = secondBranch;
            rebuildAutonCommand = true;
        }

        if (thirdStation != this.thirdStation) {
            this.thirdStation = thirdStation;
            rebuildAutonCommand = true;
        }

        if (rebuildAutonCommand) {
            this.autonCommand = buildAutonCommand(knownLocations);
        }
    }

    public void setChoosers(KnownLocations knownLocations) {
        // select the MANUAL STARTING POSITION of the robot
        this.startingPoseChooser = new SendableChooser<Pose2d>();
        startingPoseChooser.setDefaultOption("Bottom", KnownLocations.bottomMostStart);
        startingPoseChooser.addOption("Middle", KnownLocations.middleStart);
        startingPoseChooser.addOption("Top", KnownLocations.topMostStart);

        firstBranchChooser = getBranchChooser();
        secondBranchChooser = getBranchChooser();
        thirdBranchChooser = getBranchChooser();
        fourthBranchChooser = getBranchChooser();
        firstStationChooser = getCoralStationChooser();
        secondStationChooser = getCoralStationChooser();
        thirdStationChooser = getCoralStationChooser();
    }

    private SendableChooser<Pose2d> getBranchChooser() {
        SendableChooser<Pose2d> branchChooser = new SendableChooser<Pose2d>();
        branchChooser.setDefaultOption("Right Zone Right Branch", KnownLocations.rightBranchInRightZone);
        branchChooser.addOption("Bottom Right Zone Left Branch", KnownLocations.leftBranchInBottomRightZone);
        branchChooser.addOption("Bottom Right Zone Right Branch", KnownLocations.rightBranchInBottomRightZone);
        branchChooser.addOption("Bottom Left Zone Right Branch", KnownLocations.rightBranchInBottomLeftZone);
        branchChooser.addOption("Bottom Left Zone Left Branch", KnownLocations.leftBranchInBottomLeftZone);
        branchChooser.addOption("Left Zone Right Branch", KnownLocations.rightBranchInLeftZone);
        branchChooser.addOption("Left Zone Left Branch", KnownLocations.leftBranchInLeftZone);
        branchChooser.addOption("Top Left Zone Right Branch", KnownLocations.rightBranchInTopLeftZone);
        branchChooser.addOption("Top Left Zone Left Branch", KnownLocations.leftBranchInTopLeftZone);
        branchChooser.addOption("Top Right Zone Left Branch", KnownLocations.leftBranchInTopRightZone);
        branchChooser.addOption("Top Right Zone Right Branch", KnownLocations.rightBranchInTopRightZone);
        branchChooser.addOption("Right Zone Left Branch", KnownLocations.leftBranchInRightZone);
        return branchChooser;
    }

    private SendableChooser<Pose2d> getCoralStationChooser() {
        SendableChooser<Pose2d> coralStationChooser = new SendableChooser<Pose2d>();
        coralStationChooser.setDefaultOption("Outside Right", KnownLocations.rightCoralStationOutside);
        coralStationChooser.addOption("Inside Right", KnownLocations.rightCoralStationInside);
        coralStationChooser.addOption("Inside Left", KnownLocations.leftCoralStationInside);
        coralStationChooser.addOption("Outside Left", KnownLocations.leftCoralStationOutside);
        return coralStationChooser;
    }

    /**
     * Determines what we after scoring initial note
     */
    // public enum AutonTypes {
    //     DO_NOT_MOVE, // Do nothing after scoring first note
    //     LEAVE_STARTING_ZONE, // Leave the starting area after scoring first note
    //     SCORE_2ND_NOTE, // Score a second NOTE
    //     MULTI_NOTE_SCORE, // Score multiple NOTES
    //     WING_NOTES, // Score additional WING NOTES
    //     CENTER_LINE_NOTES // Score CENTER LINE NOTES
    // }

    //TODO: figure out how to do auton locations (choosers as pose or as enum?)
    // public enum AutonLocations {

    // }
}
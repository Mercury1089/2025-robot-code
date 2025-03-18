package frc.robot.commands;

import java.sql.Driver;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
// import frc.robot.sensors.DistanceSensors;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.AlgaeArticulator;
import frc.robot.subsystems.elevator.AlgaeIntake;
import frc.robot.subsystems.elevator.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.AlgaeArticulator.ArticulatorPosition;
import frc.robot.subsystems.elevator.CoralIntake.IntakeSpeed;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.util.KnownLocations;
import frc.robot.util.MercMath;
import frc.robot.util.PathUtils;
import frc.robot.util.ReefscapeUtils;
import frc.robot.util.TargetUtils;
import frc.robot.util.ReefscapeUtils.BranchSide;
import frc.robot.util.ReefscapeUtils.RobotZone;

public class DriveCommands {
    /**
    * @param  : Drivetrain, Suppliers
    * @return  :Drives based on the Joystick values given, as a Run Command
    */
    public static Command joyStickDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> angularSpeedSupplier, Drivetrain drivetrain) {
        return new RunCommand(
            () -> drivetrain.drive(
              -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(angularSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)))
          , drivetrain);
    }
    /** 
    * @param : Suppliers (xSpeed, ySpeed, angularSpeed), Drivetrain, Boolean (Field Relative)
    * @return : RunCommand
    */
    public static Command joyStickDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> angularSpeedSupplier, boolean fieldRelative, Drivetrain drivetrain) {
        return new RunCommand(
            () -> drivetrain.drive(
              -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(angularSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              false)
          , drivetrain);
    } 
    // /**
    // * @param : Supplier (xSpeed, ySpeed, heading), Drivetrain
    // * @return : Returns PID Command (Pose, Rotation, and Heading Degrees)
    // */
    // public static Command targetDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, DoubleSupplier headingSupplier, Drivetrain drivetrain) {
    //     return new PIDCommand(
    //         drivetrain.getRotationalController(),
    //         () -> drivetrain.getPose().getRotation().getDegrees(),
    //         headingSupplier,
    //         (angularSpeed) -> drivetrain.drive(
    //           -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
    //           -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
    //         angularSpeed),
    //         drivetrain);
    // }
    /**
    * @param : Supplier (xSpeed, ySpeed), 
    * @return : Returns a RunCommand telling the drivetrain to drive and calculates heading degrees required to target reef
    */
    public static Command targetDriveToReef(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
        Supplier<Double> heading = () -> ReefscapeUtils.getTargetHeadingToReef(drivetrain.getPose());
        return new RunCommand(
                () -> drivetrain.drive(
                  -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), heading.get()),
                  true)
              , drivetrain);
    }
    /**
    * @param : Supplier (xSpeed, ySpeed), 
    * @return : Returns a RunCommand telling the drivetrain to drive and calculates heading degrees required to target reef
    */
    public static Command targetDriveToStation(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
        Supplier<Double> heading = () -> drivetrain.getTargetHeadingToStation();
        return new RunCommand(
                () -> drivetrain.drive(
                  -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), heading.get()),
                  true)
              , drivetrain);
    }

    public static Command targetDriveToClosestAlgaePickUp(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
        Supplier<Double> heading = () -> drivetrain.getTargetHeadingToReef();
        return new RunCommand(
                () -> drivetrain.drive(
                  -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), Rotation2d.fromDegrees(heading.get()).rotateBy(Rotation2d.fromDegrees(180)).getDegrees()),
                  true)
              , drivetrain);
    }
    /**
    * @param : Drivetrain, Pose2d Supplier 
    * @return : Outputs a Run Command 
    */
    public static Command goToPose(Drivetrain drivetrain, Supplier<Pose2d> desiredPose) {
        return new RunCommand(
            () -> drivetrain.drive(
              drivetrain.getXController().calculate(drivetrain.getPose().getX(), desiredPose.get().getX()),
              drivetrain.getYController().calculate(drivetrain.getPose().getY(), desiredPose.get().getY()),
              drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), desiredPose.get().getRotation().getDegrees()),
              true, false,
              () -> drivetrain.getPose().getRotation())
          , drivetrain);//.until(() -> drivetrain.isAtPose(desiredPose.get()))
          //this seems like a bad idea to comment this out
    }
    /**
    * @param : Drivetrain 
    * Preffered branch (Human Input)
    * SELECT BRANCH AND ZONE BEFORE USING
    * @return : Runs a sequential command which makes it go through all the pose options, using the sensor to guide its path
    */
    public static Command goToPreferredBranch(Drivetrain drivetrain, RobotZone zone, Pose2d branch) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.setIgnoreBackCam(true)),
            PathUtils.getPathToPose(branch, () -> 0.5),
            new InstantCommand(() -> drivetrain.setIgnoreBackCam(false))
        );
    }
    /**
    * @param : Drivetrain
    * Sensor Input
    * @return : Outputs a Run Command and calculates if the robot is too far left or right it will adjust itself
    * SELECT BRANCH AND ZONE BEFORE USING
    */
    // public static Command alignwithSensors(Drivetrain drivetrain, Supplier<RobotZone> zone, Supplier<BranchSide> side) {

    //     Supplier<DistanceSensors> proximitySensor = () -> side.get() == BranchSide.LEFT ? drivetrain.getLeftSensors() : drivetrain.getRightSensors();

    //     // proximitySensor = () -> zone.get() == RobotZone.BARGE || zone.get() == RobotZone.BARGE_LEFT || zone.get() == RobotZone.BARGE_RIGHT ?
    //     //                         side.get() == BranchSide.LEFT ? drivetrain.getRightSensors() : drivetrain.getLeftSensors() :
    //     //                         side.get() == BranchSide.LEFT ? drivetrain.getLeftSensors() : drivetrain.getRightSensors();

    //     // Positive Y moves right, negative Y moves left
    //     Supplier<Double> yDirection = () -> proximitySensor.get().isTooFarLeft(zone, side) ? -1.0 : 1.0;

    //     return goCloserToReef(drivetrain, zone, side).andThen(new RunCommand(
    //         () -> drivetrain.drive(
    //           0.0, 
    //           yDirection.get() * 0.05,
    //           0.0,
    //           false)
    //     ).until(() -> proximitySensor.get().isAtReefSide())
    //         .andThen(new RunCommand(() -> drivetrain.drive(0.0,0.0,0.0))).until(() -> drivetrain.isNotMoving()));
    // }
    /**
    * SELECT SIDE AND CORAL STATION BEFORE USING
    * @param : Drivetrain 
    * @return : Returns a Sequential Command Group, and starts goToPose command to go to the prefered Station
    */
    public static Command goToCoralStation(Drivetrain drivetrain, Pose2d station) {
        return new SequentialCommandGroup(
            PathUtils.getPathToPose(station, () -> 0.5),
            goToPose(drivetrain, () -> station).until(() -> drivetrain.isAtPose(station)));
    }
    /**
    * input: Sensors, Human Input (Preferred Branch), Human Input (Preferred Level)
    * SELECT BRANCH AND ZONE BEFORE USING
    * @param : Drivetrain, Elevator, and Coral Intake
    * @return : Returns Sequential Command Group 
    */
    public static Command driveAndScoreAtBranch(Drivetrain drivetrain, Supplier<Pose2d> branch, Elevator elevator, CoralIntake coralIntake) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                goToPose(drivetrain, branch),
                new ConditionalCommand(
                    new RunCommand(() -> elevator.setPosition(() -> ReefscapeUtils.getPreferredLevel()), elevator), 
                    new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.LEVEL3), elevator), 
                    () -> ReefscapeUtils.getPreferredLevel() != ElevatorPosition.LEVEL4)
            ).until(() -> drivetrain.isAtPose(branch.get(), 0.0254)),
            new ParallelCommandGroup(
                goToPose(drivetrain, branch),
                new RunCommand(() -> elevator.setPosition(() -> ReefscapeUtils.getPreferredLevel()), elevator)
            ).until(() -> elevator.isInPosition()),
            new ParallelCommandGroup(
                new InstantCommand(() -> coralIntake.setEjecting(true)),
                new RunCommand(() -> elevator.setPosition(() -> ReefscapeUtils.getPreferredLevel()), elevator)
            ).until(() -> coralIntake.noCoralPresent()),
            new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.HOME), elevator).until(() -> elevator.isSafe())
        );
    }

    // public static Command scoreAtBranch(Drivetrain drivetrain, Supplier<RobotZone> zone, Supplier<BranchSide> side, Elevator elevator, CoralIntake coralIntake) {
    //     return new SequentialCommandGroup(
    //         new ParallelCommandGroup(
    //             // alignwithSensors(drivetrain, zone, side),
    //             //TODO: pid to pose here?
    //             new RunCommand(() -> elevator.setPosition(() -> ReefscapeUtils.getPreferredLevel()), elevator).until(() -> elevator.isInPosition())
    //         ),
    //         new ParallelCommandGroup(
    //             new RunCommand(() -> drivetrain.drive(0.0, 0.0, 0.0), drivetrain),
    //             // new RunCommand(() -> coralIntake.spitCoral(), coralIntake),
    //             new RunCommand(() -> elevator.setPosition(() -> ReefscapeUtils.getPreferredLevel()), elevator)
    //         ).until(() -> coralIntake.noCoralPresent()),
    //         new InstantCommand(() -> coralIntake.setEjecting(false)),
    //         new ConditionalCommand(
    //             new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.HOME), elevator),
    //             new InstantCommand(),
    //             () -> elevator.isAboveSafePosition() 
    //         )
    //     );
    // }

    /**
    * Input : Elevator Set Position, Coral Intake, Coral Intake Detection
    * @param : Drivetrain, Elevator, Coral Intake
    * @return : Command to drive to coral station and put elevator in position
    */
    public static Command getCoralFromStation(Drivetrain drivetrain, Elevator elevator, CoralIntake coralIntake, Supplier<Pose2d> station) {
        return new ParallelCommandGroup(
            goToCoralStation(drivetrain, station.get()).until(() -> coralIntake.hasCoralEntered()),
            new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.HOME), elevator)
        );
    }//untested
    /**
    * Input : Pose, Rotation, Degrees, Heading. 
    * SELECT BRANCH AND ZONE BEFORE USING
    * @param : Drivetrain, Joystick Supplier 
    * @return : Calculates necessary X,Y, and Rotational degrees required to align for algae
    */
    public static Command pickUpAlgaeInCurrentZone(Drivetrain drivetrain, Elevator elevator, AlgaeIntake intake, AlgaeArticulator articulator) {
        return new ParallelCommandGroup(
            // new SequentialCommandGroup(
            //     goToPose(drivetrain, () -> ReefscapeUtils.getCurrentZoneSafeAlgaePoint()).until(() -> drivetrain.isAtPose(ReefscapeUtils.getCurrentZoneSafeAlgaePoint())),
            //     goToPose(drivetrain, () -> ReefscapeUtils.getCurrentZoneScoreAlgaePoint()).until(() -> drivetrain.isAtPose(ReefscapeUtils.getCurrentZoneScoreAlgaePoint()))
            // ),
            ElevatorCommands.getAlgaeRemovalCommand(elevator, articulator, () -> ReefscapeUtils.getCurrentRobotZone())
        );
    }
    /**
    * SELECT BRANCH AND ZONE BEFORE USING
    * @param : Drivetrain
    * @return : Run Command 
    */
    // public static Command goCloserToReef(Drivetrain drivetrain, Supplier<RobotZone> zone, Supplier<BranchSide> side) {
    //     Supplier<DistanceSensors> proximitySensor = () -> side.get() == BranchSide.LEFT ? drivetrain.getRightSensors() : drivetrain.getLeftSensors(); // this is the other inner sensor
    //     // proximitySensor = () -> zone.get() == RobotZone.BARGE || zone.get() == RobotZone.BARGE_LEFT || zone.get() == RobotZone.BARGE_RIGHT ?
    //     //                         side.get() == BranchSide.LEFT ? drivetrain.getLeftSensors() : drivetrain.getRightSensors() :
    //     //                         side.get() == BranchSide.LEFT ? drivetrain.getRightSensors() : drivetrain.getLeftSensors();

    //     // Positive X moves closer, negative X moves away
    //     Supplier<Double> xDirection = () -> proximitySensor.get().isTooFarAway() ? 1.0 : -1.0;

    //     return new RunCommand(
    //         () -> drivetrain.drive(
    //           xDirection.get() * 0.1, // away from reef num
    //           0.0,
    //           0.0,
    //           false)
    //       , drivetrain).until(() -> !proximitySensor.get().isTooFarAway());
    // }

    public static Command lockToProcessor(Drivetrain drivetrain, Supplier<Double> ySpeedSupplier, Elevator elevator, AlgaeArticulator articulator) {
        Supplier<KnownLocations> locs = () -> KnownLocations.getKnownLocations();
        return new ParallelCommandGroup(
                new RunCommand(() -> drivetrain.drive(
                        drivetrain.getXController().calculate(drivetrain.getPose().getX(), locs.get().processor.getX()),
                        -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                        drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(),
                                locs.get().processor.getRotation().getDegrees()),
                        true), drivetrain),
                // new ParallelCommandGroup(
                // new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT),
                // articulator),
                new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.PROCESSOR), elevator)
        // )
        );
    }

    // public static Command goCloserWithBackLaserCan(Drivetrain drivetrain) {
    //     DistanceSensors proximitySensor = drivetrain.getBackSensor();

    //     Supplier<Double> invert = () -> proximitySensor.isTooFarAway() ? -1.0 : 1.0;

    //     return new RunCommand(
    //         () -> drivetrain.drive(
    //           invert.get() * 0.1, // away from reef num
    //           0.0,
    //           0.0,
    //           false)
    //       , drivetrain).until(() -> !proximitySensor.isTooFarAway());
    // }
    /**
     * Construct a command that will follow a path provided when the command initializes.
     * @param pathSupplier Supplier that provides the path to follow.
     * @param drivetrain Drivetrain subsystem that will follow the path
     * @return The Druvetrain path follows form path supplier
     */
    public static Command followPath(Supplier<PathPlannerPath> pathSupplier, Drivetrain drivetrain) {
        return drivetrain.defer(() -> AutoBuilder.followPath(pathSupplier.get()));
    }
   


}

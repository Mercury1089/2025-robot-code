package frc.robot.commands;

import java.sql.Driver;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SWERVE;
import frc.robot.sensors.DistanceSensors;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
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
        Supplier<Double> heading = () -> drivetrain.getTargetHeadingToReef();
        return new InstantCommand(() -> drivetrain.getRotationalController().reset(drivetrain.getPose().getRotation().getDegrees())).andThen(
            new RunCommand(
                () -> drivetrain.drive(
                  -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), heading.get()),
                  true)
              , drivetrain) 
        ) ;
    }
    /**
    * @param : Supplier (xSpeed, ySpeed), 
    * @return : Returns a RunCommand telling the drivetrain to drive and calculates heading degrees required to target reef
    */
    public static Command targetDriveToStation(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
        Supplier<Double> heading = () -> drivetrain.getTargetHeadingToStation();
        return new InstantCommand(() -> drivetrain.getRotationalController().reset(drivetrain.getPose().getRotation().getDegrees())).andThen(
            new RunCommand(
                () -> drivetrain.drive(
                  -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
                  drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), heading.get()),
                  true)
              , drivetrain) 
        ) ;
    }
    /**
    * @param : Drivetrain, Pose2d Supplier 
    * @return : Outputs a Run Command 
    */
    public static Command goToPose(Drivetrain drivetrain, Supplier<Pose2d> desiredPose) {
        return new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.getRotationalController().reset(drivetrain.getRotation().getDegrees())),
            new InstantCommand(() -> drivetrain.getXController().reset(drivetrain.getPose().getX())),
            new InstantCommand(() -> drivetrain.getYController().reset(drivetrain.getPose().getY()))
        ).andThen(
            new RunCommand(
            () -> drivetrain.drive(
              drivetrain.getXController().calculate(drivetrain.getPose().getX(), desiredPose.get().getX()),
              drivetrain.getYController().calculate(drivetrain.getPose().getY(), desiredPose.get().getY()),
              drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), desiredPose.get().getRotation().getDegrees()),
              true)
          , drivetrain).until(() -> drivetrain.isAtPose(desiredPose.get()))
        );
    }
    /**
    * @param : Drivetrain 
    * Preffered branch (Human Input)
    * SELECT BRANCH AND ZONE BEFORE USING
    * @return : Runs a sequential command which makes it go through all the pose options, using the sensor to guide its path
    */
    public static Command goToPreferredBranch(Drivetrain drivetrain) {
        return new SequentialCommandGroup(
            ReefscapeUtils.getPathToPreferredBranch(),
            goToPose(drivetrain, () -> ReefscapeUtils.getPreferredBranch()),
            alignwithSensors(drivetrain)
        );
    }
    /**
    * @param : Drivetrain
    * Sensor Input
    * @return : Outputs a Run Command and calculates if the robot is too far left or right it will adjust itself
    * SELECT BRANCH AND ZONE BEFORE USING
    */
    public static Command alignwithSensors(Drivetrain drivetrain) {

        Supplier<DistanceSensors> proximitySensor;

        Supplier<RobotZone> currentPref = () -> ReefscapeUtils.preferredZone();

        proximitySensor = () -> currentPref.get() == RobotZone.BARGE || currentPref.get() == RobotZone.BARGE_LEFT || currentPref.get() == RobotZone.BARGE_RIGHT ?
                                ReefscapeUtils.branchSide() == BranchSide.LEFT ? drivetrain.getRightSensors() : drivetrain.getLeftSensors() :
                                ReefscapeUtils.branchSide() == BranchSide.LEFT ? drivetrain.getLeftSensors() : drivetrain.getRightSensors();

        Supplier<Double> invert = () -> !proximitySensor.get().isTooFarLeft() ? 1.0 : -1.0;

        return new RunCommand(
            () -> drivetrain.drive(
              0.0, 
              invert.get() * 0.1,
              0.0,
              false)
        ).until(() -> proximitySensor.get().isAtReefSide());
    }
    /**
    * @param : Drivetrain 
    * @return : Returns a Sequential Command Group, and starts goToPose command to go to the prefered Station
    * SELECT SIDE AND CORAL STATION BEFOR USING
    */
    public static Command goToPreferredCoralStation(Drivetrain drivetrain) {
        return new SequentialCommandGroup(
            ReefscapeUtils.getPathToPreferredCoralStation(),
            goToPose(drivetrain, () -> ReefscapeUtils.getPreferredCoralStation()).until(() -> drivetrain.isAtPreferredCoralStation())
        );
    }
    /**
    * @param : Drivetrain, Elevator, and Coral Intake
    * input: Sensors, Human Input (Preferred Branch), Human Input (Preferred Level)
    * SELECT BRANCH AND ZONE BEFORE USING
    * @return : Returns Sequential Command Group 
    */
    public static Command scoreAtPreferredBranch(Drivetrain drivetrain, Elevator elevator, CoralIntake coralIntake) {
        return new SequentialCommandGroup(
            ReefscapeUtils.getPathToPreferredBranch(),
            goToPose(drivetrain, () -> ReefscapeUtils.getPreferredBranch()),
            new ParallelCommandGroup(
                alignwithSensors(drivetrain),
                new RunCommand(() -> elevator.setPosition(ReefscapeUtils.getPreferredLevel()), elevator)
            ).until(() -> elevator.isAtPosition(ReefscapeUtils.getPreferredLevel())),
            new RunCommand(() -> coralIntake.spitCoral(), coralIntake).until(() -> coralIntake.noCoralPresent())
        );
    }
    /**
    * @param : Drivetrain, Elevator, Coral Intake
    * @return : Returns Sequential Command Group 
    * Input : Elevator Set Position, Coral Intake, Coral Intake Detection
    */
    public static Command getCoralFromStation(Drivetrain drivetrain, Elevator elevator, CoralIntake coralIntake) {
        return new SequentialCommandGroup(
            goToPreferredCoralStation(drivetrain),
            new RunCommand(() -> elevator.setPosition(ElevatorPosition.CORAL_STATION), elevator).until(() -> elevator.isAtPosition(ElevatorPosition.CORAL_STATION)),
            new RunCommand(() -> coralIntake.intakeCoral(), coralIntake).until(() -> coralIntake.hasCoralEntered())
        );
    }
    /**
    * @param : Drivetrain, Joystick Supplier 
    * @return : Calculates necssary X,Y, and Rotational degrees required to align for algae
    * Input : Pose, Rotation, Degrees, Heading
    * SELECT BRANCH AND ZONE BEFORE USING
    */
    public static Command pickUpAlgaeInCurrentZone(Drivetrain drivetrain) {
        Supplier<Double> heading = () -> drivetrain.getTargetHeadingToReef();
        return new SequentialCommandGroup(
          goToPose(drivetrain, () -> ReefscapeUtils.getCurrentZoneSafeAlgaePoint()).until(() -> drivetrain.isAtPose(ReefscapeUtils.getCurrentZoneSafeAlgaePoint())),
          goToPose(drivetrain, () -> ReefscapeUtils.getCurrentZoneScoreAlgaePoint()).until(() -> drivetrain.isAtPose(ReefscapeUtils.getCurrentZoneScoreAlgaePoint())),
          goCloserToReefForAlgae(drivetrain)
        );
    }
    /**
    * @param : Drivetrain
    * @return : Run Command 
    * SELECT BRANCH AND ZONE BEFORE USING
    */
    public static Command goCloserToReefForAlgae(Drivetrain drivetrain) {
        Supplier<DistanceSensors> proximitySensor = () -> drivetrain.getLeftSensors();
        Supplier<Double> invert = () -> proximitySensor.get().isTooFarAwayFromReef() ? 1.0 : -1.0;
        return new RunCommand(
            () -> drivetrain.drive(
              invert.get() * 0.1, // away from reef num
              0.0,
              0.0,
              false)
          , drivetrain).until(() -> !proximitySensor.get().isTooFarAwayFromReef());
    }

    
    

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

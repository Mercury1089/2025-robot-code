package frc.robot.commands;

import java.sql.Driver;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
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

public class DriveCommands {
    /**
    * @param  :Joy stick values as 3 doubles as well as drivetrain
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
    * @param :Input are the 3 double supplier value, drivetrain, and values of field locations or relativity
    * @return :Drives based on joytick value inputs, as well as field relative data as a RunCommand
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
    /**
    * @param : Input is the speed, target, and heading suppliers which are doubles, as well as drivetrain
    * @return : Returns PID Command (Pose, Rotation, and Heading Degrees)
    */
    public static Command targetDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, DoubleSupplier headingSupplier, Drivetrain drivetrain) {
        return new PIDCommand(
            drivetrain.getRotationalController(),
            () -> drivetrain.getPose().getRotation().getDegrees(),
            headingSupplier,
            (angularSpeed) -> drivetrain.drive(
              -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
            angularSpeed),
            drivetrain);
    }
    /**
    * @param : Input is 2 double suppliers and the drivetrain
    * @return : Returns a RunCommand telling the drivetrain to drive and calculates heading degrees required to target reef
    */
    public static Command targetDriveToReef(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Drivetrain drivetrain) {
        Supplier<Double> heading = () -> drivetrain.getTargetHeadingToReef();
        return new RunCommand(
            () -> drivetrain.drive(
              -MercMath.squareInput(MathUtil.applyDeadband(xSpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              -MercMath.squareInput(MathUtil.applyDeadband(ySpeedSupplier.get(), SWERVE.JOYSTICK_DEADBAND)),
              drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), heading.get()),
              true)
          , drivetrain);
    }
    /**
    * @param : Drivetrain, Pose2d Supplier 
    * @return : Outputs a Run Command and calculates the required x,y, rotation to get to the deserired pose
    */
    public static Command goToPose(Drivetrain drivetrain, Supplier<Pose2d> desiredPose) {
        return new RunCommand(
            () -> drivetrain.drive(
              drivetrain.getXController().calculate(drivetrain.getPose().getX(), desiredPose.get().getX()),
              drivetrain.getYController().calculate(drivetrain.getPose().getY(), desiredPose.get().getY()),
              drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), desiredPose.get().getRotation().getDegrees()),
              true)
          , drivetrain).until(() -> drivetrain.isAtPose(desiredPose.get()));
    }
    /**
    * @param : Drivetrain 
    * Preffered branch (Human Input)
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
    * @param : Drivetrain, 
    * Sensor Input
    * @return : Outputs a Run Command and calculates if the robot is too far left or right it will adjust itself
    */
    public static Command alignwithSensors(Drivetrain drivetrain) {
        Supplier<DistanceSensors> proximitySensor = () -> ReefscapeUtils.branchSide() == BranchSide.LEFT ?
                                                        drivetrain.getLeftSensors() :
                                                        drivetrain.getRightSensors();
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
    * @param : Drivetrain, Joystick Supplier (Double)
    * @return : Calculates necssary X,Y, and Rotational degrees required to align for algae
    * Input : Pose, Rotation, Degrees, Heading
    */
    public static Command alignToReefSideForAlgae(Drivetrain drivetrain, Supplier<Double> xJoystickSupplier) {
        Supplier<Double> heading = () -> drivetrain.getTargetHeadingToReef();
        return new RunCommand(
            () -> drivetrain.drive(
              xJoystickSupplier.get(), // make it so if any input, this speed is some constant value
              drivetrain.getYController().calculate(drivetrain.getPose().getY(), 0.0), //TODO: FIGURE THIS OUT BEFORE USING
              drivetrain.getRotationalController().calculate(drivetrain.getPose().getRotation().getDegrees(), heading.get()),
              false)
          , drivetrain);
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

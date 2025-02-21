// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DS_USB;
import frc.robot.Constants.JOYSTICK_BUTTONS;
import frc.robot.commands.Autons;
import frc.robot.commands.DriveCommands;
import frc.robot.sensors.DistanceSensors;
import frc.robot.subsystems.RobotModeLEDs;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.AlgaeArticulator;
import frc.robot.subsystems.elevator.AlgaeIntake;
import frc.robot.subsystems.elevator.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.AlgaeIntake.AlgaeSpeed;
import frc.robot.subsystems.elevator.CoralIntake.IntakeSpeed;
import frc.robot.util.KnownLocations;
import frc.robot.util.ReefscapeUtils;
import frc.robot.util.TargetUtils;
import frc.robot.util.ReefscapeUtils.BranchSide;
import frc.robot.util.ReefscapeUtils.CoralStation;
import frc.robot.util.ReefscapeUtils.RobotZone;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private CommandJoystick rightJoystick, leftJoystick;
  private CommandXboxController gamepad;

  private Trigger left1, left2, left3, left4, left5, left6, left7, left8, left9, left10, left11;
  private Trigger right1, right2, right3, right4, right5, right6, right7, right8, right9, right10, right11;
  private Trigger gamepadA, gamepadB, gamepadX, gamepadY, gamepadRB, gamepadLB, gamepadL3, gamepadBack, 
  gamepadStart, gamepadLeftStickButton, gamepadRightStickButton, gamepadLT, gamepadRT, gamepadPOVDown, gamepadPOVUpLeft, 
  gamepadPOVUp, gamepadPOVUpRight, gamepadPOVLeft, gamepadPOVRight, gamepadPOVDownRight, gamepadPOVDownLeft;

  private GenericHID gamepadHID;
  private Supplier<Double> gamepadLeftX, gamepadLeftY, gamepadRightX, gamepadRightY, rightJoystickX, rightJoystickY, leftJoystickX, leftJoystickY;

  private Autons auton;
  private Drivetrain drivetrain;
  private Elevator elevator; 
  private CoralIntake coralIntake;
  private AlgaeIntake algaeIntake;
  private RobotModeLEDs leds;
  private AlgaeArticulator articulator;

  private double manualThreshold = 0.2;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // buttons & bindings
    leftJoystick = new CommandJoystick(DS_USB.LEFT_STICK);
    rightJoystick = new CommandJoystick(DS_USB.RIGHT_STICK);
    gamepad = new CommandXboxController(DS_USB.GAMEPAD);
    gamepadHID = new GenericHID(DS_USB.GAMEPAD);
    configureBindings();

    drivetrain = new Drivetrain();
    drivetrain.setDefaultCommand(DriveCommands.joyStickDrive(leftJoystickY, leftJoystickX, rightJoystickX, drivetrain));
    drivetrain.resetGyro();

    elevator = new Elevator(); 
    elevator.setDefaultCommand(new RunCommand(() -> elevator.setSpeed(gamepadLeftY), elevator));
    // gamepadA.onTrue(new RunCommand(() -> elevator.setPosition(Elevator.ElevatorPosition.LEVEL1), elevator));
    // gamepadB.onTrue(new RunCommand(() -> elevator.setPosition(Elevator.ElevatorPosition.LEVEL2), elevator));
    // gamepadY.onTrue(new RunCommand(() -> elevator.setPosition(Elevator.ElevatorPosition.LEVEL3), elevator));
    // gamepadX.onTrue(new RunCommand(() -> elevator.setPosition(Elevator.ElevatorPosition.LEVEL4), elevator));

    coralIntake = new CoralIntake();
    coralIntake.setDefaultCommand(new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.STOP), coralIntake));

    Trigger hasCoral = new Trigger(() -> coralIntake.hasCoral());
    Trigger hasCoralEntered = new Trigger(() -> coralIntake.hasCoralEntered());
    Trigger ejecting = new Trigger(() -> coralIntake.getEjecting());

    (hasCoral.negate().and(hasCoralEntered)).and(ejecting.negate()).onTrue(
      new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.SLOW_INTAKE), coralIntake)
    );

    (hasCoral.and(hasCoralEntered.negate())).and(ejecting.negate()).onTrue(
      new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.BRING_BACK), coralIntake)
    );
    
    (hasCoral.and(hasCoralEntered)).or(hasCoral.negate().and(hasCoralEntered.negate())).and(ejecting.negate()).onTrue(
      new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.STOP), coralIntake)
    );

    gamepadA.and(hasCoral.and(hasCoralEntered)).onTrue(new RunCommand(() -> coralIntake.spitCoral(), coralIntake).until(() -> coralIntake.noCoralPresent()).andThen(
      new InstantCommand(() -> coralIntake.setEjecting(false))
    ));

    algaeIntake = new AlgaeIntake();
    algaeIntake.setDefaultCommand(new RunCommand(() -> algaeIntake.setSpeed(AlgaeSpeed.STOP), algaeIntake));
    // gamepadB.onTrue(new RunCommand(() -> algaeIntake.intakeAlgae(), algaeIntake));
    // gamepadA.onTrue(new RunCommand(() -> algaeIntake.setSpeed(AlgaeSpeed.OUTTAKE), algaeIntake).withTimeout(1.0));

    articulator = new AlgaeArticulator(elevator);
    elevator.setArticulator(articulator);
    
    leds = new RobotModeLEDs();

    auton = new Autons(drivetrain);

    left10.onTrue(new InstantCommand(() -> drivetrain.resetGyro(), drivetrain).ignoringDisable(true));
    
    Trigger takeControl = new Trigger(() -> (Math.abs(leftJoystickY.get()) > manualThreshold || Math.abs(leftJoystickX.get()) > manualThreshold || Math.abs(rightJoystickX.get()) > manualThreshold));
    Trigger fidoOn = new Trigger(() -> leds.isFIDOEnabled());
    Trigger fidoOff = new Trigger(() -> !leds.isFIDOEnabled());

    takeControl.and(fidoOn).onTrue(drivetrain.getDefaultCommand());
    
    right1.onTrue(DriveCommands.targetDriveToStation(leftJoystickY, leftJoystickX, drivetrain)); //TODO: Change logic to lock to nearest station
    right3.onTrue(DriveCommands.targetDriveToReef(leftJoystickY, leftJoystickX, drivetrain));

    right4.onTrue(
      DriveCommands.goToPose(drivetrain, () -> ReefscapeUtils.getCurrentZoneLeftBranch()).andThen(
      /* DriveCommands.scoreAtCurrentZoneBranch(drivetrain, elevator, coralIntake) */
      DriveCommands.alignwithSensors(drivetrain)));
    right5.onTrue(
      DriveCommands.goToPose(drivetrain, () -> ReefscapeUtils.getCurrentZoneRightBranch()).andThen(
      /* DriveCommands.scoreAtCurrentZoneBranch(drivetrain, elevator, coralIntake) */
      DriveCommands.alignwithSensors(drivetrain)));

    left1.whileTrue(DriveCommands.lockToProcessor(drivetrain, leftJoystickX));
    left3.whileTrue(DriveCommands.pickUpAlgaeInCurrentZone(drivetrain));
    left6.onTrue(new InstantCommand(() -> leds.enableFIDO()));
    left7.onTrue(new InstantCommand(() -> leds.disableFIDO()));

    gamepadLB.and(fidoOn).onTrue(DriveCommands.goToPreferredBranch(drivetrain));
    // gamepadLB.and(fidoOn).onTrue(DriveCommands.alignwithSensors(drivetrain));
    gamepadRB.and(fidoOn).onTrue(DriveCommands.goToPreferredCoralStation(drivetrain));

    gamepadA.onTrue(new InstantCommand(() -> ReefscapeUtils.changepreferredZone(RobotZone.CLOSE)));
    gamepadB.onTrue(new InstantCommand(() -> ReefscapeUtils.changepreferredZone(RobotZone.CLOSE_RIGHT)));
    gamepadY.onTrue(new InstantCommand(() -> ReefscapeUtils.changepreferredZone(RobotZone.BARGE_RIGHT)));
    gamepadX.onTrue(new InstantCommand(() -> ReefscapeUtils.changepreferredZone(RobotZone.BARGE)));

    gamepadPOVRight.onTrue(new InstantCommand(() -> ReefscapeUtils.changepreferredCoralStation(CoralStation.OUTSIDERIGHT)));
    gamepadPOVLeft.onTrue(new InstantCommand(() -> ReefscapeUtils.changepreferredCoralStation(CoralStation.INSIDERIGHT)));

    left6.onTrue(new InstantCommand(() -> ReefscapeUtils.changepreferredBranch(BranchSide.LEFT)));
    left7.onTrue(new InstantCommand(() -> ReefscapeUtils.changepreferredBranch(BranchSide.RIGHT)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        left1 = leftJoystick.button(JOYSTICK_BUTTONS.BTN1);
        left2 = leftJoystick.button(JOYSTICK_BUTTONS.BTN2);
        left3 = leftJoystick.button(JOYSTICK_BUTTONS.BTN3);
        left4 = leftJoystick.button(JOYSTICK_BUTTONS.BTN4);
        left5 = leftJoystick.button(JOYSTICK_BUTTONS.BTN5);
        left6 = leftJoystick.button(JOYSTICK_BUTTONS.BTN6);
        left7 = leftJoystick.button(JOYSTICK_BUTTONS.BTN7);
        left8 = leftJoystick.button(JOYSTICK_BUTTONS.BTN8);
        left9 = leftJoystick.button(JOYSTICK_BUTTONS.BTN9);
        left10 = leftJoystick.button(JOYSTICK_BUTTONS.BTN10);
        left11 = leftJoystick.button(JOYSTICK_BUTTONS.BTN11);

        right1 = rightJoystick.button(JOYSTICK_BUTTONS.BTN1);
        right2 = rightJoystick.button(JOYSTICK_BUTTONS.BTN2);
        right3 = rightJoystick.button(JOYSTICK_BUTTONS.BTN3);
        right4 = rightJoystick.button(JOYSTICK_BUTTONS.BTN4);
        right5 = rightJoystick.button(JOYSTICK_BUTTONS.BTN5);
        right6 = rightJoystick.button(JOYSTICK_BUTTONS.BTN6);
        right7 = rightJoystick.button(JOYSTICK_BUTTONS.BTN7);
        right8 = rightJoystick.button(JOYSTICK_BUTTONS.BTN8);
        right9 = rightJoystick.button(JOYSTICK_BUTTONS.BTN9);
        right10 = rightJoystick.button(JOYSTICK_BUTTONS.BTN10);
        right11 = rightJoystick.button(JOYSTICK_BUTTONS.BTN11);

        gamepadA = gamepad.a();
        gamepadB = gamepad.b();
        gamepadX = gamepad.x();
        gamepadY = gamepad.y();
        gamepadRB = gamepad.rightBumper();
        gamepadLB = gamepad.leftBumper();
        gamepadBack = gamepad.back();
        gamepadStart = gamepad.start();
        gamepadLeftStickButton = gamepad.leftStick();
        gamepadRightStickButton = gamepad.rightStick();
        gamepadLT = gamepad.leftTrigger();
        gamepadRT = gamepad.rightTrigger();
        
        gamepadPOVDown = gamepad.povDown();
        gamepadPOVUpLeft = gamepad.povUpLeft();
        gamepadPOVUp = gamepad.povUp();
        gamepadPOVUpRight = gamepad.povUpRight();
        gamepadPOVLeft = gamepad.povLeft();
        gamepadPOVRight = gamepad.povRight();
        gamepadPOVDownRight = gamepad.povDownRight();
        gamepadPOVDownLeft = gamepad.povDownLeft();

        gamepadLeftX = () -> gamepad.getLeftX();
        gamepadRightX = () -> gamepad.getRightX();
        gamepadLeftY = () -> -gamepad.getLeftY();
        gamepadRightY = () -> -gamepad.getRightY();

        leftJoystickX = () -> leftJoystick.getX();
        leftJoystickY = () -> leftJoystick.getY();
        rightJoystickX = () -> rightJoystick.getX();
        rightJoystickY = () -> rightJoystick.getY();


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Autons getAutonomous() {
    // An example command will be run in autonomous
    return auton;
  }
}

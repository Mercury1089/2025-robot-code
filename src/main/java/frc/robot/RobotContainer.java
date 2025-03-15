// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DS_USB;
import frc.robot.Constants.JOYSTICK_BUTTONS;
import frc.robot.commands.Autons;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
// import frc.robot.sensors.DistanceSensors;
import frc.robot.subsystems.RobotModeLEDs;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.AlgaeArticulator;
import frc.robot.subsystems.elevator.AlgaeIntake;
import frc.robot.subsystems.elevator.CoralIntake;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.AlgaeArticulator.ArticulatorPosition;
import frc.robot.subsystems.elevator.AlgaeIntake.AlgaeSpeed;
import frc.robot.subsystems.elevator.CoralIntake.IntakeSpeed;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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
  private CommandGenericHID reefBoard;
  private CommandGenericHID secondEncoderBoard;

  private Trigger left1, left2, left3, left4, left5, left6, left7, left8, left9, left10, left11;
  private Trigger right1, right2, right3, right4, right5, right6, right7, right8, right9, right10, right11;
  private Trigger gamepadA, gamepadB, gamepadX, gamepadY, gamepadRB, gamepadLB, gamepadL3, gamepadBack, 
  gamepadStart, gamepadLeftStickButton, gamepadRightStickButton, gamepadLT, gamepadRT, gamepadPOVDown, gamepadPOVUpLeft, 
  gamepadPOVUp, gamepadPOVUpRight, gamepadPOVLeft, gamepadPOVRight, gamepadPOVDownRight, gamepadPOVDownLeft;

  private Trigger bargeSideLeftBranchBTN,
                  bargeSideRightBranchBTN,
                  rightBargeSideLeftBranchBTN,
                  rightBargeSideRightBranchBTN,
                  closeRightSideRightBranchBTN,
                  closeRightSideLeftBranchBTN,
                  closeSideRightBranchBTN,
                  closeSideLeftBranchBTN,
                  leftCloseSideRightBranchBTN,
                  leftCloseSideLeftBranchBTN,
                  leftBargeSideLeftBranchBTN,
                  leftBargeSideRightBranchBTN;
  private Trigger outerLeftStationBTN,
                  innerLeftStationBTN,
                  outerRightStationBTN,
                  innerRightStationBTN;
  private Trigger level1BTN, level2BTN, level3BTN, level4BTN;
  private Trigger fidoBTN;

  private GenericHID gamepadHID;
  private Supplier<Double> gamepadLeftX, gamepadLeftY, gamepadRightX, gamepadRightY;
  private Supplier<Double> rightJoystickX, rightJoystickY, leftJoystickX, leftJoystickY;

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
    reefBoard = new CommandGenericHID(DS_USB.REEF_BOARD);
    secondEncoderBoard = new CommandGenericHID(DS_USB.SECOND_ENCODER_BOARD);
    gamepadHID = new GenericHID(DS_USB.GAMEPAD);
    configureBindings();

    drivetrain = new Drivetrain();
    drivetrain.setDefaultCommand(DriveCommands.joyStickDrive(leftJoystickY, leftJoystickX, rightJoystickX, drivetrain));
    drivetrain.resetGyro();

    elevator = new Elevator(); 
    elevator.setDefaultCommand(new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.HOME), elevator));

    coralIntake = new CoralIntake();
    coralIntake.setDefaultCommand(new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.STOP), coralIntake));

    /**
     * CORAL MECHANISM TRIGGERS
     */

    Trigger hasCoral = new Trigger(() -> coralIntake.hasCoral());
    Trigger noCoralPresent = new Trigger(() -> coralIntake.noCoralPresent() && !coralIntake.isEjecting());
    Trigger isCoralEntering = new Trigger(() -> coralIntake.isCoralEntering() && !coralIntake.isEjecting());
    Trigger isCoralExiting = new Trigger(() -> coralIntake.isCoralExiting() && !coralIntake.isEjecting());
    Trigger ejecting = new Trigger(() -> coralIntake.isEjecting());

    Trigger readyToScore = new Trigger(() -> elevator.isAtPosition(ReefscapeUtils.getPreferredLevel()) &&
                                            drivetrain.isAtPose(ReefscapeUtils.getCurrentZoneLeftBranch(), 0.0254) && 
                                            drivetrain.isAtPose(ReefscapeUtils.getCurrentZoneRightBranch(), 0.0254));

    readyToScore.and(hasCoral).onTrue(
      new InstantCommand(() -> coralIntake.setEjecting(true))
    );

    isCoralEntering.whileTrue(
      new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.SLOW_INTAKE), coralIntake)
    );

    (isCoralExiting).whileTrue(
      new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.BRING_BACK), coralIntake)
    );
    
    (hasCoral).or(noCoralPresent).whileTrue(
      new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.STOP), coralIntake)
    );

    ejecting.whileTrue(
      new RunCommand(() -> coralIntake.setSpeed(IntakeSpeed.OUTTAKE), coralIntake)
    );

    Trigger fidoOff = new Trigger(() -> !leds.isFIDOEnabled() && DriverStation.isTeleop());

    // gamepadA.and(hasCoral.and(hasCoralEntered)).onTrue(new RunCommand(() -> coralIntake.spitCoral(), coralIntake).until(() -> coralIntake.noCoralPresent()).andThen(
    //   new InstantCommand(() -> coralIntake.setEjecting(false))
    // ));

    algaeIntake = new AlgaeIntake();
    algaeIntake.setDefaultCommand(new RunCommand(() -> algaeIntake.setSpeed(AlgaeSpeed.STOP), algaeIntake));

    articulator = new AlgaeArticulator();
    articulator.setDefaultCommand(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.IN), articulator));

    
    leds = new RobotModeLEDs();

    auton = new Autons(drivetrain, coralIntake, elevator);


    /**
     * MANUAL CONTROL
     */
    left10.onTrue(new InstantCommand(() -> drivetrain.resetGyro(), drivetrain).ignoringDisable(true));
    right2.onTrue(drivetrain.getDefaultCommand());
    right2.onTrue(new RunCommand(() -> elevator.setPosition(() -> ElevatorPosition.HOME), elevator).until(() -> elevator.isInPosition()));
    right8.onTrue(new RunCommand(() -> elevator.setPosition(() -> ReefscapeUtils.getPreferredLevel()), elevator));
    right9.onTrue(new InstantCommand(() -> coralIntake.setEjecting(true), coralIntake));
    right10.onTrue(new RunCommand(() -> elevator.setSpeed(() -> -0.25)));

    /**
     * TARGET DRIVE BUTTONS & AUTO SCORE
     */
    right1.onTrue(DriveCommands.targetDriveToStation(leftJoystickY, leftJoystickX, drivetrain));
    right3.onTrue(DriveCommands.targetDriveToReef(leftJoystickY, leftJoystickX, drivetrain));
    right4.onTrue(DriveCommands.driveAndScoreAtBranch(drivetrain, () -> ReefscapeUtils.getCurrentZoneLeftBranch(), elevator, coralIntake));
    right5.onTrue(DriveCommands.driveAndScoreAtBranch(drivetrain, () -> ReefscapeUtils.getCurrentZoneRightBranch(), elevator, coralIntake));

    // right4.onTrue(
    //   DriveCommands.goToPose(drivetrain, () -> ReefscapeUtils.getCurrentZoneLeftBranch()));
    // right5.onTrue(
    //   DriveCommands.goToPose(drivetrain, () -> ReefscapeUtils.getCurrentZoneRightBranch()));

    /**
     * ALGAE BUTTONS
     */
    left3.whileTrue(DriveCommands.lockToProcessor(drivetrain, leftJoystickX, elevator, articulator));
    left1.whileTrue(ElevatorCommands.getAlgaeRemovalCommand(elevator, articulator, () -> ReefscapeUtils.getCurrentRobotZone()))
      .onFalse(new RunCommand(() -> articulator.setPosition(ArticulatorPosition.OUT), articulator));
    gamepadA.onTrue(new RunCommand(() -> algaeIntake.intakeAlgae(), algaeIntake));
    gamepadX.onTrue(new RunCommand(() -> algaeIntake.setSpeed(AlgaeSpeed.OUTTAKE), algaeIntake));

    /**
     * ELEVATOR LEVELS
     */
    level1BTN.onTrue(new InstantCommand(() -> ReefscapeUtils.changePreferredLevel(ElevatorPosition.LEVEL1)));
    level2BTN.onTrue(new InstantCommand(() -> ReefscapeUtils.changePreferredLevel(ElevatorPosition.LEVEL2)));
    level3BTN.onTrue(new InstantCommand(() -> ReefscapeUtils.changePreferredLevel(ElevatorPosition.LEVEL3)));
    level4BTN.onTrue(new InstantCommand(() -> ReefscapeUtils.changePreferredLevel(ElevatorPosition.LEVEL4)));

    initializeTriggers();
  }

  public void initializeTriggers() {
    Trigger fidoOn = new Trigger(() -> leds.isFIDOEnabled() && DriverStation.isTeleop());
    // Trigger hasCoralEntered = new Trigger(() -> coralIntake.hasCoralEntered()); 
    Trigger hasCoralEntered = new Trigger(() -> coralIntake.hasCoralEntered() && DriverStation.isTeleop());

    closeSideLeftBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.CLOSE, KnownLocations.getKnownLocations().closeSideLeftBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.CLOSE, () -> BranchSide.LEFT, () -> KnownLocations.getKnownLocations().closeSideLeftBranch, elevator, coralIntake)
    );
    closeSideRightBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.CLOSE, KnownLocations.getKnownLocations().closeSideRightBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.CLOSE, () -> BranchSide.RIGHT, () -> KnownLocations.getKnownLocations().closeSideRightBranch, elevator, coralIntake)
    );
    leftCloseSideLeftBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.CLOSE_LEFT, KnownLocations.getKnownLocations().leftCloseSideLeftBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.CLOSE_LEFT, () -> BranchSide.LEFT, () -> KnownLocations.getKnownLocations().leftCloseSideLeftBranch, elevator, coralIntake)
    );
    leftCloseSideRightBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.CLOSE_LEFT, KnownLocations.getKnownLocations().leftCloseSideLeftBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.CLOSE_LEFT, () -> BranchSide.RIGHT, () ->  KnownLocations.getKnownLocations().leftCloseSideRightBranch, elevator, coralIntake)
    );
    closeRightSideLeftBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.CLOSE_RIGHT, KnownLocations.getKnownLocations().rightCloseSideLeftBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.CLOSE_RIGHT, () -> BranchSide.LEFT, () -> KnownLocations.getKnownLocations().closeRightSideLeftBranch, elevator, coralIntake)
    );
    closeRightSideRightBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.CLOSE_RIGHT, KnownLocations.getKnownLocations().rightCloseSideRightBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.CLOSE_RIGHT, () -> BranchSide.RIGHT, () -> KnownLocations.getKnownLocations().closeRightSideRightBranch, elevator, coralIntake)
    );
    bargeSideLeftBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.BARGE, KnownLocations.getKnownLocations().bargeSideLeftBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.BARGE, () -> BranchSide.LEFT, () -> KnownLocations.getKnownLocations().bargeSideLeftBranch, elevator, coralIntake)
    );
    bargeSideRightBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.BARGE, KnownLocations.getKnownLocations().bargeSideRightBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.BARGE, () -> BranchSide.RIGHT, () -> KnownLocations.getKnownLocations().bargeSideRightBranch, elevator, coralIntake)
    );
    leftBargeSideLeftBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.BARGE_LEFT, KnownLocations.getKnownLocations().leftBargeSideLeftBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.BARGE_LEFT, () -> BranchSide.LEFT, () -> KnownLocations.getKnownLocations().leftBargeSideLeftBranch, elevator, coralIntake)
    );
    leftBargeSideRightBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.BARGE_LEFT, KnownLocations.getKnownLocations().leftBargeSideRightBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.BARGE_LEFT, () -> BranchSide.RIGHT, () -> KnownLocations.getKnownLocations().leftBargeSideRightBranch, elevator, coralIntake)
    );
    rightBargeSideLeftBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.BARGE_RIGHT, KnownLocations.getKnownLocations().rightBargeSideLeftBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.BARGE_RIGHT, () -> BranchSide.LEFT, () -> KnownLocations.getKnownLocations().rightBargeSideLeftBranch, elevator, coralIntake)
    );
    rightBargeSideRightBranchBTN.and(fidoOn).and(hasCoralEntered).whileTrue(
      DriveCommands.goToPreferredBranch(drivetrain, RobotZone.BARGE_RIGHT, KnownLocations.getKnownLocations().rightBargeSideRightBranch)
      // DriveCommands.driveAndScoreAtBranch(drivetrain, () -> RobotZone.BARGE_RIGHT, () -> BranchSide.RIGHT, () -> KnownLocations.getKnownLocations().rightBargeSideRightBranch, elevator, coralIntake)
    );

    outerLeftStationBTN.and(fidoOn).and(hasCoralEntered.negate()).whileTrue(
      // DriveCommands.goToCoralStation(drivetrain, KnownLocations.getKnownLocations().leftCoralStationOutside)
      new InstantCommand(() -> ReefscapeUtils.changePreferredCoralStation(CoralStation.OUTSIDELEFT)).andThen(
      DriveCommands.getCoralFromStation(drivetrain, elevator, coralIntake, () -> KnownLocations.getKnownLocations().leftCoralStationOutside)
    ));
    outerRightStationBTN.and(fidoOn).and(hasCoralEntered.negate()).whileTrue(
      // DriveCommands.goToCoralStation(drivetrain, KnownLocations.getKnownLocations().rightCoralStationOutside)  
      new InstantCommand(() -> ReefscapeUtils.changePreferredCoralStation(CoralStation.OUTSIDERIGHT)).andThen(
      DriveCommands.getCoralFromStation(drivetrain, elevator, coralIntake, () -> KnownLocations.getKnownLocations().rightCoralStationOutside)
    ));
    innerLeftStationBTN.and(fidoOn).and(hasCoralEntered.negate()).whileTrue(
      // DriveCommands.goToCoralStation(drivetrain, KnownLocations.getKnownLocations().leftCoralStationInside)
      new InstantCommand(() -> ReefscapeUtils.changePreferredCoralStation(CoralStation.INSIDELEFT)).andThen(
      DriveCommands.getCoralFromStation(drivetrain, elevator, coralIntake, () -> KnownLocations.getKnownLocations().leftCoralStationInside)
    ));
    innerRightStationBTN.and(fidoOn).and(hasCoralEntered.negate()).whileTrue(
      new InstantCommand(() -> ReefscapeUtils.changePreferredCoralStation(CoralStation.OUTSIDERIGHT)).andThen(
      // DriveCommands.goToCoralStation(drivetrain, KnownLocations.getKnownLocations().rightCoralStationInside)
      DriveCommands.getCoralFromStation(drivetrain, elevator, coralIntake, () -> KnownLocations.getKnownLocations().rightCoralStationInside)
    ));

    fidoBTN.onTrue(new InstantCommand(() -> leds.toggleFIDO()));
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

        closeSideLeftBranchBTN = reefBoard.button(1);
        closeSideRightBranchBTN = reefBoard.button(2);
        closeRightSideLeftBranchBTN = reefBoard.button(3);
        closeRightSideRightBranchBTN = reefBoard.button(4);
        rightBargeSideRightBranchBTN = reefBoard.button(5);
        rightBargeSideLeftBranchBTN = reefBoard.button(6);
        bargeSideRightBranchBTN = reefBoard.button(7);
        bargeSideLeftBranchBTN = reefBoard.button(8);
        leftBargeSideRightBranchBTN = reefBoard.button(9);
        leftBargeSideLeftBranchBTN = reefBoard.button(10);
        leftCloseSideLeftBranchBTN = reefBoard.button(11);
        leftCloseSideRightBranchBTN = reefBoard.button(12);

        fidoBTN = secondEncoderBoard.button(10);
        level1BTN = secondEncoderBoard.button(2);
        level2BTN = secondEncoderBoard.button(3);
        level3BTN = secondEncoderBoard.button(4);
        level4BTN = secondEncoderBoard.button(5);
        outerRightStationBTN = secondEncoderBoard.button(6);
        innerRightStationBTN = secondEncoderBoard.button(7);
        innerLeftStationBTN = secondEncoderBoard.button(8);
        outerLeftStationBTN = secondEncoderBoard.button(9);


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

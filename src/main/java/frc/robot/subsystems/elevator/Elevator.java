// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.elevator.AlgaeArticulator.ArticulatorPosition;

public class Elevator extends SubsystemBase {

  /** Creates a new Elevator. */
  public static final int
    ARM_PID_SLOT = 0;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0 / 10.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0;

  private static final float ARM_SOFT_LIMIT_FWD = (float) 120;

  private static final float ARM_SOFT_LIMIT_REV = (float) 50;

  private static final double ANGLE_OFFSET = -3.5;

  public final double GEAR_RATIO = 125.0 / 1.0;
  public final double THRESHOLD_DEGREES = 0.5;

  
  private SparkFlex leftMotor, rightMotor;
  private SparkClosedLoopController elevatorClosedLoopController;
  private RelativeEncoder relativeEncoder;
  private double setPosition;

  private AlgaeArticulator articulator;

  public Elevator() {
    leftMotor = new SparkFlex(CAN.ELEVATOR_LEFT, MotorType.kBrushless);
    rightMotor = new SparkFlex(CAN.ELEVATOR_RIGHT, MotorType.kBrushless);

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    SparkFlexConfig rightConfig = new SparkFlexConfig();

    leftConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .closedLoopRampRate(0.6);
   // leftConfig.absoluteEncoder
    //  .positionConversionFactor(360.0); // multiplied by native units
    // leftConfig.softLimit
    //   .forwardSoftLimitEnabled(true)
    //   .forwardSoftLimit(ARM_SOFT_LIMIT_FWD)
    //   .reverseSoftLimitEnabled(true)
    //   .reverseSoftLimit(ARM_SOFT_LIMIT_REV);
    leftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
    //  .pid(ARM_NORMAL_P_VAL, ARM_NORMAL_I_VAL, ARM_NORMAL_D_VAL)
      .pid(0.0225,0,0)
      .positionWrappingEnabled(false)
      .outputRange(-1,1);
    //leftConfig.closedLoop.maxMotion
      //.maxVelocity(7.5)
      //.maxAcceleration(15);
    // leftConfig.closedLoop.maxMotion
    //   .maxVelocity(4200)
    //   .maxAcceleration(12000)
    //   .allowedClosedLoopError(0.2);
    rightConfig
      .idleMode(IdleMode.kBrake)
      .follow(Constants.CAN.ELEVATOR_LEFT,true);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    elevatorClosedLoopController = leftMotor.getClosedLoopController();

    relativeEncoder = leftMotor.getEncoder();
    setPosition = getArmPosition();

  }

  public void setArticulator(AlgaeArticulator artic) {
    this.articulator = artic;
  }
  
  public void resetEncoders() {
    elevatorClosedLoopController.setReference(0, SparkMax.ControlType.kPosition);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    leftMotor.set((speedSupplier.get() * 0.5));
  }

  public void setPosition(ElevatorPosition pos) {
    setPosition(pos.degreePos);
  }

  public void changePos() {
    setPosition(SmartDashboard.getNumber("Arm/Position", 110.0));
  }

  public void setPosition(double pos) {
    setPosition = pos;
    elevatorClosedLoopController.setReference(pos, SparkMax.ControlType.kPosition);
  }

  public boolean isAtPosition(double pos) {
    return Math.abs(getArmPosition() - pos) < THRESHOLD_DEGREES;
  }

  public boolean isAtPosition(ElevatorPosition pos) {
    return isAtPosition(pos.degreePos);
  }

  public boolean isInPosition() {
    return isAtPosition(setPosition);
  }

  public double getArmPosition() {
    return relativeEncoder.getPosition();
  }

  public boolean isAboveSafePosition() {
    return getArmPosition() > ElevatorPosition.SAFE_POS.degreePos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Position", getArmPosition());
    SmartDashboard.putBoolean("Elevator/isInPosition", isInPosition());
  }
  

  public enum ElevatorPosition {
    // AMP(150.0),
    // HOME(ARM_SOFT_LIMIT_BKW),
    // SHUTTLE(78.0),
    // PICKUP_FLOOR(ARM_SOFT_LIMIT_BKW);
    LEVEL4(150.0,"level4"),
    LEVEL3(100.0, "level3"),
    LEVEL2(50.0, "level2"),
    LEVEL1(0.0, "level1"),
    HOME(0.0, "home"),
    L2_ALGAE(0.0, "level2Algae"),
    L3_ALGAE(0.0, "level3Algae"),
    CORAL_STATION(0.0, "coralStation"),
    SAFE_POS(50.0, "safePos");
  
    //this.setDefaultCommand(new RunCommand(() -> elevator.setPosition(LEVEL1), elevator));
    
    public final double degreePos;
    public final String lev;
      ElevatorPosition(double degreePos, String lev) {
        this.degreePos = degreePos;
        this.lev = lev;
      }
  }

}
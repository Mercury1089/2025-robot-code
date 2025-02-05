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

public class Elevator extends SubsystemBase {

  /** Creates a new Arm. */
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

  public Elevator() {
    leftMotor = new SparkFlex(CAN.ELEVATOR_LEFT, MotorType.kBrushless);
    rightMotor = new SparkFlex(CAN.ELEVATOR_RIGHT, MotorType.kBrushless);

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    SparkFlexConfig rightConfig = new SparkFlexConfig();

    leftConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false);
   // leftConfig.absoluteEncoder
    //  .positionConversionFactor(360.0); // multiplied by native units
    // leftConfig.softLimit
    //   .forwardSoftLimitEnabled(true)
    //   .forwardSoftLimit(ARM_SOFT_LIMIT_FWD)
    //   .reverseSoftLimitEnabled(true)
    //   .reverseSoftLimit(ARM_SOFT_LIMIT_REV);
    leftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //  .pid(ARM_NORMAL_P_VAL, ARM_NORMAL_I_VAL, ARM_NORMAL_D_VAL)
      .pid(0.085,0,0)
      .positionWrappingEnabled(false)
      .outputRange(-1,1);
    //leftConfig.closedLoop.maxMotion
      //.maxVelocity(7.5)
      //.maxAcceleration(15);
    leftConfig.closedLoop.maxMotion
      .maxVelocity(4200)
      .maxAcceleration(12000)
      .allowedClosedLoopError(0.2);
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
  
  public void resetEncoders() {
    elevatorClosedLoopController.setReference(0, SparkMax.ControlType.kMAXMotionPositionControl);
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
    elevatorClosedLoopController.setReference(pos, SparkMax.ControlType.kMAXMotionPositionControl);
  }

  public double getPosToTarget(double distance) {
    // return (54.1 + (0.397 * distance) - 0.00132 * (distance * distance)) + ANGLE_OFFSET;
    return (37.1 + (0.633 * distance) - (0.00207 * (distance * distance))) + ANGLE_OFFSET;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm/Position", getArmPosition());

  }
  

  public enum ElevatorPosition {
    // AMP(150.0),
    // HOME(ARM_SOFT_LIMIT_BKW),
    // SHUTTLE(78.0),
    // PICKUP_FLOOR(ARM_SOFT_LIMIT_BKW);
    LEVEL4(150.0),
    LEVEL3(100.0),
    LEVEL2(50.0),
    LEVEL1(0.0),
    HOME(0.0),
    CORAL_STATION(0.0);
  
    //this.setDefaultCommand(new RunCommand(() -> elevator.setPosition(LEVEL1), elevator));
    
    public final double degreePos;
      ElevatorPosition(double degreePos) {
        this.degreePos = degreePos;
      }
  }

}
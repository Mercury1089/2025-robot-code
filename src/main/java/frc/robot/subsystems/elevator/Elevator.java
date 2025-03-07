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
import com.revrobotics.spark.SparkLimitSwitch;
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

  public Elevator() {
    leftMotor = new SparkFlex(CAN.ELEVATOR_LEFT, MotorType.kBrushless);
    rightMotor = new SparkFlex(CAN.ELEVATOR_RIGHT, MotorType.kBrushless);

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    SparkFlexConfig rightConfig = new SparkFlexConfig();

    leftConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .closedLoopRampRate(0.4);
    leftConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(21.0);
    //   .reverseSoftLimitEnabled(true)
    //   .reverseSoftLimit(ARM_SOFT_LIMIT_REV);
    leftConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(0.225,0,0)
      .positionWrappingEnabled(false)
      .outputRange(-1,1);

    rightConfig
      .idleMode(IdleMode.kBrake)
      .follow(Constants.CAN.ELEVATOR_LEFT,true);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    elevatorClosedLoopController = leftMotor.getClosedLoopController();

    relativeEncoder = leftMotor.getExternalEncoder();
    setPosition = getArmPosition();

  }
  
  public void resetEncoders() {
    // elevatorClosedLoopController.setReference(0, SparkMax.ControlType.kPosition); this DRIVES to zero
    relativeEncoder.setPosition(0.0);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    leftMotor.set((speedSupplier.get()));
  }

  public void setPosition(Supplier<ElevatorPosition> pos) {
    setPosition(pos.get().degreePos);
  }

  public void changePos() {
    setPosition(SmartDashboard.getNumber("Elevator/Position", 110.0));
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
  
  public boolean isSwitchEngaged() {
    return leftMotor.getReverseLimitSwitch().isPressed();
  }

  @Override
  public void periodic() {

    // if (leftMotor.getReverseLimitSwitch().isPressed() && getArmPosition() != 0.0) {
    //   resetEncoders();
    // }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator/Position", getArmPosition());
    SmartDashboard.putBoolean("Elevator/isInPosition", isInPosition());
    SmartDashboard.putBoolean("Elevator/limitSwitchEngaged", leftMotor.getReverseLimitSwitch().isPressed());
    SmartDashboard.putBoolean("Elevator/isHome", isAtPosition(ElevatorPosition.HOME));
  }
  

  public enum ElevatorPosition {
    LEVEL4(20.6,"level4"),
    LEVEL3(12.9, "level3"),
    LEVEL2(7.14, "level2"),
    LEVEL1(5.07, "level1"), // check this
    HOME(-0.15, "home"),
    L2_ALGAE(5.9, "level2Algae"), // check
    L3_ALGAE(10.75, "level3Algae"), // check
    CORAL_STATION(0.0, "coralStation"),
    SAFE_POS(7.14, "safePos"),
    PROCESSOR(0.0, "processor");
    
    public final double degreePos;
    public final String lev;
      ElevatorPosition(double degreePos, String lev) {
        this.degreePos = degreePos;
        this.lev = lev;
      }
  }

}

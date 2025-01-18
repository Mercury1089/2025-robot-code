// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;


public class Elevator extends SubsystemBase {
  /** Creates a new Arm. */
  public static final int
    ARM_PID_SLOT = 0;

  private static final double 
    ARM_NORMAL_P_VAL = 1.0 / 10.0,
    ARM_NORMAL_I_VAL = 0.0,
    ARM_NORMAL_D_VAL = 0.0;

  private static final float ARM_SOFT_LIMIT_FWD = (float) 147;

  private static final float ARM_SOFT_LIMIT_BKW = (float) 45.3;

  private static final double ANGLE_OFFSET = -3.5;

  private final double 
    NOMINAL_OUTPUT_FORWARD = 0.01, //0.02,
    PEAK_OUTPUT_FORWARD = 1.0, // 0.6,
    NOMINAL_OUTPUT_REVERSE = -0.01, //-0.5,
    PEAK_OUTPUT_REVERSE = -0.6;

  public final double GEAR_RATIO = 125.0 / 1.0;
  public final double THRESHOLD_DEGREES = 0.5;

  
  private SparkMax armLeft;
  private SparkMax armRight;
  private SparkClosedLoopController armPIDController;
  private AbsoluteEncoder armAbsoluteEncoder;
  private double setPosition;

  public Elevator() {
    armLeft = new SparkMax(CAN.ARM_LEFT, MotorType.kBrushless);
    armRight = new SparkMax(CAN.ARM_RIGHT, MotorType.kBrushless);

    // TODO: Configure...

    //TODO: How do we follow with the new REVLib??
    //armRight.follow(armLeft, true);

    armPIDController = armLeft.getClosedLoopController();

    setPosition = getArmPosition();

    SmartDashboard.putNumber("Arm/Position", getArmPosition());
  }
  
  public void resetEncoders() {
    armPIDController.setReference(0, SparkMax.ControlType.kPosition);
  }

  public void setSpeed(Supplier<Double> speedSupplier) {
    armLeft.set((speedSupplier.get() * 0.5));
  }

  public void setPosition(ArmPosition pos) {
    setPosition(pos.degreePos);
  }

  public void changePos() {
    setPosition(SmartDashboard.getNumber("Arm/Position", 110.0));
  }

  public void setPosition(double pos) {
    // if (pos > ARM_SOFT_LIMIT_FWD) {
    //   pos = ARM_SOFT_LIMIT_FWD;
    // } else if (pos < ARM_SOFT_LIMIT_BKW) {
    //   pos = ARM_SOFT_LIMIT_BKW;
    // }

    setPosition = pos;

    armPIDController.setReference(pos, SparkMax.ControlType.kPosition);
  }

  public double getPosToTarget(double distance) {
    // return (54.1 + (0.397 * distance) - 0.00132 * (distance * distance)) + ANGLE_OFFSET;
    return (37.1 + (0.633 * distance) - (0.00207 * (distance * distance))) + ANGLE_OFFSET;
  }

  

  public boolean isAtPosition(double pos) {
    return Math.abs(getArmPosition() - pos) < THRESHOLD_DEGREES;
  }

  public boolean isAtPosition(ArmPosition pos) {
    return isAtPosition(pos.degreePos);
  }

  public double getArmPosition() {
    return armAbsoluteEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  public enum ArmPosition {
    // AMP(150.0),
    // HOME(ARM_SOFT_LIMIT_BKW),
    // SHUTTLE(78.0),
    // PICKUP_FLOOR(ARM_SOFT_LIMIT_BKW);
    LEVEL4(150.0),
    LEVEL3(120.0),
    LEVEL2(90.0),
    LEVEL1(60.0);
  
    
    public final double degreePos;
      ArmPosition(double degreePos) {
        this.degreePos = degreePos;
      }
  }

}
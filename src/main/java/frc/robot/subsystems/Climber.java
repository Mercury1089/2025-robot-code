package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class Climber extends SubsystemBase{

    private Servo servo;
    private SparkFlex climber;
    private double lockAngle = 80.0, unlockAngle = 46.0;

    private double angle = 0.0;
    private RelativeEncoder encoder;
    
    public Climber() {
        servo = new Servo(1);
        climber = new SparkFlex(CAN.CLIMBER, MotorType.kBrushless);

        SparkFlexConfig climberConfig = new SparkFlexConfig();
        
        climberConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        climberConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(43.0);

        climber.configure(climberConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        encoder = climber.getEncoder();

        
    }

    public void lockRatchet() {
        servo.setAngle(lockAngle);
    }

    public void unlockRatchet() {
        servo.setAngle(unlockAngle);
    }

    public void stopClimber() {
        climber.set(0.0);
        lockRatchet();
    }

    public void climberOut() {
        unlockRatchet();
        climber.set(0.50);
    }

    public void climberIn() {
        lockRatchet();
        climber.set(-0.50);
    }

    public void changePos() {
        servo.setAngle(angle);
    }

    public void resetEncoder() {
        encoder.setPosition(0.0);
    }

    @Override
    public void periodic() {
        angle = SmartDashboard.getNumber("Climber/servoPos", 0.0);
        SmartDashboard.putNumber("Climber/servoPos", angle);
        SmartDashboard.putNumber("Climber/climberAngle", encoder.getPosition());
    }
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;

public class Climber extends SubsystemBase{

    private Servo servo;
    private SparkMax climber;
    private double lockAngle = 80.0, unlockAngle = 46.0;

    private double angle = 0.0;
    
    public Climber() {
        servo = new Servo(1);
        climber = new SparkMax(CAN.CLIMBER, MotorType.kBrushless);

        SparkMaxConfig climberConfig = new SparkMaxConfig();
        
        climberConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        climber.configure(climberConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        
    }

    public void lockRatchet() {
        servo.setAngle(lockAngle);
    }

    public void unlockRatchet() {
        servo.setAngle(unlockAngle);
    }

    public void setSpeed(Supplier<Double> speed) {
        climber.set(speed.get());
    }

    public void climberOut() {
        if (servo.getAngle() == lockAngle) {
            setSpeed(() -> 1.0);
        } else {
            lockRatchet();
            climberOut();
        }
    }

    public void climberIn() {
        if (servo.getAngle() == unlockAngle) {
            setSpeed(() -> -1.0);
        } else {
            unlockRatchet();
            climberIn();
        }
    }

    public void changePos() {
        servo.setAngle(angle);
    }

    @Override
    public void periodic() {
        angle = SmartDashboard.getNumber("Climber/servoPos", 0.0);
        SmartDashboard.putNumber("Climber/servoPos", angle);
    }
}

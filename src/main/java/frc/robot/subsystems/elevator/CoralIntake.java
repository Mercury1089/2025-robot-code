package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BREAKBEAM;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.sensors.ProximitySensor;

public class CoralIntake extends SubsystemBase {

    private SparkMax coralIntake;
    private ProximitySensor coralSensor;
    private final double coralTriggerValue = 0.1; // TODO: need to test for this

    /** Creates a new intake. */
    public CoralIntake() {
        coralIntake = new SparkMax(CAN.INTAKE, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake)
        .inverted(false);

        coralIntake.configure(intakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


        coralSensor = new ProximitySensor(CAN.CORAL_SENSOR, coralTriggerValue);

    }

    public void setSpeed(IntakeSpeed intakeSpeed) {
        coralIntake.set(intakeSpeed.speed);
    }

    public boolean hasCoral() {
        return coralSensor.isTriggered();
    }

    public enum IntakeSpeed {
        INTAKE(1.0),
        OUTTAKE(-1.0),
        SHOOT(1.0),
        AMP(0.75),
        STOP(0.0);

        public final double speed;

        IntakeSpeed(double speed) {
            this.speed = speed;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake/hasCoral", hasCoral());
    }
}

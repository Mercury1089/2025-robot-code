package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BREAKBEAM;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.sensors.ProximitySensor;

public class CoralIntake extends SubsystemBase {

    private SparkMax coralIntake;
    private ProximitySensor frontCoralSensor, backCoralSensor;
    private boolean ejecting = false;
    private final double coralTriggerValue = 0.1; // TODO: need to test for this

    /** Creates a new intake. */
    public CoralIntake() {
        coralIntake = new SparkMax(CAN.CORAL_INTAKE, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        coralIntake.configure(intakeConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        frontCoralSensor = new ProximitySensor(CAN.FRONT_CORAL_SENSOR, coralTriggerValue);
        backCoralSensor = new ProximitySensor(CAN.BACK_CORAL_SENSOR, coralTriggerValue);

    }

    public void setSpeed(IntakeSpeed intakeSpeed) {
        coralIntake.set(intakeSpeed.speed);
    }

    public boolean hasCoralEntered() {
        return backCoralSensor.isTriggered();
    }

    public boolean isCoralEntering() {
        return backCoralSensor.isTriggered() && !frontCoralSensor.isTriggered();
    }
    public boolean isCoralExiting() {
        return !backCoralSensor.isTriggered() && frontCoralSensor.isTriggered();
    }
    public boolean hasCoral() {
        return backCoralSensor.isTriggered() && frontCoralSensor.isTriggered();
    }
    public boolean noCoralPresent() {
        return !backCoralSensor.isTriggered() && !frontCoralSensor.isTriggered();
    }

    public void setEjecting(boolean eject) {
        ejecting = eject;
    }

    public boolean isEjecting() {
        return ejecting;
    }
 
    public void intakeCoral() {
        setSpeed(IntakeSpeed.SLOW_INTAKE);
    }
    
    public void stopIntake() {
        setSpeed(IntakeSpeed.STOP);
    }

    public enum IntakeSpeed {
        INTAKE(0.3),
        SLOW_INTAKE(0.3),
        BRING_BACK(-0.1),
        OUTTAKE(0.85),
        OUTTAKEL4(1.0),
        STOP(0.0),
        L1_OUTTAKE(-1.0);

        public final double speed;

        IntakeSpeed(double speed) {
            this.speed = speed;
        }
    }

    @Override
    public void periodic() {
        if (noCoralPresent()) {
            setEjecting(false);
        }
        SmartDashboard.putBoolean("Intake/hasCoralEntered", hasCoralEntered());
        SmartDashboard.putBoolean("Intake/hasCoral", hasCoral());
        SmartDashboard.putBoolean("Intake/ejecting", ejecting);
        SmartDashboard.putBoolean("Intake/shouldRun",(!hasCoral() && hasCoralEntered()) || (hasCoral() && !hasCoralEntered()));
    }
}

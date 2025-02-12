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
    private final double coralTriggerValue = 0.2; // TODO: need to test for this

    /** Creates a new intake. */
    public CoralIntake() {
        coralIntake = new SparkMax(CAN.CORAL_INTAKE, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake)
        .inverted(false);

        coralIntake.configure(intakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        frontCoralSensor = new ProximitySensor(CAN.FRONT_CORAL_SENSOR, coralTriggerValue);
        backCoralSensor = new ProximitySensor(CAN.BACK_CORAL_SENSOR, coralTriggerValue);

    }

    public void setSpeed(IntakeSpeed intakeSpeed) {
        coralIntake.set(intakeSpeed.speed * 0.5);
    }

    public boolean hasCoralEntered() {
        return backCoralSensor.isTriggered();
    }

    public boolean hasCoral() {
        return frontCoralSensor.isTriggered();
    }

    public boolean noCoralPresent() {
        return !backCoralSensor.isTriggered() && !frontCoralSensor.isTriggered();
    }

    public void spitCoral() {
        ejecting = true;
        intakeCoral();
    }

    public void setEjecting(boolean eject) {
        ejecting = eject;
    }

    public boolean getEjecting() {
        return ejecting;
    }
 
    public void intakeCoral() {
        setSpeed(IntakeSpeed.INTAKE);
    }
    
    public void stopIntake() {
        setSpeed(IntakeSpeed.STOP);
    }

    public enum IntakeSpeed {
        INTAKE(0.8),
        SLOW_INTAKE(0.25),
        BRING_BACK(-0.25),
        OUTTAKE(0.8),
        STOP(0.0);

        public final double speed;

        IntakeSpeed(double speed) {
            this.speed = speed;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake/hasCoralEntered", hasCoralEntered());
        SmartDashboard.putBoolean("Intake/hasCoral", hasCoral());
        SmartDashboard.putBoolean("Intake/shouldRun",(!hasCoral() && hasCoralEntered()) || (hasCoral() && !hasCoralEntered()));
    }
}

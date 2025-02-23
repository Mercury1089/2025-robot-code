package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.sensors.ProximitySensor;

public class AlgaeIntake extends SubsystemBase {
    private SparkFlex algaeIntake;
    private final int ALGAE_INTAKE_CURRENT_LIMIT = 18;
    private SparkClosedLoopController controller;

    private int loops = 0;
    private boolean holdingAlgae = false;

    public AlgaeIntake() {
        algaeIntake = new SparkFlex(CAN.ALGAE_INTAKE, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(ALGAE_INTAKE_CURRENT_LIMIT);

        intakeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kNoSensor)
        .pid(0.5,0.0,0.0);

        algaeIntake.configure(intakeConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        controller = algaeIntake.getClosedLoopController();
        
    }

    public void intakeAlgae() {
        controller.setReference((double) ALGAE_INTAKE_CURRENT_LIMIT, ControlType.kCurrent);
    }

    public void setSpeed(double speed) {
        algaeIntake.set(speed);
    }

    public void setSpeed(AlgaeSpeed speed) {
        algaeIntake.set(speed.speed);
    }

    public void powerCheck() {
        if ((algaeIntake.getOutputCurrent() > 17.5 && algaeIntake.getAppliedOutput() < 0.05) || (holdingAlgae && algaeIntake.getAppliedOutput() == 0.0)) { 
            loops++;
            if (loops > 10) {
                holdingAlgae = true;
            }
        } else {
            loops = 0;
            holdingAlgae = false;
        }
    }

    public boolean hasAlgae() {
        return holdingAlgae;
    }

    public enum AlgaeSpeed {
        INTAKE(1.0),
        OUTTAKE(-1.0),
        STOP(0.0);

        public final double speed;

        AlgaeSpeed(double speed) {
            this.speed = speed;
        }
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      powerCheck();
      SmartDashboard.putNumber("AlgaeIntake/Algae Intake Current", algaeIntake.getOutputCurrent());
      SmartDashboard.putNumber("AlgaeIntake/Algae Intake Output", algaeIntake.getAppliedOutput());
      SmartDashboard.putBoolean("AlgaeIntake/hasAlgae", hasAlgae());
    }
}

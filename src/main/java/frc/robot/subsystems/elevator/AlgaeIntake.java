package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
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
    private SparkMax algaeIntake;
    private final int ALGAE_INTAKE_CURRENT_LIMIT = 10;
    private SparkClosedLoopController controller;
    private int loops = 0;

    public AlgaeIntake() {
        algaeIntake = new SparkMax(CAN.ALGAE_INTAKE, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake)
        .inverted(true)
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

    // public boolean hasAlgae() {
    //     // if (algaeIntake.getOutputCurrent() > ALGAE_INTAKE_CURRENT_LIMIT * 0.95) { 
    //     //     loops++;
    //     //     if (loops > 10) {
    //     //         return true;
    //     //     }
    //     // } else {
    //     //     loops = 0;
    //     // }

    //     // return false;
    // }

    public enum AlgaeSpeed {
        INTAKE(0.5),
        OUTTAKE(-0.5),
        STOP(0.0);

        public final double speed;

        AlgaeSpeed(double speed) {
            this.speed = speed;
        }
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("AlgaeIntake/Algae Intake Current", algaeIntake.getOutputCurrent());
      
      SmartDashboard.putNumber("AlgaeIntake/Algae Intake Output", algaeIntake.getAppliedOutput());
    }
}

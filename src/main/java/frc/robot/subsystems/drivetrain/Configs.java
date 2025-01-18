package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public final class Configs {
    
     public static final class Elevator {
        public static final SparkMaxConfig leftConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightConfig = new SparkMaxConfig();

        static {

                double turningFactor = 360; //this used to be 2pi

                double pVal = 1.0 / 10.0;
                double iVal = 0.0;
                double dVal = 0.0;

                leftConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
                leftConfig.absoluteEncoder
                    .inverted(false)
                    .positionConversionFactor(turningFactor) // not radians
                    .velocityConversionFactor(turningFactor / 60.0); // degrees per second probably lol
                leftConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(pVal, iVal, dVal)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(false) // sounds bad, used to be true
                    //.positionWrappingInputRange(0, turningFactor); we shouldn't be wrapping
                    ;
                rightConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .follow(Constants.CAN.ARM_LEFT,true); //works probably
                rightConfig.absoluteEncoder
                    
                    .positionConversionFactor(turningFactor) // not radians
                    .velocityConversionFactor(turningFactor / 60.0); // degrees per second probably lol
                rightConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(pVal, iVal, dVal)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(false) // sounds bad, used to be true
                    //.positionWrappingInputRange(0, turningFactor); we shouldn't be wrapping
                    ;
        }
        
    }
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = Constants.SWERVE.WHEEL_DIAMETER * Math.PI
                    / Constants.SWERVE.MOTOR_REDUCTION;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / Constants.SWERVE.DRIVE_WHEEL_FREE_SPEED;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}
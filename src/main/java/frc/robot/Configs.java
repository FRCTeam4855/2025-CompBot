package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public final class Configs {

    public static final class MAXSwerveModule{

        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turnConfig = new SparkMaxConfig();

        static {
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            //DRIVE CONFIG

            driveConfig
                .idleMode(ModuleConstants.kDrivingMotorIdleMode)
                .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);

            driveConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60);

            driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

             //TURN CONFIG

            turnConfig
                .idleMode(ModuleConstants.kDrivingMotorIdleMode)
                .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

            turnConfig.absoluteEncoder
                .inverted(false)
                .positionConversionFactor(turningFactor)
                .velocityConversionFactor(turningFactor / 60);

            turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
                .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class ElevatorSubsystem {

        public static final SparkFlexConfig rightElevatorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig leftElevatorConfig = new SparkFlexConfig();

        static {
                //RIGHT ELEVATOR CONFIG

                rightElevatorConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(30);
                rightElevatorConfig.encoder
                    //.inverted(false)
                    .positionConversionFactor(10)
                    .velocityConversionFactor(10);
                rightElevatorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pidf(.5, 0, 0, 0)
                    .outputRange(-.05, .05)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 1000);

                //LEFT ELEVATOR CONFIG

                leftElevatorConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(30)
                    .follow(11, true);
                leftElevatorConfig.encoder
                    //.inverted(true)
                    .positionConversionFactor(10)
                    .velocityConversionFactor(10);
                leftElevatorConfig.closedLoop
                    .pidf(.5, 0, 0, 0)
                    .outputRange(-.05, .05)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 1000);
        }
    }

    public static final class ManipulatorSubsystem {

        public static final SparkMaxConfig rightManipulatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftManipulatorConfig = new SparkMaxConfig();

        static {
            //RIGHT MANIPULATOR CONFIG

            rightManipulatorConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(20);
            rightManipulatorConfig.encoder
                // .inverted(false)
                .positionConversionFactor(10)
                .velocityConversionFactor(10);
            rightManipulatorConfig.closedLoop
                .pidf(1, 0, 0, 0)
                .outputRange(-1, 1);

            //LEFT MANIPULATOR CONFIG

            leftManipulatorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .follow(9, true);
            leftManipulatorConfig.encoder
                .positionConversionFactor(10)
                .velocityConversionFactor(10);
            leftManipulatorConfig.closedLoop
                .pidf(1, 0, 0, 0)
                .outputRange(-1, 1);
        }
    }
}
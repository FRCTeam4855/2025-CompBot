package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ModuleConstants;

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
                .inverted(true)
                .idleMode(ModuleConstants.kDrivingMotorIdleMode)
                .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

            turnConfig.absoluteEncoder
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
                    .idleMode(IdleMode.kCoast)
                    .inverted(false)
                    .closedLoopRampRate(.05)
                    .smartCurrentLimit(60);
                rightElevatorConfig.encoder
                    .positionConversionFactor(1);
                rightElevatorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pidf(0.25, 0, 0, 0)
                    .outputRange(-.4, 1)
                    .positionWrappingEnabled(false);

                //LEFT ELEVATOR CONFIG

                leftElevatorConfig
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(60)
                    .closedLoopRampRate(.05)
                    .follow(9, true);
                leftElevatorConfig.encoder
                    .positionConversionFactor(1);
                leftElevatorConfig.externalEncoder
                    .positionConversionFactor(12);
                leftElevatorConfig.closedLoop
                    .pidf(0.25, 0, 0, 0)
                    .outputRange(-.4, 1)
                    .positionWrappingEnabled(false);
        }
    }

    public static final class ManipulatorSubsystem {

        public static final SparkMaxConfig rightManipulatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftManipulatorConfig = new SparkMaxConfig();

        static {
            //RIGHT MANIPULATOR CONFIG

            rightManipulatorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .inverted(true);
            // rightManipulatorConfig.encoder
            //     .inverted(false)
            //     .positionConversionFactor(10)
            //     .velocityConversionFactor(10);
            rightManipulatorConfig.closedLoop
                .pidf(1, 0, 0, 0)
                .outputRange(-1, 1);

            //LEFT MANIPULATOR CONFIG

            leftManipulatorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .follow(11, true);
            // leftManipulatorConfig.encoder
            //     .positionConversionFactor(10)
            //     .velocityConversionFactor(10);
            leftManipulatorConfig.closedLoop
                .pidf(1, 0, 0, 0)
                .outputRange(-1, 1);
        }
    }

    public static final class AlgaeSubsystem {

        public static final SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
        public static final SparkMaxConfig algaeArmConfig = new SparkMaxConfig();

        static {
            algaeIntakeConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(10);
            algaeIntakeConfig.closedLoop
                .pidf(1, 0, 0, 0)
                .outputRange(-1, 1);

            algaeArmConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(20);
            algaeArmConfig.encoder
                .positionConversionFactor(360)
                .velocityConversionFactor(360);
            algaeArmConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(1, 0, 0, 0)
                .positionWrappingEnabled(false)
                .outputRange(-1, 1);
        }
    }

    public static final class ClimberSubsystem {

        public static final SparkMaxConfig climberWinchConfig = new SparkMaxConfig();
        public static final SparkMaxConfig climberRotateConfig = new SparkMaxConfig();

        static {
                //CLIMBER WINCH CONFIG
                climberWinchConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .closedLoopRampRate(.05)
                    .smartCurrentLimit(10);
                climberWinchConfig.encoder
                    .positionConversionFactor(1);
                climberWinchConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pidf(1, 0, 0, 0)
                    .outputRange(-0.25, 0.25);

                //CLIMBER INTAKE ROTATION MOTOR CONFIG
                climberRotateConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .closedLoopRampRate(.05)
                    .smartCurrentLimit(5);
                climberRotateConfig.encoder
                    .positionConversionFactor(1);
                climberRotateConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pidf(1, 0, 0, 0)
                    .outputRange(-0.5, 0.5);
        }
    }
}
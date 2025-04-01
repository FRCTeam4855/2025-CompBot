package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends Subsystem {

    private final ManipulatorSubsystem m_manipulator = ManipulatorSubsystem.getInstance();
    public final SparkFlex rightSpark;
    public final SparkFlex leftSpark;
    public final RelativeEncoder rightEncoder;
    public final RelativeEncoder leftEncoder, extEncoder;
    public final SparkClosedLoopController rightPIDController;
    public final SparkClosedLoopController leftPIDController;
    public boolean sensorOverride = false;
    public double elevatorAdjustment = 0;

    @Override
    public void robotInit() {
        DataLogManager.log("ElevatorSubsystem in robotInit");
    }

    @Override
    public void autonomousInit() {
        DataLogManager.log("ElevatorSubsystem in autonomousInit");
    }

    @Override
    public void teleopInit() {
        DataLogManager.log("ElevatorSubsystem in teleopInit");
        ElevatorToSetpoint(0);
    }

    private static ElevatorSubsystem mInstance;
    public static ElevatorSubsystem getInstance() {
      if (mInstance == null) {
        mInstance = new ElevatorSubsystem();
      }
      return mInstance;
    }
  

    public ElevatorSubsystem() {

        rightSpark = new SparkFlex(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
        leftSpark = new SparkFlex(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
        rightEncoder = rightSpark.getEncoder();
        leftEncoder = leftSpark.getEncoder();
        rightPIDController = rightSpark.getClosedLoopController();
        leftPIDController = leftSpark.getClosedLoopController();

        rightSpark.configure(Configs.ElevatorSubsystem.rightElevatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        leftSpark.configure(Configs.ElevatorSubsystem.leftElevatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        extEncoder = leftSpark.getExternalEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator Clear", m_manipulator.isElevatorClear());
        SmartDashboard.putNumber("Right Elevator Pos", rightEncoder.getPosition());
        SmartDashboard.putNumber("Left Elevator Pos", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Elevator Output I", rightSpark.getOutputCurrent());
        SmartDashboard.putNumber("Left Elevator Output I", leftSpark.getOutputCurrent());
        SmartDashboard.putNumber("Left Elevator Ext Pos", extEncoder.getPosition());
    }

    public void AdjustElevator(double offset) {
        elevatorAdjustment = elevatorAdjustment + offset;
    }

    public void ElevatorAdjustReset() {
        elevatorAdjustment = 0;
    }

    public void ElevatorToSetpoint(int goalSetpoint) {
        if (m_manipulator.isElevatorClear() || sensorOverride) {
        rightPIDController.setReference(ElevatorConstants.elevatorPos[goalSetpoint] + elevatorAdjustment, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kElevatorConstantsGravityFF, ArbFFUnits.kVoltage);
        } else {
            System.out.println("! ELEVATOR NOT CLEAR !");
        }
    }

    public void overrideSensor() {
        sensorOverride =! sensorOverride;
    }
}
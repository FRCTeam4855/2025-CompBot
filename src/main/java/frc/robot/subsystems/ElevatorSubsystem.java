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

    public final ManipulatorSubsystem m_manipulator;
    public final SparkFlex rightSpark;
    public final SparkFlex leftSpark;
    public final RelativeEncoder rightEncoder;
    public final RelativeEncoder leftEncoder;
    public final SparkClosedLoopController rightPIDController;
    public final SparkClosedLoopController leftPIDController;
    public boolean sensorOverride = false;

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

    public ElevatorSubsystem(ManipulatorSubsystem m_manipulator) {
        this.m_manipulator = m_manipulator;

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
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator Clear", m_manipulator.isElevatorClear());
        SmartDashboard.putNumber("Right Elevator Pos", rightEncoder.getPosition());
        SmartDashboard.putNumber("Left Elevator Pos", leftEncoder.getPosition());
    }

    public void ElevatorToSetpoint(int goalSetpoint) {
        if (m_manipulator.isElevatorClear() || sensorOverride) {
        rightPIDController.setReference(ElevatorConstants.elevatorPos[goalSetpoint], SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kElevatorConstantsGravityFF, ArbFFUnits.kVoltage);
        } else {
            System.out.println("! ELEVATOR NOT CLEAR !");
        }
    }

    public void overrideSensor() {
        sensorOverride =! sensorOverride;
    }
}
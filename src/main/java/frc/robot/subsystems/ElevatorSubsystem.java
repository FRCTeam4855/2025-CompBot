package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    public final ManipulatorSubsystem m_manipulator;
    public final SparkFlex rightSpark;
    public final SparkFlex leftSpark;
    public final SparkClosedLoopController rightPIDController;
    public final SparkClosedLoopController leftPIDController;
    public double elevatorPos;

    public ElevatorSubsystem(ManipulatorSubsystem m_manipulator) {
        this.m_manipulator = m_manipulator;

        rightSpark = new SparkFlex(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
        leftSpark = new SparkFlex(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
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

    }

    public void setElevatorPosition(int goalSetpoint) {
        elevatorPos = ElevatorConstants.elevatorPos[goalSetpoint];
    }

    public void raiseElevator() {
        rightPIDController.setReference(elevatorPos, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        //leftPIDController.setReference(elevatorPos, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
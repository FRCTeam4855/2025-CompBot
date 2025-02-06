package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    public final SparkAbsoluteEncoder rightAbsoluteEncoder;
    public final SparkAbsoluteEncoder leftAbsoluteEncoder;
    public int goalSetpoint;
    public double elevatorPos;

    public ElevatorSubsystem(ManipulatorSubsystem m_manipulator) {
        this.m_manipulator = m_manipulator;

        rightSpark = new SparkFlex(ElevatorConstants.kRightElevatorCanId, MotorType.kBrushless);
        leftSpark = new SparkFlex(ElevatorConstants.kLeftElevatorCanId, MotorType.kBrushless);
        rightPIDController = rightSpark.getClosedLoopController();
        leftPIDController = leftSpark.getClosedLoopController();
        rightAbsoluteEncoder = rightSpark.getAbsoluteEncoder();
        leftAbsoluteEncoder = leftSpark.getAbsoluteEncoder();

        rightSpark.configure(Configs.ElevatorSubsystem.rightElevatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        leftSpark.configure(Configs.ElevatorSubsystem.leftElevatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Elevator Clear", m_manipulator.isElevatorClear());

    }

    public double getRightPosition() {
        return(rightAbsoluteEncoder.getPosition());
    }
    public double getLeftPosition() {
        return(rightAbsoluteEncoder.getPosition());
    }

    public void setElevatorPosition(int goalSetpoint) {
        switch(goalSetpoint) { //TODO put the long code block in constants and make a single array for elevator position here
            case 1: 
                elevatorPos = ElevatorConstants.elevatorPos[0];
            case 2: 
                elevatorPos = ElevatorConstants.elevatorPos[1];
            case 3: 
                elevatorPos = ElevatorConstants.elevatorPos[2];
            case 4: 
                elevatorPos = ElevatorConstants.elevatorPos[3];
            case 5: 
                elevatorPos = ElevatorConstants.elevatorPos[4];
            case 6: 
                elevatorPos = ElevatorConstants.elevatorPos[5];
            case 7: 
                elevatorPos = ElevatorConstants.elevatorPos[6];
            case 8:
                elevatorPos = ElevatorConstants.elevatorPos[7];
            default:
                elevatorPos = ElevatorConstants.elevatorPos[0];
        }
    }

    public void raiseElevator() {
        rightPIDController.setReference(elevatorPos, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        leftPIDController.setReference(elevatorPos, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
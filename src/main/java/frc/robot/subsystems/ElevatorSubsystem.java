package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    public final SparkFlex rightSpark;
    public final SparkFlex leftSpark;
    public final SparkClosedLoopController rightPIDController;
    public final SparkClosedLoopController leftPIDController;
    public final SparkAbsoluteEncoder rightAbsoluteEncoder;
    public final SparkAbsoluteEncoder leftAbsoluteEncoder;
    public int goalSetpoint;
    public double elevatorPos;

    public ElevatorSubsystem(ElevatorSubsystem m_elevatorSubsystem) {
        rightSpark = new SparkFlex(9, MotorType.kBrushless);
        leftSpark = new SparkFlex(10, MotorType.kBrushless);
        rightPIDController = rightSpark.getClosedLoopController();
        leftPIDController = leftSpark.getClosedLoopController();
        rightAbsoluteEncoder = rightSpark.getAbsoluteEncoder();
        leftAbsoluteEncoder = leftSpark.getAbsoluteEncoder();

        rightSpark.configure(Configs.ElevatorSubsystem.rightElevatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        leftSpark.configure(Configs.ElevatorSubsystem.leftElevatorConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public double getRightPosition() {
        return(rightAbsoluteEncoder.getPosition());
    }
    public double getLeftPosition() {
        return(rightAbsoluteEncoder.getPosition());
    }

    public void setElevatorPosition(int goalSetpoint) {
        switch(goalSetpoint) {
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
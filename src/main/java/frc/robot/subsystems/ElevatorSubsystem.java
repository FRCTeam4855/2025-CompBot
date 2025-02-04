package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    public int elevatorSetpoint;
    public final SparkFlex rightSpark;
    public final SparkFlex leftSpark;
    public final SparkClosedLoopController rightPIDController;
    public final SparkClosedLoopController leftPIDController;
    public final SparkAbsoluteEncoder rightAbsoluteEncoder;
    public final SparkAbsoluteEncoder leftAbsoluteEncoder;

    public ElevatorSubsystem() {
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

    public double setElevatorPosition() {
        switch(elevatorSetpoint) {
            case 1: 
                return Constants.ElevatorConstants.kElevatorPos1;
            case 2: 
                return Constants.ElevatorConstants.kElevatorPos2;
            case 3: 
                return Constants.ElevatorConstants.kElevatorPos3;
            case 4: 
                return Constants.ElevatorConstants.kElevatorPos4;
            case 5: 
                return Constants.ElevatorConstants.kElevatorPos5;
            case 6: 
                return Constants.ElevatorConstants.kElevatorPos6;
            case 7: 
                return Constants.ElevatorConstants.kElevatorPos7;
            case 8:
                return Constants.ElevatorConstants.kElevatorPos8;
            default:
                return Constants.ElevatorConstants.kElevatorPos1;
        }
                
    }
}
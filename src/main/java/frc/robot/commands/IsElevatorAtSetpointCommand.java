package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class IsElevatorAtSetpointCommand extends Command {
    
    private ElevatorSubsystem m_elevatorSubsystem = ElevatorSubsystem.getInstance();
    double elevatorPosition;
    int goalSetpoint;
    double goalPosition;

    public IsElevatorAtSetpointCommand(int goalSetpoint) {
        this.goalSetpoint = goalSetpoint;
        //goalPosition = ElevatorConstants.elevatorPos[goalSetpoint];
    }

    @Override
    public void initialize() {
        goalPosition = ElevatorConstants.elevatorPos[goalSetpoint] + m_elevatorSubsystem.elevatorAdjustment;
    }

    @Override 
    public void execute() {
        elevatorPosition = m_elevatorSubsystem.rightEncoder.getPosition();
    }

    @Override
    public boolean isFinished() {
        if (goalPosition - 1 <= elevatorPosition && elevatorPosition <= goalPosition + 1) {
            return true;
        } else { 
            return false;
        }
    }
}
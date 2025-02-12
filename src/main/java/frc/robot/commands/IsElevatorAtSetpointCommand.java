package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class IsElevatorAtSetpointCommand extends Command {
    
    ElevatorSubsystem m_elevatorSubsystem;
    double elevatorPosition;
    int goalSetpoint;
    double goalPosition;

    public IsElevatorAtSetpointCommand(ElevatorSubsystem m_elevatorSubsystem, int goalSetpoint) {
        this.m_elevatorSubsystem = m_elevatorSubsystem;
        this.goalSetpoint = goalSetpoint;
        goalPosition = Constants.ElevatorConstants.elevatorPos[goalSetpoint];
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
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
public class RaiseElevatorCommand extends Command {
    
    private ElevatorSubsystem m_elevatorSubsystem;
    private int goalSetpoint;

    public RaiseElevatorCommand(ElevatorSubsystem m_elevatorSubsystem, int goalSetpoint) {
        this.m_elevatorSubsystem = m_elevatorSubsystem;
        this.goalSetpoint = goalSetpoint;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setElevatorPosition(goalSetpoint);
        m_elevatorSubsystem.raiseElevator();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
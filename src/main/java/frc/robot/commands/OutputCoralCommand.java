package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
public class OutputCoralCommand extends Command {
    
    ManipulatorSubsystem m_manipulator;
    private double speed;
    public OutputCoralCommand(ManipulatorSubsystem m_manipulator, double speed) {
        this.m_manipulator = m_manipulator;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        m_manipulator.RunManipulator(speed);
    }

    @Override
    public boolean isFinished() {
        if (!m_manipulator.outerSensor || m_manipulator.stopManipulator == true) {
            m_manipulator.StopManipulator();
            return true;
        } else {
            return false;
        }
    }
}

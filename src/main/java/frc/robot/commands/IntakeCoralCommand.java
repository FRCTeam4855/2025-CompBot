package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
public class IntakeCoralCommand extends Command {
    
    ManipulatorSubsystem m_manipulator;
    public IntakeCoralCommand(ManipulatorSubsystem m_manipulator) {
        this.m_manipulator = m_manipulator;
    }

    @Override
    public void initialize() {
        m_manipulator.RunManipulator(1);
    }

    @Override
    public boolean isFinished() {
        if (m_manipulator.outerSensor && !m_manipulator.innerSensor) {
            m_manipulator.StopManipulator();
            return true;
        } else {
            return false;
        }
    }
}
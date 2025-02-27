package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;
public class OutputCoralCommand extends Command {
    
    ManipulatorSubsystem m_manipulator;
    public OutputCoralCommand(ManipulatorSubsystem m_manipulator) {
        this.m_manipulator = m_manipulator;
    }

    @Override
    public void initialize() {
        m_manipulator.RunManipulator(ManipulatorConstants.kManipulatorSpeed);
    }

    @Override
    public boolean isFinished() {
        if (!m_manipulator.outerSensor) {
            m_manipulator.StopManipulator();
            return true;
        } else {
            return false;
        }
    }
}

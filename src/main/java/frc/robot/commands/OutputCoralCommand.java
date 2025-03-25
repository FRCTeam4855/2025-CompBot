package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightsConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
public class OutputCoralCommand extends Command {
    
    ManipulatorSubsystem m_manipulator;
    private double speed;
    private LightsSubsystem lights;
    public OutputCoralCommand(ManipulatorSubsystem m_manipulator, double speed, LightsSubsystem lights) {
        this.m_manipulator = m_manipulator;
        this.speed = speed;
        this.lights = lights;
    }

    @Override
    public void initialize() {
        m_manipulator.stopManipulator = false;
        m_manipulator.RunManipulator(speed);
    }

    @Override
    public boolean isFinished() {
        if (!m_manipulator.outerSensor || m_manipulator.stopManipulator == true) {
            m_manipulator.StopManipulator();
            lights.setLEDs(LightsConstants.C1_AND_C2_SINELON);
            return true;
        } else {
            return false;
        }
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IntakeCoralCommand extends Command {
    
    ManipulatorSubsystem m_manipulator;
    private double speed, currentSpeed;
    private LightsSubsystem lights;
    public IntakeCoralCommand(ManipulatorSubsystem m_manipulator, double speed, LightsSubsystem lights) {
        this.m_manipulator = m_manipulator;
        this.speed = speed;
        this.lights = lights;

        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        m_manipulator.stopManipulator = false;
        m_manipulator.RunManipulator(speed);
        currentSpeed = speed;
    }

    @Override 
    public void execute() {
        if (m_manipulator.outerSensor && (currentSpeed != ManipulatorConstants.kManipulatorSlowSpeed)) {
            m_manipulator.RunManipulator(ManipulatorConstants.kManipulatorSlowSpeed);
            currentSpeed = ManipulatorConstants.kManipulatorSlowSpeed;
            SmartDashboard.putNumber("Intake Speed", currentSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        if (m_manipulator.outerSensor && !m_manipulator.innerSensor) {
            lights.setLEDs(LightsConstants.GREEN);
            m_manipulator.StopManipulator();
            return true;
        } else {
            return false;
        }
    }
}
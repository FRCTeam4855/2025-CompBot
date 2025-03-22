package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IntakeCoralCommandClearJam extends Command {
    
    ManipulatorSubsystem m_manipulator;
    private double speed, currentSpeed;
    private Timer timer;
    public IntakeCoralCommandClearJam(ManipulatorSubsystem m_manipulator, double speed) {
        this.m_manipulator = m_manipulator;
        this.speed = speed;
        timer = new Timer();
        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        m_manipulator.stopManipulator = false;
        timer.reset();
        timer.start();
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
        if(timer.hasElapsed(3.5)) {
            m_manipulator.RunManipulator(speed);
            timer.reset();
        } else {
            if(timer.hasElapsed(3))
                m_manipulator.RunManipulator(-speed); 
        }
    }
 

    @Override
    public boolean isFinished() {
            if (m_manipulator.outerSensor && !m_manipulator.innerSensor){
            m_manipulator.StopManipulator();
            return true;
        } else {
            return false;
        }
    }
}
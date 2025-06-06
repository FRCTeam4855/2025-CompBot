package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IntakeCoralCommandClearJam extends Command {
    
    private ManipulatorSubsystem m_manipulator = ManipulatorSubsystem.getInstance();
    private double speed, currentSpeed;
    private Timer timer;
    private LightsSubsystem lights = LightsSubsystem.getInstance();
    public IntakeCoralCommandClearJam(double speed) {
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
        if(timer.hasElapsed(3.3)) {
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
            lights.setLEDs(LightsConstants.GREEN);
            return true;
        } else {
            return false;
        }
    }
}
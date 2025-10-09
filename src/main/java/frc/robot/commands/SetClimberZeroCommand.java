package frc.robot.commands;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberZeroCommand extends Command {

    private ClimberSubsystem m_climber = ClimberSubsystem.getInstance();

    public void initialize() {
        // Configs.ClimberSubsystem.climberWinchConfig.smartCurrentLimit(2);
        m_climber.m_winchSpark.configure(Configs.ClimberSubsystem.climberWinchConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_climber.ClimberWinchDriveDirect(-.25);
    }

    public void execute() {

    }

    public boolean isFinished() {
        //if (m_climber.m_winchSpark.getOutputCurrent() >= 4) {
          //  m_climber.ClimberWinchDriveDirect(0);
            //m_climber.m_winchEncoder.setPosition(0);
            //Configs.ClimberSubsystem.climberWinchConfig.smartCurrentLimit(40);
            //return true;
        //} else {
            return false;
        //}
    }
}

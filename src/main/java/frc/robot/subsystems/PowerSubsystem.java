package frc.robot.subsystems;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PowerSubsystemConstants;

public class PowerSubsystem extends Subsystem {
    public final PowerDistribution m_pdp = new PowerDistribution(PowerSubsystemConstants.kPDP_CAN_ID, PowerSubsystemConstants.kPDP_ModuleType);
    private int m_pdbNumChannels = m_pdp.getNumChannels();
    
    private static PowerSubsystem mInstance;
    public static PowerSubsystem getInstance() {
      if (mInstance == null) {
        mInstance = new PowerSubsystem();
      }
      return mInstance;
    }
    
    @Override
    public void robotInit() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void teleopInit() {
    }

    PowerSubsystem() {
    }

    @Override
    public void periodic() {
        /*SmartDashboard.putNumber("Battery Voltage", m_pdp.getVoltage());
        SmartDashboard.putNumber("Total Current", m_pdp.getTotalCurrent());
        for (int i = 0; i < m_pdbNumChannels; i++) {
            SmartDashboard.putNumber("Channel " + i + " Current", m_pdp.getCurrent(i));
        }*/
    }
}

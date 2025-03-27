package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends Subsystem {
    
    public final Spark m_Blinkin;  //4855
    
    @Override
    public void robotInit() {
        DataLogManager.log("LightsSubsystem in robotInit");
    }

    @Override
    public void autonomousInit() {
        DataLogManager.log("LightsSubsystem in autonomousInit");
        setLEDs(LightsConstants.C1_AND_C2_TWINKLES);
    }

    @Override
    public void teleopInit() {
        DataLogManager.log("LightsSubsystem in teleopInit");
        setLEDs(LightsConstants.C1_AND_C2_SINELON);
    }

    private static LightsSubsystem mInstance;
    public static LightsSubsystem getInstance() {
      if (mInstance == null) {
        mInstance = new LightsSubsystem();
      }
      return mInstance;
    }

    public LightsSubsystem() { 
        m_Blinkin = new Spark(1);
        setLEDs(LightsConstants.C1_AND_C2_SINELON);
    }

    public void setLEDs(double color) {
        m_Blinkin.set(color);
    }

}

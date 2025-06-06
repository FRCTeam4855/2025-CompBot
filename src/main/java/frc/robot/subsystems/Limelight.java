package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends Subsystem {

  public double[] tagPose;
  public double[] llPose;
  public boolean limelightHasTarget;

  @Override
  public void robotInit() {
    DataLogManager.log("LimelightSubsystem in robotInit");
    initialize();
  }

  @Override
  public void autonomousInit() {
    DataLogManager.log("LimelightSubsystem in autonomousInit");

    LimelightHelpers.SetIMUMode("limelight", 2); // Set IMU to 2D mode
  }

  @Override
  public void teleopInit() {
    DataLogManager.log("LimelightSubsystem in teleopInit");
    LimelightHelpers.SetIMUMode("limelight", 2); // Set IMU to 2D mode
  }

  private static Limelight mInstance;
  public static Limelight getInstance() {
    if (mInstance == null) {
      mInstance = new Limelight();
    }
    return mInstance;
  }

  public Limelight() {
    LimelightHelpers.SetIMUMode("limelight", 1);
    
  }
  
  public void setOffset(double xOffset) {
    LimelightHelpers.setFiducial3DOffset("limelight", 0, xOffset, 0);
  }

  public void setFilters(int[] validIDs) {
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
  }

  public void resetFilters() {
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", LimelightConstants.kAllIDs);
  }

  public void initialize() {
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", LimelightConstants.kAllIDs);
    LimelightHelpers.SetFidcuial3DOffset("limelight", 0, 0, 0);
  }
  
  @Override
  public void periodic() {
    tagPose = LimelightHelpers.getTargetPose_RobotSpace("limelight"); //tx = [0] ty = [1] tz = [2] roll = [3] pitch = [4] yaw = [5]
    llPose = LimelightHelpers.getBotPose_wpiBlue("limelight"); //tx = [0] ty = [1] tz = [2] roll = [3] pitch = [4] yaw = [5]
    limelightHasTarget = LimelightHelpers.getTV("limelight");
    
    if(limelightHasTarget) {
      SmartDashboard.putNumber("Limelight X", tagPose[0]);
      SmartDashboard.putNumber("Limelight Y", tagPose[1]);
      SmartDashboard.putNumber("Limelight Z", tagPose[2]);
      SmartDashboard.putNumber("Limelight Roll", tagPose[3]);
      SmartDashboard.putNumber("Limelight Pitch", tagPose[4]);
      SmartDashboard.putNumber("Limelight Yaw", tagPose[5]);
      SmartDashboard.putNumber("Limelight Area", LimelightHelpers.getTA("limelight"));
      SmartDashboard.putBoolean("Limelight Has Target", LimelightHelpers.getTV("limelight"));
      SmartDashboard.putString("Limelight in AprilTag Mode", LimelightHelpers.getCurrentPipelineType("limelight"));
    
      SmartDashboard.putNumber("p_tx", llPose[0]);
      SmartDashboard.putNumber("p_ty", llPose[1]);
      SmartDashboard.putNumber("p_yaw", llPose[5]);
    } else {
      SmartDashboard.putNumber("Limelight X", 0);
      SmartDashboard.putNumber("Limelight Y", 0);
      SmartDashboard.putNumber("Limelight Z", 0);
      SmartDashboard.putNumber("Limelight Roll", 0);
      SmartDashboard.putNumber("Limelight Pitch", 0);
      SmartDashboard.putNumber("Limelight Yaw", 0);
      SmartDashboard.putNumber("Limelight Area", 0);
      SmartDashboard.putBoolean("Limelight Has Target", false);
    
      SmartDashboard.putNumber("p_tx", 0);
      SmartDashboard.putNumber("p_ty", 0);
      SmartDashboard.putNumber("p_yaw", 0);
    }
  }  
}

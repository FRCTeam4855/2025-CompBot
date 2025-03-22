// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LightsConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  PowerDistribution m_pdp = new PowerDistribution(9, ModuleType.kRev);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    FollowPathCommand.warmupCommand().schedule();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    m_robotContainer.m_limelight.initialize();
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Field Oriented", m_robotContainer.fieldOriented);

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.m_robotDrive.updateOdometry();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    m_robotContainer.m_robotDrive.resetPose(new Pose2d(m_robotContainer.m_limelight.llPose[0], m_robotContainer.m_limelight.llPose[1], Rotation2d.fromDegrees(m_robotContainer.m_limelight.llPose[5])));

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();//m_autoSelectedString);

    LimelightHelpers.SetIMUMode("limelight", 2); // Set IMU to 2D mode

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.m_robotDrive.autoFlipped = true;

    m_robotContainer.m_lights.setLEDs(LightsConstants.C1_HEARTBEAT_SLOW);
  }

  /** This function is called periodically during autonomous. */
  @Override  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_algaeSubsystem.ArmToPosition(0);
    m_robotContainer.m_elevatorSubsystem.ElevatorToSetpoint(0);
    m_robotContainer.fieldOriented = true;
    m_robotContainer.m_lights.setLEDs(LightsConstants.C1_AND_C2_SINELON);
    if (m_robotContainer.m_limelight.llPose[0] != 0) {
      m_robotContainer.m_robotDrive.resetPose(new Pose2d(m_robotContainer.m_limelight.llPose[0], m_robotContainer.m_limelight.llPose[1], Rotation2d.fromDegrees(m_robotContainer.m_limelight.llPose[5])));
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

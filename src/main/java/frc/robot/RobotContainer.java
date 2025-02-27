// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveWithAprilTagCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.IsElevatorAtSetpointCommand;
import frc.robot.commands.OutputCoralCommand;
import frc.robot.commands.TimedLeftStrafeCommand;
import frc.robot.commands.TimedRightStrafeCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ManipulatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import frc.robot.subsystems.Limelight;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final LightsSubsystem m_lights = new LightsSubsystem();
    private final Limelight m_limelight = new Limelight();
    private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(m_manipulator);
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
   

    // The driver's controller
    Joystick m_leftDriverController = new Joystick(OIConstants.kLeftDriverControllerPort);
    Joystick m_rightDriverController = new Joystick(OIConstants.kRightDriverControllerPort);
    
    // The Operator Controller
    CommandXboxController m_operatorController1 = new CommandXboxController(OIConstants.kOperatorControllerPort1);
    GenericHID m_operatorBoard = new GenericHID(3);
    //CommandGenericHID m_buttonBoard = new CommandGenericHID();

    public static boolean fieldOriented = false;
    public double speedMultiplier = OIConstants.kSpeedMultiplierDefault;
    private final SendableChooser<Command> autoChooser;

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer() {

        //Register Named Commands
        NamedCommands.registerCommand("AlgaeFloorPickup", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0)),
            new InstantCommand(() -> m_algaeSubsystem.SetFloorPickup()),
            new InstantCommand(() -> m_algaeSubsystem.IntakeAlgae()),
            new InstantCommand(() -> m_algaeSubsystem.ArmToPosition(2))));

        NamedCommands.registerCommand("AlgaeReefPickupOne", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(3)),
            new IsElevatorAtSetpointCommand(m_elevatorSubsystem, 3),
            new InstantCommand(() -> m_algaeSubsystem.SetReefPickup()),
            new InstantCommand(() -> m_algaeSubsystem.IntakeAlgae()),
            new InstantCommand(() -> m_algaeSubsystem.ArmToPosition(1))));

        NamedCommands.registerCommand("AlgaeReefPickupTwo", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(5)),
            new IsElevatorAtSetpointCommand(m_elevatorSubsystem, 5),
            new InstantCommand(() -> m_algaeSubsystem.SetReefPickup()),
            new InstantCommand(() -> m_algaeSubsystem.IntakeAlgae()),
            new InstantCommand(() -> m_algaeSubsystem.ArmToPosition(2))));

        NamedCommands.registerCommand("AutoDeliverLevelOne", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(1)),
            new IsElevatorAtSetpointCommand(m_elevatorSubsystem, 1),
            new OutputCoralCommand(m_manipulator),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("AutoDeliverLevelTwo", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(2)),
            new IsElevatorAtSetpointCommand(m_elevatorSubsystem, 2),
            new OutputCoralCommand(m_manipulator),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("AutoDeliverLevelThree", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(3)),
            new IsElevatorAtSetpointCommand(m_elevatorSubsystem, 3),
            new OutputCoralCommand(m_manipulator),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("AutoDeliverLevelFour", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(4)),
            new IsElevatorAtSetpointCommand(m_elevatorSubsystem, 4),
            new OutputCoralCommand(m_manipulator),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        NamedCommands.registerCommand("IntakeCoral", new SequentialCommandGroup(
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0)),
            new IntakeCoralCommand(m_manipulator)));

        NamedCommands.registerCommand("DeliverCoral", new SequentialCommandGroup(
            new OutputCoralCommand(m_manipulator),
            new InstantCommand(() -> m_elevatorSubsystem.ElevatorToSetpoint(0))));

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
        // The left Joystick controls translation of the robot.
        // The right Joystick controls rotation of the robot.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_leftDriverController.getRawAxis(1) * speedMultiplier, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_leftDriverController.getRawAxis(0) * speedMultiplier, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_rightDriverController.getRawAxis(0) * speedMultiplier, OIConstants.kDriveDeadband) * OIConstants.kRotateScale,
                fieldOriented, true),
            m_robotDrive));

            autoChooser = AutoBuilder.buildAutoChooser(); 
            SmartDashboard.putData("Auto Chooser", autoChooser);             
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
    
////    Driver Controls

       new JoystickButton(m_leftDriverController,OIConstants.kJS_BB)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));

        new JoystickButton(m_leftDriverController,OIConstants.kJS_LB)
            .onTrue(new TimedLeftStrafeCommand(
                 m_robotDrive));

        new JoystickButton(m_leftDriverController,OIConstants.kJS_RB)
            .onTrue(new TimedRightStrafeCommand(
                 m_robotDrive));
       
        new JoystickButton(m_rightDriverController, OIConstants.kJS_RB).debounce(0.1)  //Gyro reset
            .whileTrue(new InstantCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive)); 

        new JoystickButton(m_rightDriverController, OIConstants.kJS_LB)  //Field oriented toggle
            .whileTrue(new InstantCommand(
                () -> toggleFieldOriented()));
        
        new JoystickButton(m_leftDriverController, OIConstants.kJS_Trigger)  //Precise Driving Mode set
            .whileTrue(new InstantCommand(
                () -> speedMultiplier=OIConstants.kSpeedMultiplierPrecise));

        new JoystickButton(m_leftDriverController, OIConstants.kJS_Trigger)  //Precise Driving Mode clear
            .whileFalse(new InstantCommand(
                () -> speedMultiplier=OIConstants.kSpeedMultiplierDefault));

        new JoystickButton(m_rightDriverController, OIConstants.kJS_Trigger)
            .whileTrue(new DriveWithAprilTagCommand(
            m_robotDrive, m_limelight, m_leftDriverController, m_rightDriverController));



        new JoystickButton(m_operatorBoard, 1)
            .onTrue(new InstantCommand(
                () -> m_algaeSubsystem.IntakeAlgae()));    

        new JoystickButton(m_operatorBoard, 2)
            .onTrue(new InstantCommand(
                () -> m_algaeSubsystem.DeliverAlgae()));

        new JoystickButton(m_operatorBoard, 3)
            .onTrue(new InstantCommand(
                () -> m_algaeSubsystem.StopIntake()));

        new JoystickButton(m_operatorBoard, 4)
            .onChange(new InstantCommand(
                () -> m_algaeSubsystem.TogglePickup()));

        new JoystickButton(m_operatorBoard, 6)
            .onTrue(NamedCommands.getCommand("AlgaeReefPickupOne"));

            new JoystickButton(m_operatorBoard, 7)
            .onTrue(NamedCommands.getCommand("AlgaeReefPickupTwo"));

        new JoystickButton(m_operatorBoard, 10)
             .onTrue(NamedCommands.getCommand("AlgaeFloorPickup"));
            
        new JoystickButton(m_operatorBoard, 14)
            .onTrue(new InstantCommand(
                () -> m_algaeSubsystem.ArmToPosition(0)));

        new JoystickButton(m_operatorBoard, 17)
            .onTrue(new OutputCoralCommand(m_manipulator));

        new JoystickButton(m_operatorBoard, 16)
            .onTrue(new IntakeCoralCommand(m_manipulator));

        new JoystickButton(m_operatorBoard, 5)
            .onTrue(new InstantCommand(
                () -> m_manipulator.RunManipulator(1)));

        new JoystickButton(m_operatorBoard, 11)
            .onTrue(new InstantCommand(
                () -> m_manipulator.StopManipulator()));

        new JoystickButton(m_operatorBoard, 12)
            .onTrue(new InstantCommand(
                () -> m_manipulator.RunManipulator(-1)));
        
        new JoystickButton(m_operatorBoard, 15)
        .onTrue(new InstantCommand(
            () -> m_elevatorSubsystem.overrideSensor())); 

        new JoystickButton(m_operatorBoard, 18)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(0)));

        new JoystickButton(m_operatorBoard, 19)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(1)));

        new JoystickButton(m_operatorBoard, 20)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(2)));

        new JoystickButton(m_operatorBoard, 21)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(3)));

        new JoystickButton(m_operatorBoard, 22)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(4)));

        new JoystickButton(m_operatorBoard, 13)
            .onTrue(new InstantCommand(
                () -> m_elevatorSubsystem.ElevatorToSetpoint(5)));

        // Operator Controls

        m_operatorController1.a()
            .whileTrue(new RunCommand(
                () -> m_lights.setLEDs(LightsConstants.GREEN),
                m_lights));
        m_operatorController1.x()
            .whileTrue(new RunCommand(
                () -> m_lights.setLEDs(LightsConstants.RED),
                m_lights));
        m_operatorController1.b()
            .whileTrue(new RunCommand(
                () -> m_lights.setLEDs(LightsConstants.VIOLET),
                m_lights));
        m_operatorController1.y()
            .whileTrue(new RunCommand(
                () -> m_lights.setLEDs(LightsConstants.GOLD),
                m_lights));
          }

    private void toggleFieldOriented () {
        fieldOriented = !fieldOriented;
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Phoniex https://api.ctr-electronics.com/phoenix6/release/java/

package frc.robot;


// import frc.robot.subsystems.PneumaticSubsystem;
import static frc.robot.Constants.Constants.InputConstants.DRIVER_XBOX_CONTROLLER_PORT;
import static frc.robot.Constants.Constants.InputConstants.OPERATOR_XBOX_CONTROLLER_PORT;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TunerConstants;
import static frc.robot.Constants.Constants.ElevatorConstants.*;
//Commands
// import frc.robot.commands.ParallelCommandGroup.IntakeCoralFromFeedStation;
import frc.robot.commands.DriveCommands.DefaultDrive;
// import frc.robot.Simulation.SimulationTele;
import frc.robot.subsystems.AlgeeManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.subsystems.Swerve.Pigon;
import frc.robot.subsystems.Swerve.SwerveSubsystem;






public class RobotContainer {

  private final CommandXboxController  DRIVER_XBOX;
  private final CommandXboxController OPERATOR_XBOX;
  public final SwerveSubsystem SWERVE;
  public final ElevatorSubsystem ELEVATOR;
  public final EndEffectorSubsystem ENDEFFECTOR;
  public final VisionSubsystem VISION; 
  // public final AlgeeManipulatorSubsystem ALGEE;
  // public static final Pigon PIGEON = new Pigon();
  public boolean AlgeeCoralToggle = true; // True Means Coral is selected 
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public RobotContainer() {
    DRIVER_XBOX = new CommandXboxController(DRIVER_XBOX_CONTROLLER_PORT);
    OPERATOR_XBOX = new CommandXboxController(OPERATOR_XBOX_CONTROLLER_PORT);
    SWERVE = TunerConstants.DriveTrain;
    ELEVATOR = new ElevatorSubsystem();
    ENDEFFECTOR = new EndEffectorSubsystem();
    VISION = new VisionSubsystem(); 
    // ALGEE = new AlgeeManipulatorSubsystem();

    //**********************************************************************************
      SWERVE.setDefaultCommand(new DefaultDrive(OPERATOR_XBOX,SWERVE));
    //**********************************************************************************
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // NamedCommands.registerCommand("AlignWithAprilTag", new AlignWithAprilTag(drivetrain, drive,piCamera1));
  
    configureBindings();

  }


  private void configureBindings() {
  //DRIVER CONTROLS -- David  
  
  DRIVER_XBOX.x().onTrue(ELEVATOR.ReefSetpointPositionCommand(L1SetpointC));
  DRIVER_XBOX.y().onTrue(ELEVATOR.ReefSetpointPositionCommand(L2SetpointC));
  DRIVER_XBOX.a().onTrue(ELEVATOR.ReefSetpointPositionCommand(L3SetpointC));
  DRIVER_XBOX.b().onTrue(ELEVATOR.ReefSetpointPositionCommand(L4SetpointC));
  DRIVER_XBOX.rightBumper().onTrue(ENDEFFECTOR.rollerOutCommand());
  DRIVER_XBOX.leftBumper().onTrue(new SequentialCommandGroup(
      new PrintCommand("Commands called"),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(ENDEFFECTOR.CoralEnterSensorTriggered()),
        ENDEFFECTOR.rollerOutCommand()
      ),
      new PrintCommand("Commands ending"),
      createRumbleCommand(OPERATOR_XBOX,1, 2),
      new PrintCommand("Commands rumble")
    ));
  DRIVER_XBOX.rightTrigger().onTrue(SWERVE.SlowSpeedCommand());
  

  if (AlgeeCoralToggle) {
    DRIVER_XBOX.x().onTrue(ELEVATOR.ReefSetpointPositionCommand(L1SetpointC));
    DRIVER_XBOX.y().onTrue(ELEVATOR.ReefSetpointPositionCommand(L2SetpointC));
    DRIVER_XBOX.a().onTrue(ELEVATOR.ReefSetpointPositionCommand(L3SetpointC));
    DRIVER_XBOX.b().onTrue(ELEVATOR.ReefSetpointPositionCommand(L4SetpointC));
  } else {
    DRIVER_XBOX.x().onTrue(ELEVATOR.ReefSetpointPositionCommand(L1SetpointA));
    DRIVER_XBOX.y().onTrue(ELEVATOR.ReefSetpointPositionCommand(L2SetpointA));
    DRIVER_XBOX.a().onTrue(ELEVATOR.ReefSetpointPositionCommand(L3SetpointA));
    DRIVER_XBOX.b().onTrue(ELEVATOR.ReefSetpointPositionCommand(L4SetpointA));
  }
  //OPERATOR CONTROLS -- Jack/Backup

    

  //Test Controls
    
    OPERATOR_XBOX.b().whileTrue(ENDEFFECTOR.rollerOutCommand());
    OPERATOR_XBOX.y().whileTrue(ENDEFFECTOR.rollerInCommand());
    OPERATOR_XBOX.a().toggleOnTrue(SWERVE.SlowSpeedCommand());
    OPERATOR_XBOX.start().onTrue(ELEVATOR.resetElevatorCommand());
    
    OPERATOR_XBOX.povDown().onTrue(ELEVATOR.ReefSetpointPositionCommand(L1SetpointC));
    OPERATOR_XBOX.povLeft().onTrue(ELEVATOR.ReefSetpointPositionCommand(L2SetpointC));
    OPERATOR_XBOX.povRight().onTrue(ELEVATOR.ReefSetpointPositionCommand(L3SetpointC));
    OPERATOR_XBOX.povUp().onTrue(ELEVATOR.ReefSetpointPositionCommand(L4SetpointC));

    OPERATOR_XBOX.rightBumper().onTrue(ELEVATOR.ReefSetpointPositionCommand(0));
    //Intake from Source
    OPERATOR_XBOX.x().onTrue(new SequentialCommandGroup(
      new PrintCommand("Commands called"),
      new ParallelDeadlineGroup(
        new WaitUntilCommand(ENDEFFECTOR.CoralEnterSensorTriggered()),
        ENDEFFECTOR.rollerOutCommand()
      ),
      new PrintCommand("Commands ending"),
      createRumbleCommand(OPERATOR_XBOX,1, 2),
      new PrintCommand("Commands rumble")
    ));

    

  }

  public Command createRumbleCommand(CommandXboxController commandXboxController, double intensity, double duration) {
    return Commands.startEnd(
        () -> commandXboxController.setRumble(GenericHID.RumbleType.kBothRumble, intensity),
        () -> commandXboxController.setRumble(GenericHID.RumbleType.kBothRumble, 0)
    ).withTimeout(duration);
  }

   public static boolean IsRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

  
 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

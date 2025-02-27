// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Phoniex https://api.ctr-electronics.com/phoenix6/release/java/

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.subsystems.Swerve.Pigon;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
// import frc.robot.Simulation.SimulationTele;
// import frc.robot.subsystems.AlgeeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
// import frc.robot.subsystems.PneumaticSubsystem;
import static frc.robot.Constants.Constants.InputConstants.*;

//Commands
// import frc.robot.commands.ParallelCommandGroup.IntakeCoralFromFeedStation;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.commands.DriveCommands.TunePIDDrive;
// import frc.robot.commands.IndividualCommands.DropElevatorToStow;
// import frc.robot.commands.IndividualCommands.RaiseElevatorToMax;






public class RobotContainer {

  private final CommandXboxController  DRIVER_XBOX = new CommandXboxController(DRIVER_XBOX_CONTROLLER_PORT);
  private final CommandXboxController OPERATOR_XBOX = new CommandXboxController(OPERATOR_XBOX_CONTROLLER_PORT);
  public final SwerveSubsystem SWERVE;
  public final ElevatorSubsystem ELEVATOR;
  public final EndEffectorSubsystem ENDEFFECTOR;
  public final VisionSubsystem VISION; 

  // public static final Pigon PIGEON = new Pigon();
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public RobotContainer() {
    SWERVE = TunerConstants.DriveTrain;
    ELEVATOR = new ElevatorSubsystem();
    ENDEFFECTOR = new EndEffectorSubsystem();
    VISION = new VisionSubsystem(); 

    //**********************************************************************************
      SWERVE.setDefaultCommand(new DefaultDrive(DRIVER_XBOX,SWERVE));
    //**********************************************************************************
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // NamedCommands.registerCommand("AlignWithAprilTag", new AlignWithAprilTag(drivetrain, drive,piCamera1));
  
    configureBindings();
  }


  private void configureBindings() {
  //DRIVER CONTROLS
  
  //OPERATOR CONTROLS
    OPERATOR_XBOX.rightBumper().onTrue(ELEVATOR.RaiseUpcommand());
    OPERATOR_XBOX.leftBumper().onTrue(ELEVATOR.LowerDowncommand());
    OPERATOR_XBOX.b().whileTrue(ENDEFFECTOR.rollerOutCommand());
    OPERATOR_XBOX.y().whileTrue(ENDEFFECTOR.rollerInCommand());
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

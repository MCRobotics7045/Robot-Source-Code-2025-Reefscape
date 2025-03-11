// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Phoniex https://api.ctr-electronics.com/phoenix6/release/java/

package frc.robot;


// import frc.robot.subsystems.PneumaticSubsystem;
import static frc.robot.Constants.Constants.InputConstants.DRIVER_XBOX_CONTROLLER_PORT;
import static frc.robot.Constants.Constants.InputConstants.OPERATOR_XBOX_CONTROLLER_PORT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TunerConstants;
import static frc.robot.Constants.Constants.ElevatorConstants.*;

import frc.robot.commands.DriveCommands.AlignCommand;
//Commands
// import frc.robot.commands.ParallelCommandGroup.IntakeCoralFromFeedStation;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.commands.DriveCommands.DriveAndAlignReefCommand;
// import frc.robot.Simulation.SimulationTele;
import frc.robot.subsystems.AlgeeManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.SensorsIO;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.PneumaticSubsytem;

import static frc.robot.Constants.Constants.SensorIOConstants.*;


public class RobotContainer {

  public static  CommandXboxController  DRIVER_XBOX;
    public static CommandXboxController OPERATOR_XBOX;

    public final ElevatorSubsystem ELEVATOR;
    public final EndEffectorSubsystem ENDEFFECTOR;
    public static VisionSubsystem VISION; 
    public final PneumaticSubsytem PNEUMATICS;
    public final SwerveSubsystem SWERVE;
        // public final AlgeeManipulatorSubsystem ALGEE;
        public static SensorsIO SENSORS;
        public boolean AlgeeCoralToggle = true; // True Means Coral is selected 
          
        SendableChooser<Command> autoChooser = new SendableChooser<>();
    public RobotContainer() {
        DRIVER_XBOX = new CommandXboxController(DRIVER_XBOX_CONTROLLER_PORT);
        OPERATOR_XBOX = new CommandXboxController(OPERATOR_XBOX_CONTROLLER_PORT);

        ELEVATOR = new ElevatorSubsystem();
        ENDEFFECTOR = new EndEffectorSubsystem();
        VISION = new VisionSubsystem(); 
        SWERVE = TunerConstants.createDrivetrain();
        PNEUMATICS = new PneumaticSubsytem();
        // ALGEE = new AlgeeManipulatorSubsystem();
        SENSORS = new SensorsIO();
        
      //**********************************************************************************
        SWERVE.setDefaultCommand(new DefaultDrive(OPERATOR_XBOX,SWERVE));
      //**********************************************************************************
      NamedCommands.registerCommand("CoralL3Set", ELEVATOR.ReefSetpointPositionCommand(L3SetpointC));
      NamedCommands.registerCommand("Fire Coral", ENDEFFECTOR.rollerOutCommand());
      NamedCommands.registerCommand("CoralL2Set", ELEVATOR.ReefSetpointPositionCommand(L2SetpointC));
      NamedCommands.registerCommand("AlignAprilTag", ENDEFFECTOR.rollerInCommand());
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
      SmartDashboard.putNumber("Threashold", Threashold);
      // NamedCommands.registerCommand("AlignWithAprilTag", new AlignWithAprilTag(drivetrain, drive,piCamera1));
      
      configureBindings();
  
    }
  
  
    private void configureBindings() {
    //DRIVER CONTROLS -- David  
    
  
    // //Normal Coral Setpoints
    // DRIVER_XBOX.x().onTrue(Commands.runOnce(() -> {
    //   if (AlgeeCoralToggle) { //Coral
    //     ELEVATOR.ReefSetpointPositionCommand(L1SetpointC);
    //   } else { //Algee
    //     ELEVATOR.ReefSetpointPositionCommand(L1SetpointA);
    //   }}));
  
    // DRIVER_XBOX.y().onTrue(Commands.runOnce(() -> {
    //   if (AlgeeCoralToggle) {
    //     ELEVATOR.ReefSetpointPositionCommand(L2SetpointC);
    //   } else {
    //     ELEVATOR.ReefSetpointPositionCommand(L2SetpointA);
    //   }}));
  
    // DRIVER_XBOX.a().onTrue(Commands.runOnce(() -> {
    //   if (AlgeeCoralToggle) {
    //     ELEVATOR.ReefSetpointPositionCommand(L3SetpointC);
    //   } else {
    //     ELEVATOR.ReefSetpointPositionCommand(L3SetpointA);
    //   }}));
      
    // DRIVER_XBOX.b().onTrue(ELEVATOR.ReefSetpointPositionCommand(L4SetpointC));
    
      
    // DRIVER_XBOX.start().onTrue(Commands.runOnce(() -> {AlgeeCoralToggle = !AlgeeCoralToggle;}));
    // DRIVER_XBOX.rightBumper().whileTrue(ENDEFFECTOR.rollerOutCommand());
  
    // DRIVER_XBOX.x().onTrue(new SequentialCommandGroup(
    //   new PrintCommand("Intake Run called"),
    //   new ParallelDeadlineGroup(
    //     new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered()).andThen(new WaitUntilCommand(SENSORS.OppCoralRampEnterSensorTriggered())),
    //     ENDEFFECTOR.rollerOutCommand()
    //   ),
    //   new PrintCommand("Coral Grabbed Ready To Move"),
    //   createRumbleCommand(2,1, 1),
    //   new ParallelDeadlineGroup(
    //     new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered()),
    //     ENDEFFECTOR.SetRollerSpeed(0.5)
    //   )
    // ));
  
    
    // DRIVER_XBOX.leftTrigger().onTrue(new DriveAndAlignReefCommand(SWERVE, VISION, true));
  
    //OPERATOR CONTROLS -- Jack/Backup
  
      
  
    //Test Controls
      OPERATOR_XBOX.leftTrigger().whileTrue(ELEVATOR.UnspoolCommand());
      OPERATOR_XBOX.leftBumper().onTrue(new AlignCommand(SWERVE, VISION, true, true));
      OPERATOR_XBOX.b().whileTrue(ENDEFFECTOR.rollerOutCommand());
      OPERATOR_XBOX.y().whileTrue(ENDEFFECTOR.rollerInCommand());
      // OPERATOR_XBOX.a().toggleOnTrue(SWERVE.SlowSpeedCommand());
      OPERATOR_XBOX.start().onTrue(ELEVATOR.resetElevatorCommand());
      OPERATOR_XBOX.back().onTrue(SENSORS.ZeroPigeonIMU());
      OPERATOR_XBOX.povDown().onTrue(ELEVATOR.ReefSetpointPositionCommand(L1SetpointC));
      OPERATOR_XBOX.povLeft().onTrue(ELEVATOR.ReefSetpointPositionCommand(L2SetpointC));
      OPERATOR_XBOX.povRight().onTrue(ELEVATOR.ReefSetpointPositionCommand(L3SetpointC));
      OPERATOR_XBOX.povUp().onTrue(ELEVATOR.ReefSetpointPositionCommand(L4SetpointC));
      OPERATOR_XBOX.rightTrigger().whileTrue(ENDEFFECTOR.ChangeEndEffectorRollerSpeed(0.3));
      OPERATOR_XBOX.rightTrigger().onFalse(ENDEFFECTOR.ChangeEndEffectorRollerSpeed(1.0));
      OPERATOR_XBOX.rightBumper().onTrue(ELEVATOR.ReefSetpointPositionCommand(0));

      
      //Sys ID
      OPERATOR_XBOX.back().and(OPERATOR_XBOX.y()).whileTrue(SWERVE.sysIdDynamic(SysIdRoutine.Direction.kForward));
      OPERATOR_XBOX.back().and(OPERATOR_XBOX.x()).whileTrue(SWERVE.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      OPERATOR_XBOX.start().and(OPERATOR_XBOX.y()).whileTrue(SWERVE.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      OPERATOR_XBOX.start().and(OPERATOR_XBOX.x()).whileTrue(SWERVE.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));


      OPERATOR_XBOX.a().and(OPERATOR_XBOX.b()).onTrue(PNEUMATICS.Extend());

      
      //Intake from Source
      OPERATOR_XBOX.x().onTrue(new SequentialCommandGroup(
        new PrintCommand("Intake Run called"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered()).andThen(new WaitUntilCommand(SENSORS.OppCoralRampEnterSensorTriggered())),
          ENDEFFECTOR.rollerOutCommand()
        ),
        new PrintCommand("Coral Grabbed Ready To Move"),
        createRumbleCommand(2,1, 1),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered()),
          ENDEFFECTOR.SetRollerSpeed(0.5)
        )
      ));
  
      
  
    }
  
    public static Command createRumbleCommand(int commandXboxController, double intensity, double duration) {
      if (commandXboxController == 1) {
        return Commands.startEnd(
          () -> DRIVER_XBOX.setRumble(GenericHID.RumbleType.kBothRumble, intensity),
        () -> DRIVER_XBOX.setRumble(GenericHID.RumbleType.kBothRumble, 0)
    ).withTimeout(duration);
    } else {
      return Commands.startEnd(
        () -> OPERATOR_XBOX.setRumble(GenericHID.RumbleType.kBothRumble, intensity),
        () -> OPERATOR_XBOX.setRumble(GenericHID.RumbleType.kBothRumble, 0)
    ).withTimeout(duration);
    }
    
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

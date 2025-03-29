
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Phoniex https://api.ctr-electronics.com/phoenix6/release/java/

package frc.robot;


import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Constants.ElevatorConstants.HoldBelowL2;
import static frc.robot.Constants.Constants.ElevatorConstants.L1SetpointC;
import static frc.robot.Constants.Constants.ElevatorConstants.L2SetpointC;
import static frc.robot.Constants.Constants.ElevatorConstants.L3SetpointC;
import static frc.robot.Constants.Constants.ElevatorConstants.L4SetpointC;
// import frc.robot.subsystems.PneumaticSubsystem;
import static frc.robot.Constants.Constants.InputConstants.DRIVER_XBOX_CONTROLLER_PORT;
import static frc.robot.Constants.Constants.InputConstants.OPERATOR_XBOX_CONTROLLER_PORT;
import static frc.robot.Constants.Constants.SensorIOConstants.Threashold;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.AutoCommands.DriveForwardToTag;
import frc.robot.commands.AutoCommands.DriveOffset;
import frc.robot.commands.AutoCommands.DriveToPostOffset;
//Commands
// import frc.robot.commands.ParallelCommandGroup.IntakeCoralFromFeedStation;
import frc.robot.commands.DriveCommands.DefaultDrive;
// import frc.robot.commands.DriveCommands.DriveWithJoystick;
// import frc.robot.commands.DriveCommands.DriveAndAlignReefCommand;
// import frc.robot.Simulation.SimulationTele;
// import frc.robot.subsystems.AlgeeManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.SensorsIO;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.TalonFXElevatorSubsytem;

public class RobotContainer {

  public static  CommandXboxController  DRIVER_XBOX;
  public static CommandJoystick DRIVER_JOYSTICK;
    public static CommandXboxController OPERATOR_XBOX;

    public static SwerveSubsystem SWERVE;
    public final TalonFXElevatorSubsytem elevatorSubsytem;
    public final EndEffectorSubsystem ENDEFFECTOR;
    public static VisionSubsystem VISION; 
   

    public static int ElevatorSet;
    // public final PneumaticSubsytem PNEUMATICS;
  
        // public final AlgeeManipulatorSubsystem ALGEE;
        public static SensorsIO SENSORS;
        public final LEDSubsystem LED;
        public boolean AlgeeCoralToggle = true; // True Means Coral is selected 
          
        SendableChooser<Command> autoChooser = new SendableChooser<>();
        public static AprilTagFieldLayout kTagLayouts = null;
                
         public static AprilTagFieldLayout getTagLayout() {
            if (kTagLayouts == null) {
              kTagLayouts = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            }
                return kTagLayouts;
            }

    
    Trigger leftTrigger;
    Trigger rightTrigger;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public RobotContainer() {
        ElevatorSet = -40;
        DRIVER_XBOX = new CommandXboxController(DRIVER_XBOX_CONTROLLER_PORT);
        OPERATOR_XBOX = new CommandXboxController(OPERATOR_XBOX_CONTROLLER_PORT);
        DRIVER_JOYSTICK = new CommandJoystick(DRIVER_XBOX_CONTROLLER_PORT);
        elevatorSubsytem = new TalonFXElevatorSubsytem(); 
        ENDEFFECTOR = new EndEffectorSubsystem();
        LED = new LEDSubsystem();
        SWERVE = TunerConstants.createDrivetrain();
        VISION = new VisionSubsystem(SWERVE::addVisionMeasurement); 
        // PNEUMATICS = new PneumaticSubsytem();
        // ALGEE = new AlgeeManipulatorSubsystem();
        SENSORS = new SensorsIO();
        
      //**********************************************************************************
        SWERVE.setDefaultCommand(new DefaultDrive(DRIVER_XBOX,SWERVE));
        // SWERVE.setDefaultCommand(new DriveWithJoystick(DRIVER_JOYSTICK, SWERVE));
      //**********************************************************************************
      NamedCommands.registerCommand("Intake", new SequentialCommandGroup(
        new PrintCommand("Intake Run called"),
        // ELEVATOR.DropElevator(), // IF IT BREAKS IT OR COMMAND DOESNT RUN REMOVE THIS 
        new ParallelDeadlineGroup(
          new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered()).andThen(new WaitUntilCommand(SENSORS.OppCoralRampEnterSensorTriggered())),
          ENDEFFECTOR.rollerOutCommand()
        ),
        new PrintCommand("Coral Grabbed Ready To Move"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered()),
          ENDEFFECTOR.SetRollerSpeed(0.5)
        ),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(SENSORS.OppCoralRampEnterSensorTriggered()).andThen(new WaitCommand(0.01)),
          ENDEFFECTOR.SetRollerSpeed(-0.2)
        )
      ));
      NamedCommands.registerCommand("LowerElevator", elevatorSubsytem.DropElevator());
      NamedCommands.registerCommand("Intake Coral", new SequentialCommandGroup(
        new PrintCommand("Intake Run called"),
        // ELEVATOR.DropElevator(), // IF IT BREAKS IT OR COMMAND DOESNT RUN REMOVE THIS 
        new ParallelDeadlineGroup(
          new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered()).andThen(new WaitUntilCommand(SENSORS.OppCoralRampEnterSensorTriggered())),
          ENDEFFECTOR.rollerOutCommand()
        ),
        new PrintCommand("Coral Grabbed Ready To Move"),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered()),
          ENDEFFECTOR.SetRollerSpeed(0.5)
        ),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(SENSORS.OppCoralRampEnterSensorTriggered()).andThen(new WaitCommand(0.1)),
          ENDEFFECTOR.SetRollerSpeed(-0.3)
        )
      ));
      NamedCommands.registerCommand("Fire Coral", ENDEFFECTOR.rollerOutCommand());
      NamedCommands.registerCommand("CoralL1Set", elevatorSubsytem.ReefSetpointPositionCommand(L1SetpointC));
      NamedCommands.registerCommand("CoralL2Set", elevatorSubsytem.ReefSetpointPositionCommand(L2SetpointC));
      NamedCommands.registerCommand("CoralL3Set", elevatorSubsytem.ReefSetpointPositionCommand(L3SetpointC));
      NamedCommands.registerCommand("CoralL4Set", elevatorSubsytem.ReefSetpointPositionCommand(L4SetpointC));
      NamedCommands.registerCommand("AutoAlignLeftL4", new DriveToPostOffset(SWERVE, VISION.bPCamera, 0.2, -0.10));
      NamedCommands.registerCommand("AutoAlignLeftL3-L2", new DriveToPostOffset(SWERVE, VISION.bPCamera, 0.2, -.06));
      // NamedCommands.registerCommand("AlignAprilTag", new DriveAndAlignReefCommand(SWERVE, VISION, true));
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
      SmartDashboard.putNumber("Threashold", Threashold);
      
      // NamedCommands.registerCommand("AlignWithAprilTag", new AlignWithAprilTag(drivetrain, drive,piCamera1));
      
      
      configureBindings();
  
    }
  
  
    private void configureBindings() {


  //   SWERVE.setDefaultCommand(SWERVE.applyRequest(() ->
  //   drive.withVelocityX(-DRIVER_XBOX.getLeftY() * 2) // Drive forward with negative Y (forward)
  //       .withVelocityY(-DRIVER_XBOX.getLeftX() * 2) // Drive left with negative X (left)
  //       .withRotationalRate(-DRIVER_XBOX.getRightX() * 1) // Drive counterclockwise with negative X (left)
  // )); 
    //DRIVER CONTROLS -- David  
    rightTrigger = new Trigger(()-> DRIVER_XBOX.getRightTriggerAxis() > 0.1);
    leftTrigger = new Trigger(()-> DRIVER_XBOX.getLeftTriggerAxis() > 0.1);
  
    // //Normal Coral Setpoints
   
  


    DRIVER_XBOX.rightBumper().whileTrue(ENDEFFECTOR.rollerOutCommand());
    //A 0
    //X .04 // NOT L2 
    //Y .06
    //B .10
    
    rightTrigger.and(DRIVER_XBOX.x()).whileTrue(new DriveOffset(SWERVE, VISION.bPCamera, true));
    
    leftTrigger.and(DRIVER_XBOX.b()).whileTrue(new DriveToPostOffset(SWERVE, VISION.bPCamera, 0.2, -.10)); // L4 B
    leftTrigger.and(DRIVER_XBOX.y()).whileTrue(new DriveToPostOffset(SWERVE, VISION.bPCamera, 0.2, -.06)); //L2 X
    leftTrigger.and(DRIVER_XBOX.x()).whileTrue(new DriveToPostOffset(SWERVE, VISION.bPCamera, 0.2, -.06)); //L3 Y

    // DRIVER_XBOX.povDown().onTrue(ALGEE.StowPostion());
    // DRIVER_XBOX.povUp().onTrue(ALGEE.HoldCommand());
    // DRIVER_XBOX.povLeft().onTrue(ALGEE.dropOutCommand());

      // OPERATOR_XBOX.povDown().onTrue(Commands.runOnce(()-> { 
      //   ElevatorSet--;
      //   System.out.println("New Elevator SetPoint" + ElevatorSet);
      //   ELEVATOR.ReefSetpointPositionCommand(ElevatorSet).schedule();
      //   })
      // );
      // OPERATOR_XBOX.povUp().onTrue(Commands.runOnce(()-> { 
      //   ElevatorSet++;
      //   System.out.println("New Elevator SetPoint" + ElevatorSet);
      //   ELEVATOR.ReefSetpointPositionCommand(ElevatorSet).schedule();
      //   })
      // );
      
      // OPERATOR_XBOX.povDown().whileTrue(elevatorSubsytem.UnspoolCommand()
      // );
      OPERATOR_XBOX.povUp().whileTrue(ENDEFFECTOR.LaunchCommand()
      );
      OPERATOR_XBOX.povRight().whileTrue(ENDEFFECTOR.rollerOutCommand());
      OPERATOR_XBOX.povLeft().whileTrue(ENDEFFECTOR.rollerInCommand());

      // OPERATOR_XBOX.start().onTrue(ELEVATOR.resetElevatorCommand());
      OPERATOR_XBOX.back().onTrue(SWERVE.runOnce(()-> SWERVE.seedFieldCentric()));


      // //Sys ID
      

      //A L1
      //X L2
      //Y L3 
      //B L4
      OPERATOR_XBOX.a().whileTrue(elevatorSubsytem.ReefSetpointPositionCommand(4));
      OPERATOR_XBOX.x().whileTrue(elevatorSubsytem.ReefSetpointPositionCommand(L2SetpointC));
      OPERATOR_XBOX.y().whileTrue(elevatorSubsytem.ReefSetpointPositionCommand(L3SetpointC)); //OLD 100
      OPERATOR_XBOX.b().whileTrue(elevatorSubsytem.ReefSetpointPositionCommand(L4SetpointC)); // -150


      OPERATOR_XBOX.rightBumper().whileTrue(elevatorSubsytem.DropElevator());
      OPERATOR_XBOX.leftBumper().onTrue(intakeCoralCommand());
  
      
  
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

  

  public Command DropElevatorToStowPostion() {
    return new ParallelCommandGroup(
      // ALGEE.StowPostion(),
      elevatorSubsytem.DropElevator()
    );
  }


  public Command intakeCoralCommand() {
    return new SequentialCommandGroup(
      new PrintCommand("Intake Run called"),
      LED.ChangeColor(1), //Find Open Angle
      new ParallelDeadlineGroup(
        new WaitUntilCommand(SENSORS.CoralRampEnterSensorTriggered())
        .andThen(new WaitUntilCommand(SENSORS.OppCoralRampEnterSensorTriggered()))
        .andThen(new WaitCommand(0.05)),
        ENDEFFECTOR.SetRollerSpeed(-0.5)
      ),
      //Find Close Angle
      new PrintCommand("Coral Grabbed Ready To Move"),
      LED.ChangeColor(4),
      createRumbleCommand(1,1, 1),
      
      new ParallelDeadlineGroup(
        new WaitCommand(0.05),
        ENDEFFECTOR.SetRollerSpeed(0.5)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.05),
        ENDEFFECTOR.SetRollerSpeed(-0.5)
      ),
      LED.ChangeColor(4)
      
    );
  }

  public Command GrabAlgaeFromReef() {
    return new SequentialCommandGroup(

    );
  }


  public void EncoderUp() {
    ElevatorSet =+ 1;
  }
}

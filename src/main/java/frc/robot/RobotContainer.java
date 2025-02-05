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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TunerConstants;


import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve.Pigon;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.Simulation.SimulationTele;
import frc.robot.subsystems.AlgeeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import static frc.robot.Constants.Constants.InputConstants.*;

//Commands
import frc.robot.commands.ParallelCommandGroup.IntakeCoralFromFeedStation;
import frc.robot.commands.DriveCommands.DefaultDrive;
import frc.robot.commands.DriveCommands.TunePIDDrive;
import frc.robot.commands.IndividualCommands.RollerOut;
import frc.robot.commands.DriveCommands.AlignInFrontofTagCommand;


public class RobotContainer {

   private final XboxController XBOX = new XboxController(XBOX_CONTROLLER_PORT);

  public static final Pigon PIGEON = new Pigon();
  public static final VisionSubsystem VISION  = new VisionSubsystem(); 
  // public static final SimulationTele SIMULATION_TELE = new SimulationTele();
  public static final GripperSubsystem GRIPPER = new GripperSubsystem();
  public static final ElevatorSubsystem ELEVATOR = new ElevatorSubsystem();

  public static final PneumaticSubsystem PNEUMATIC = new PneumaticSubsystem();
  public static final AlgeeIntakeSubsystem ALGEE_INTAKE = new AlgeeIntakeSubsystem();
  public static final SwerveSubsystem SWERVE = TunerConstants.DriveTrain;
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  public RobotContainer() {
    
    System.out.println("Robot Container Started ");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // NamedCommands.registerCommand("AlignWithAprilTag", new AlignWithAprilTag(drivetrain, drive,piCamera1));

    SWERVE.setDefaultCommand(new DefaultDrive(XBOX,SWERVE));
    configureBindings();
  }


  private void configureBindings() {
    final POVButton dPadRight = new POVButton(XBOX, 90);
    final POVButton dPadLeft = new POVButton(XBOX, 270);
    final POVButton dPadUp = new POVButton(XBOX, 0);
    final POVButton dPadDown = new POVButton(XBOX, 180);
    final JoystickButton buttonY = new JoystickButton(XBOX, xboxYellowButton);
    final JoystickButton buttonA = new JoystickButton(XBOX, xboxGreenButton);   
    final JoystickButton buttonX = new JoystickButton(XBOX, xboxBlueButton);
    final JoystickButton buttonB = new JoystickButton(XBOX, xboxRedButton);
  
    final JoystickButton buttonRB = new JoystickButton(XBOX, xboxRBButton);
    final JoystickButton buttonLB = new JoystickButton(XBOX, xboxLBButton);
    // final JoystickButton buttonStart = new JoystickButton(XBOX, xboxStartButton);
    // final JoystickButton buttonMENU = new JoystickButton(XBOX, xboxMenuButton);

    buttonA.whileTrue(new TunePIDDrive(SWERVE));
    buttonRB.whileTrue(new AlignInFrontofTagCommand(SWERVE, VISION, VISION.postionCamera, XBOX));
    buttonLB.onTrue(new IntakeCoralFromFeedStation(ELEVATOR, GRIPPER));
    dPadUp.onTrue(new InstantCommand(ELEVATOR::RaiseMax));  //Raise Elevator
    dPadDown.onTrue(new InstantCommand(ELEVATOR::LowerMax));  //Lower Elevator
    // buttonY.onTrue(new InstantCommand(ELEVATOR::StopMotor)); 
    dPadLeft.whileTrue(  //Spit Coral Out
      new RollerOut(GRIPPER)
    );  
    dPadRight.whileTrue(  //Suck Coral In
      new RunCommand(GRIPPER::RollerIn, GRIPPER)
        .andThen(() -> GRIPPER.StopRoller())
    );
    
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

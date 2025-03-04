// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.Constants.EndEffectorConstants.*;

import java.util.function.BooleanSupplier;

import frc.robot.RobotContainer;
// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new End Effector. */
  public SparkMax TopMotor;
  public SparkMax BottomMotor;
  public SparkClosedLoopController topMClosedLoopController;
  public SparkClosedLoopController bottomMClosedLoopController;
  public RelativeEncoder Top_Encoder;
  public RelativeEncoder Bottom_Encoder;
  public SparkMaxConfig config;
  SparkFlexConfig configClosedLoop ;
  DigitalInput CoralEnterSensor = new DigitalInput(CoralEnterSensorID);
  DigitalInput CoralExitSensor = new DigitalInput(CoralExitSensorID);
  
  public EndEffectorSubsystem() {
    TopMotor = new SparkMax(Top_MotorID, MotorType.kBrushless);
    BottomMotor = new SparkMax(Bottom_MotorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0.1)
      .inverted(true);


    configClosedLoop = new SparkFlexConfig();

    configClosedLoop.closedLoop //I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS 
      .p(1);
      // .i(0.1)
      // .d(0.1);
  
    TopMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    TopMotor.configure(configClosedLoop, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    BottomMotor.configure(configClosedLoop, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Top_Encoder = TopMotor.getEncoder();
    Bottom_Encoder = BottomMotor.getEncoder();
    topMClosedLoopController = TopMotor.getClosedLoopController();
    bottomMClosedLoopController = BottomMotor.getClosedLoopController();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Logger.recordOutput("Gripper Encoder Speed", Top_Encoder.getVelocity());
    // Logger.recordOutput("Is Gripper Enaged", RollerEngaged());
    SmartDashboard.putNumber("End Effector Encoder Speed", Top_Encoder.getVelocity());
    SmartDashboard.putBoolean("Coral SWitch", CoralEnterSensor.get());


  
      

  }
// i dont know what to call it. Spit out? Exhast? Get rid of? 

  public void RollerOut() {
    TopMotor.set(MotorFowardSpeed);
    BottomMotor.set(MotorReverseSpeed);
    System.out.println("Roller Out Called ");
  }

  public Command rollerOutCommand() {
    return Commands.startEnd(() -> RollerOut(),() -> StopRoller(), this);
  } 

  public void RollerIn() {
    TopMotor.set(MotorReverseSpeed);
    BottomMotor.set(MotorFowardSpeed);
    System.out.println("Roller In Called ");

  }

  public Command rollerInCommand() {
    return Commands.startEnd(() -> RollerIn(),() -> StopRoller(), this);
  } 

  //COMMAND FOR SLOW SPEED FOR L1

  public void SetSpeed(double speedSet) {
    TopMotor.set(speedSet);
    BottomMotor.set(speedSet);
  }

  public void RollerIntakeCoral() {
    TopMotor.set(MotorIntakeSpeed);
    BottomMotor.set(MotorIntakeSpeed);
  }
  
  public void StopRoller(){
    TopMotor.stopMotor();
    BottomMotor.stopMotor();
  }

  public Command rollerStopCommand() {
    return Commands.startEnd(()-> StopRoller(), ()-> StopRoller(), this);
  }

  public boolean RollerEngaged() {
    if (Top_Encoder.getVelocity() < 0 && Top_Encoder.getVelocity() > 0) {
      return true;
    } else {
      return false;
    }
  }

  public BooleanSupplier CoralEnterSensorTriggered() {
    return () -> !CoralEnterSensor.get();
  }

  public boolean CoralExitSensorTriggered() {
    if (CoralExitSensor.get()) {
      return false;
    } else {
      return true;
    }
  }
}

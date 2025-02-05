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
import static frc.robot.Constants.Constants.GripperConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
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
  
  public GripperSubsystem() {
    TopMotor = new SparkMax(Top_MotorID, MotorType.kBrushless);
    BottomMotor = new SparkMax(Bottom_MotorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0.1)
      .inverted(true);

    configClosedLoop = new SparkFlexConfig();

    configClosedLoop.closedLoop //I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS I HATE PIDS 
      .p(0.1)
      .i(0.1)
      .d(0.1);
  
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
    Logger.recordOutput("Gripper roller Speed", TopMotor.get());
    Logger.recordOutput("Is Gripper Enaged", RollerEngaged());
    SmartDashboard.putNumber("Gripper roller Speed", TopMotor.get());


  }
// i dont know what to call it. Spit out? Exhast? Get rid of? 

  public void RollerOut() {
    TopMotor.set(MotorFowardSpeed);
    BottomMotor.set(MotorFowardSpeed);
    System.out.println("Roller Out Called ");
  }

  public void RollerIn() {
    TopMotor.set(MotorReverseSpeed);
    BottomMotor.set(MotorReverseSpeed);
    System.out.println("Roller In Called ");
  }

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
    System.out.println("Stop Motor Called ");
  }

  public boolean RollerEngaged() {
    if (Top_Encoder.getVelocity() < 0 && Top_Encoder.getVelocity() > 0) {
      return true;
    } else {
      return false;
    }
  }

  public boolean CoralEnterSensorTriggered() {
    if (CoralEnterSensor.get()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean CoralExitSensorTriggered() {
    if (CoralExitSensor.get()) {
      return true;
    } else {
      return false;
    }
  }
}

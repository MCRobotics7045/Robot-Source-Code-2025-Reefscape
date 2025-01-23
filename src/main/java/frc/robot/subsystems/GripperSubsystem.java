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
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.Constants.GripperConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  public SparkMax coralMotor;
  public SparkClosedLoopController coralMotorClosedLoopController;
  public RelativeEncoder coral_Encoder;
  public SparkMaxConfig config;
  DigitalInput CoralEnterSensor = new DigitalInput(CoralEnterSensorID);
  DigitalInput CoralExitSensor = new DigitalInput(CoralExitSensorID);
  
  public GripperSubsystem() {
    coralMotor = new SparkMax(Coral_MotorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0.1)
      .inverted(true);

    coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coral_Encoder = coralMotor.getEncoder();
    coralMotorClosedLoopController = coralMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Gripper roller Speed", coralMotor.get());
    Logger.recordOutput("Is Gripper Enaged", RollerEngaged());
    SmartDashboard.putNumber("Gripper roller Speed", coralMotor.get());


  }
// i dont know what to call it. Spit out? Exhast? Get rid of? 

  public void RollerOut() {
    coralMotor.set(MotorFowardSpeed);
  }

  public void RollerIn() {
    coralMotor.set(MotorReverseSpeed);
  }

  public void SetSpeed(double speedSet) {
    coralMotor.set(speedSet);
  }

  public void RollerIntakeCoral() {
    coralMotor.set(MotorIntakeSpeed);
  }
  
  public void StopRoller(){
    coralMotor.stopMotor();;
  }

  public boolean RollerEngaged() {
    if (coral_Encoder.getVelocity() < 0 && coral_Encoder.getVelocity() > 0) {
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

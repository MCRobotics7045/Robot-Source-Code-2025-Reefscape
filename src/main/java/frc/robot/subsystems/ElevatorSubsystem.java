// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Constants.ElevatorConstants.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.littletonrobotics.junction.Logger;


public class ElevatorSubsystem extends SubsystemBase {

  SparkMax Elev_Motor;
  SparkClosedLoopController Elev_Motor_controller;
  RelativeEncoder Elevator_encoder;
  SparkMaxConfig config;
  double Elevator_Pos, Elv_proccsesvarible;
  DigitalInput MaxHeightSensor = new DigitalInput(MaxHeightSensorID);
  DigitalInput StowPostiontSensor = new DigitalInput(StowPostiontSensorID);

  public ElevatorSubsystem() {
//Setup Motor
    Elev_Motor = new SparkMax(Elevator_MotorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      .openLoopRampRate(0.1)
      .inverted(false);
    Elev_Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Elev_Motor_controller = Elev_Motor.getClosedLoopController();
    Elevator_encoder = Elev_Motor.getEncoder();
    

    //PID TUNING DO YOU KNOW I HATE PIDS THEY SUCK I DONT GET WHY THE EVEN EXIST LIKE WHY ARE THEY NAMED PID LIKE WHO NAMED THEM PROPORTINAL INTERGAL AND DERIVITATE AND IF THAT WASNT ENOUGH THEY ADDED THREE MORE FOR THE FUN OF IT LIKE ARE YOU SERIOUS HOW STUID ARE YOU HOW BOUT YOU TEACH YOURSELF SOMETHING WHERE NO ONE REALY UNDERSTANDS HOW TO EXPLAIN IT BUT THEY UNDERSTAND IT THEMSELF HUH???
    //i didnt do it by the way. nor will i and ill just say i did it and blame it on the elevator build :D
    
  }


  @Override
  public void periodic() {
    Logger.recordOutput("Elevator Height", Elevator_encoder.getPosition());
    Logger.recordOutput("Elevator Stowed", StowPostionSwitch());
    Logger.recordOutput("Elevator Engaged", ElevatorEngaged());

    SmartDashboard.putNumber("Elevator Height", Elevator_encoder.getPosition());
    SmartDashboard.putBoolean("Elevator Stowed", StowPostionSwitch());
    SmartDashboard.putBoolean("Elevator Engaged", ElevatorEngaged());
    SmartDashboard.putBoolean("Elevator Max Height", MaxHeightSwitch());
  }

  public void RaiseMax() {
    Elevator_Pos = MaxHeight;
    Elev_Motor_controller.setReference(Elevator_Pos, SparkBase.ControlType.kMAXMotionPositionControl);
    Elv_proccsesvarible = Elevator_encoder.getPosition();
  }

  public void LowerMax() {
    Elevator_Pos = StowPostion;
    Elev_Motor_controller.setReference(Elevator_Pos, SparkBase.ControlType.kMAXMotionPositionControl);
    Elv_proccsesvarible = Elevator_encoder.getPosition();
  }

  public void setHeight(double SetPoint) {
    Elevator_Pos = SetPoint;
    Elev_Motor_controller.setReference(Elevator_Pos, SparkBase.ControlType.kMAXMotionPositionControl);
    Elv_proccsesvarible = Elevator_encoder.getPosition();
  }

  public void setPointL2() {
    Elevator_Pos = L2Height;
    Elev_Motor_controller.setReference(Elevator_Pos, SparkBase.ControlType.kMAXMotionPositionControl);
    Elv_proccsesvarible = Elevator_encoder.getPosition();
  }

  public void setPointL3() {
    Elevator_Pos = L3Height;
    Elev_Motor_controller.setReference(Elevator_Pos, SparkBase.ControlType.kMAXMotionPositionControl);
    Elv_proccsesvarible = Elevator_encoder.getPosition();
  }

  public boolean MaxHeightSwitch() {
    if (MaxHeightSensor.get() == true) {
      return true;
    } else {
      return false;
    }
  }

  public boolean StowPostionSwitch() {
    if (StowPostiontSensor.get() == true) {
      return true;
    } else {
      return false;
    }
  }

  public boolean ElevatorEngaged() {
    if (StowPostiontSensor.get() == false && MaxHeightSensor.get() == false) {
      return true;
    } else {
      return false;
    }
  }
   //ahhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
}

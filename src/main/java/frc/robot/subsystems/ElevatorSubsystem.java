// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.ElevatorConstants;

import static frc.robot.Constants.Constants.ElevatorConstants.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
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
  int cylce = 0;
  private boolean StowPostionSensorBoolean = true;
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
    //JACK IF YOU DONT SEE CHANGES WHEN PID GETS CHANGED CREATE A NEW CONFIG PARAMETER CAUSE ITS PROPALLY NOT SEQUENTIAL 1/27/2025
    config.closedLoop // pid pid pid help 
      .p(2.5)
      .i(0)
      .d(1.3);

    Elev_Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Elev_Motor_controller = Elev_Motor.getClosedLoopController();
    Elevator_encoder = Elev_Motor.getEncoder();
    
  

// Set MAXMotion parameters
    config.closedLoop.maxMotion
      .maxVelocity(10000)
      .maxAcceleration(8000)
      .allowedClosedLoopError(1);

    Elev_Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //PID TUNING DO YOU KNOW I HATE PIDS THEY SUCK I DONT GET WHY THE EVEN EXIST LIKE WHY ARE THEY NAMED PID LIKE WHO NAMED THEM PROPORTINAL INTERGAL AND DERIVITATE AND IF THAT WASNT ENOUGH THEY ADDED THREE MORE FOR THE FUN OF IT LIKE ARE YOU SERIOUS HOW STUID ARE YOU HOW BOUT YOU TEACH YOURSELF SOMETHING WHERE NO ONE REALY UNDERSTANDS HOW TO EXPLAIN IT BUT THEY UNDERSTAND IT THEMSELF HUH???
    //i didnt do it by the way. nor will i and ill just say i did it and blame it on the elevator build :D
    
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Height", Elevator_encoder.getPosition());
    SmartDashboard.putBoolean("Elevator Stowed", StowPostiontSensor.get());
    SmartDashboard.putBoolean("Elevator Engaged", StowPostionSensorBoolean);
    
    // if (!StowPostiontSensor.get()) { 
    //   StopMotor();
    //   Elevator_encoder.setPosition(0);
    //   System.out.println("Stow sensor triggered: Elevator stopped and encoder reset.");
    // }
  }

  public double calculateEncoderSetpoint(double desiredTravelMeters) {
      double circumference = Math.PI * ElevatorConstants.SPOOL_DIAMETER_METERS;
      double outputRotations = desiredTravelMeters / circumference;
      return outputRotations * ElevatorConstants.GEARBOX_RATIO;
  }

  // public void LowerElevator() {
  //   //Checks to make sure Encoder isnt gonna unspool it. 
  //   if (Elevator_encoder.getPosition() >= 0 || StowPostiontSensor.get() == false) {
  //     StopMotor();
  //   } else {
  //     Elev_Motor.set(1);
  //   }

  // }


  // public Command LowerDowncommand() {
  //     return Commands.startEnd(()-> LowerElevator(), ()-> StopMotor(), this);

  // }

  // public void RaiseEleavtor() {
  //   if (Elevator_encoder.getPosition() <= -365) {
  //     StopMotor();
  //   } else {
  //     Elev_Motor.set(-1);
  //   }
    
  // }

  // public Command RaiseUpcommand() {
  //   return Commands.startEnd(()-> RaiseEleavtor(), ()-> StopMotor(), this);
  // }



  public void StopMotor() {
    Elev_Motor.stopMotor();
  }
  

  public Command resetElevatorCommand() {
    return Commands.run(()->Elevator_encoder.setPosition(0), this);
  }
  
  public BooleanSupplier getStowPositionSupplier() {
    return () -> StowPostiontSensor.get();
  }

  public Command ReefSetpointPositionCommand(double SetPoint) {
    return Commands.startEnd(()-> Elev_Motor_controller.setReference(SetPoint, SparkBase.ControlType.kMAXMotionPositionControl),()->StopMotor(),this).until(()-> false);
  }
  
}

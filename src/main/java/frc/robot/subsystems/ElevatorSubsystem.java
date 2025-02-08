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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Constants.ElevatorConstants.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
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

  //Mechism2d
  private final Mechanism2d elevatorMechanism2d;
  private final MechanismRoot2d elevMechanism1Root2d;
  private final MechanismLigament2d elevatorMechanismLigament2dStage1;
  private final MechanismRoot2d elevMechanism2Root2d;
  private final MechanismLigament2d elevatorMechanismLigament2dStage2;
  private final MechanismRoot2d elevMechanism3Root2d;
  private final MechanismLigament2d elevatorMechanismLigament2dStage3; 

  private double stage2DisplayY = 0.1; 
  private double stage3DisplayY = 0.1; 

  public void simulationPeriodic() {
    double elevatorHeight = Elevator_encoder.getPosition() * MaxHeight; 
    elevatorHeight = Math.max(0, Math.min(elevatorHeight, MaxHeight));
    
    elevMechanism1Root2d.setPosition(0.1, 0.1); 
    

    if (Elevator_Pos == MaxHeight) {
      
      double stage2TargetY = 0.8; 
      double stage3TargetY = 1.5; 
      double delta = 0.04; 

      
      if (0. < stage2TargetY) {
        stage2DisplayY = Math.min(stage2DisplayY + delta, stage2TargetY);
      } else if (stage2DisplayY > stage2TargetY) {
        stage2DisplayY = Math.max(stage2DisplayY - delta, stage2TargetY);
      }
      

      if (stage3DisplayY < stage3TargetY) {
        stage3DisplayY = Math.min(stage3DisplayY + delta, stage3TargetY);
      } else if (stage3DisplayY > stage3TargetY) {
        stage3DisplayY = Math.max(stage3DisplayY - delta, stage3TargetY);
      }
      

      elevMechanism2Root2d.setPosition(0.2, stage2DisplayY);
      elevMechanism3Root2d.setPosition(0.3, stage3DisplayY);
    } else if (Elevator_Pos == StowPostion) {
      double stage2TargetY = 0.1; 
      double stage3TargetY = 0.1; 
      double delta = 0.04; 
      if (stage2DisplayY < stage2TargetY) {
        stage2DisplayY = Math.min(stage2DisplayY + delta, stage2TargetY);
      } else if (stage2DisplayY > stage2TargetY) {
        stage2DisplayY = Math.max(stage2DisplayY - delta, stage2TargetY);
      }
      if (stage3DisplayY < stage3TargetY) {
        stage3DisplayY = Math.min(stage3DisplayY + delta, stage3TargetY);
      } else if (stage3DisplayY > stage3TargetY) {
        stage3DisplayY = Math.max(stage3DisplayY - delta, stage3TargetY);
      }
      elevMechanism2Root2d.setPosition(0.2, stage2DisplayY);
      elevMechanism3Root2d.setPosition(0.3, stage3DisplayY);
    } else if (Elevator_Pos == L2Height) {
      double stage2TargetY = 0.6; 
      double stage3TargetY = 1.2; 
      double delta = 0.04; 
      if (stage2DisplayY < stage2TargetY) {
        stage2DisplayY = Math.min(stage2DisplayY + delta, stage2TargetY);
      } else if (stage2DisplayY > stage2TargetY) {
        stage2DisplayY = Math.max(stage2DisplayY - delta, stage2TargetY);
      }
      if (stage3DisplayY < stage3TargetY) {
        stage3DisplayY = Math.min(stage3DisplayY + delta, stage3TargetY);
      } else if (stage3DisplayY > stage3TargetY) {
        stage3DisplayY = Math.max(stage3DisplayY - delta, stage3TargetY);
      }
      elevMechanism2Root2d.setPosition(0.2, stage2DisplayY);
      elevMechanism3Root2d.setPosition(0.3, stage3DisplayY);
    } else if (Elevator_Pos == L3Height) {
      double stage2TargetY = 0.6; 
      double stage3TargetY = 1.2; 
      double delta = 0.04; 
      if (stage2DisplayY < stage2TargetY) {
        stage2DisplayY = Math.min(stage2DisplayY + delta, stage2TargetY);
      } else if (stage2DisplayY > stage2TargetY) {
        stage2DisplayY = Math.max(stage2DisplayY - delta, stage2TargetY);
      }
      

      if (stage3DisplayY < stage3TargetY) {
        stage3DisplayY = Math.min(stage3DisplayY + delta, stage3TargetY);
      } else if (stage3DisplayY > stage3TargetY) {
        stage3DisplayY = Math.max(stage3DisplayY - delta, stage3TargetY);
      }
      

      elevMechanism2Root2d.setPosition(0.2, stage2DisplayY);
      elevMechanism3Root2d.setPosition(0.3, stage3DisplayY);
    }
    
  }
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
      .p(2)
      .i(1)
      .d(1);

    Elev_Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Elev_Motor_controller = Elev_Motor.getClosedLoopController();
    Elevator_encoder = Elev_Motor.getEncoder();
    
  

// Set MAXMotion parameters
    config.closedLoop.maxMotion
      .maxVelocity(5)
      .maxAcceleration(3)
      .allowedClosedLoopError(0.1);

    Elev_Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Mechism2d

    elevatorMechanism2d = new Mechanism2d(0.5, 3);
    elevMechanism1Root2d = elevatorMechanism2d.getRoot("Stage 1 Mounting Point", 0.1, 0.1);
    elevatorMechanismLigament2dStage1 = elevMechanism1Root2d.append(new MechanismLigament2d("Stage 1", Stage1Height, 90, ExtrustionThickness, new Color8Bit(Color.kDarkGoldenrod)));
    elevMechanism2Root2d = elevatorMechanism2d.getRoot("Stage 2 Holding Point", 0.2, 0.1);
    elevatorMechanismLigament2dStage2 = elevMechanism2Root2d.append(new MechanismLigament2d("Stage 2", Stage2Height, 90, ExtrustionThickness, new Color8Bit(Color.kDarkCyan)));
    elevMechanism3Root2d = elevatorMechanism2d.getRoot("Stage 3 Holding Point", 0.3, 0.1);
    elevatorMechanismLigament2dStage3 = elevMechanism3Root2d.append(new MechanismLigament2d("Stage 3", Stage3Height, 90, ExtrustionThickness, new Color8Bit(Color.kDarkGray)));

    //PID TUNING DO YOU KNOW I HATE PIDS THEY SUCK I DONT GET WHY THE EVEN EXIST LIKE WHY ARE THEY NAMED PID LIKE WHO NAMED THEM PROPORTINAL INTERGAL AND DERIVITATE AND IF THAT WASNT ENOUGH THEY ADDED THREE MORE FOR THE FUN OF IT LIKE ARE YOU SERIOUS HOW STUID ARE YOU HOW BOUT YOU TEACH YOURSELF SOMETHING WHERE NO ONE REALY UNDERSTANDS HOW TO EXPLAIN IT BUT THEY UNDERSTAND IT THEMSELF HUH???
    //i didnt do it by the way. nor will i and ill just say i did it and blame it on the elevator build :D
    
  }


  @Override
  public void periodic() {
    Logger.recordOutput("Elevator Height", Elevator_encoder.getPosition());
    Logger.recordOutput("Elevator Stowed", StowPostionSwitch());
    Logger.recordOutput("Elevator Engaged", ElevatorEngaged());
    SmartDashboard.putData("Elevator Mech2d", elevatorMechanism2d);
    SmartDashboard.putNumber("Elevator Encoder Height", Elevator_encoder.getPosition());
    SmartDashboard.putNumber("Elevator Height", Elevator_Pos);
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

  public void StopMotor() {
    Elev_Motor.stopMotor();
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

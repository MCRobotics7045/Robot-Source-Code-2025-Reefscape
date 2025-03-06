 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.SensorIOConstants.CoralEnterSensorID;
import static frc.robot.Constants.Constants.SensorIOConstants.CoralExitSensorID;
import static frc.robot.Constants.Constants.SensorIOConstants.Pigeon2Iid;
import static frc.robot.Constants.Constants.SensorIOConstants.StowPostiontSensorID;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class SensorsIO extends SubsystemBase {
  
  public Pigeon2 PigeonIMU;
  public DigitalInput ElevatorStowPostiontSensor;
  public DigitalInput CoralRampEnterSensor;
  public DigitalInput CoralEndEffectorEnterSensor;

  public SensorsIO() {
    PigeonIMU = new Pigeon2(Pigeon2Iid);
    ElevatorStowPostiontSensor = new DigitalInput(StowPostiontSensorID);
    CoralEndEffectorEnterSensor = new DigitalInput(CoralExitSensorID);
    CoralRampEnterSensor = new DigitalInput(CoralEnterSensorID);

  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("Elevator Stowed", ElevatorStowPostiontSensor.get());
      SmartDashboard.putBoolean("Coral Ramp", CoralRampEnterSensor.get());
      SmartDashboard.putBoolean("Coral End Effector", CoralEndEffectorEnterSensor.get());

  }
   public BooleanSupplier getStowPositionSupplier() {
      return () -> ElevatorStowPostiontSensor.get();
   }

   public BooleanSupplier CoralRampEnterSensorTriggered() {
    return () -> !CoralRampEnterSensor.get();
   }

   public BooleanSupplier CoralEndEffectorEnterSensorTriggered() {
    return () -> !CoralEndEffectorEnterSensor.get();
   }

   public Command ZeroPigeonIMU() {
    return Commands.runOnce(()-> PigeonIMU.setYaw(0));
   }
}

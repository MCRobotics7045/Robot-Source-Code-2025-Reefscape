 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import static frc.robot.Constants.Constants.SensorIOConstants.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class SensorsIO extends SubsystemBase {
  
  public static Pigeon2 PigeonIMU;
  public DigitalInput ElevatorStowPostiontSensor;
  public DigitalInput CoralRampEnterSensor;
  public DigitalInput CoralEndEffectorEnterSensor;
  public static AnalogInput frontLeftUltrasonic;
  public static AnalogInput frontRightUltrasonic;
  public static AnalogInput rearLeftUltrasonic;
  public static AnalogInput rearRightUltrasonic;


  public SensorsIO() {
    PigeonIMU = new Pigeon2(Pigeon2Iid);
    ElevatorStowPostiontSensor = new DigitalInput(StowPostiontSensorID);
    CoralEndEffectorEnterSensor = new DigitalInput(CoralEnterSensorID);
    CoralRampEnterSensor = new DigitalInput(CoralExitSensorID);
    frontLeftUltrasonic = new AnalogInput(FrontLeft);
    frontRightUltrasonic = new AnalogInput(FrontRight);
    rearLeftUltrasonic = new AnalogInput(RearLeft);
    rearRightUltrasonic = new AnalogInput(RearRight);

  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("Elevator Stowed", ElevatorStowPostiontSensor.get());
      SmartDashboard.putBoolean("Coral Ramp", CoralRampEnterSensor.get());
      SmartDashboard.putBoolean("Coral End Effector", CoralEndEffectorEnterSensor.get());
      SmartDashboard.putNumber("Front Right Ultra", ReadSensorinCM(frontRightUltrasonic));
      SmartDashboard.putNumber("Front Left Ultra", ReadSensorinCM(frontLeftUltrasonic));
      SmartDashboard.putNumber("Rear Left Ultra", ReadSensorinCM(rearLeftUltrasonic));
      SmartDashboard.putNumber("Rear Right Ultra", ReadSensorinCM(rearRightUltrasonic));
      SmartDashboard.putNumber("Pigeon 2 Yaw", PigeonIMU.getAngle());
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


   public static double ReadSensorinCM(AnalogInput Sensor) {
    double Voltage = Sensor.getVoltage();
    return Voltage * 100; //CM
   }

   public static boolean InRangeOfSensor(AnalogInput Sensor, double Distance) {
    double CMREAD = ReadSensorinCM(Sensor);
    return CMREAD < Distance;
   }

   public static String getObstaclePosition() {
    boolean frontLeft = ReadSensorinCM(frontLeftUltrasonic) < Threashold;
    boolean frontRight = ReadSensorinCM(frontRightUltrasonic) < Threashold;
    boolean backLeft = ReadSensorinCM(rearLeftUltrasonic) < Threashold;
    boolean backRight = ReadSensorinCM(rearRightUltrasonic) < Threashold;

    if (frontLeft && frontRight) return "Front";
    if (backLeft && backRight) return "Back";
    if (frontLeft) return "Left Front";
    if (frontRight) return "Right Front";
    if (backLeft) return "Left Back";
    if (backRight) return "Right Back";
    
    return "None"; 
}
}

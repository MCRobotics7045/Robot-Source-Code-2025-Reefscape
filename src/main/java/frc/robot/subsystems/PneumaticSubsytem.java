// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsytem extends SubsystemBase {
  /** Creates a new PneumaticSubsytem. */
  DoubleSolenoid climberSolenoid;
  public boolean startDown = true;
  public boolean PostionClimber = false; 
  // DoubleSolenoid climberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2); 
  // private final Compressor climber_Compressor; 
  public PneumaticSubsytem() {
    climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 3);
    // climber_Compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    if (startDown) {
      climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    
  }
  public Command Extend() {
    return Commands.run(()-> climberSolenoid.set(DoubleSolenoid.Value.kForward), this);
  } 
  public Command Retract() {
    return Commands.run(()-> climberSolenoid.set(DoubleSolenoid.Value.kReverse), this);
  }

}

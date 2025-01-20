// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.Constants.Constants.PneumaticConstatns.*;

public class PneumaticSubsystem extends SubsystemBase {

  private DoubleSolenoid[] dSolenoids = new DoubleSolenoid[5];
  /** Creates a new Pneumatic. */
  public PneumaticSubsystem() {
    dSolenoids[1] =  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DSOL1_PORT_FWD , DSOL1_PORT_REV);
    dSolenoids[2] = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DSOL2_PORT_FWD, DSOL2_PORT_REV);
    dSolenoids[3] =  new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DSOL3_PORT_FWD , DSOL3_PORT_REV);
    dSolenoids[4] = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DSOL4_PORT_FWD, DSOL4_PORT_REV);
  }
  public void init() {
    for (int i = 1; i < dSolenoids.length; i++) {
      if (dSolenoids[i] != null) {
        dSolenoids[i].set(DoubleSolenoid.Value.kReverse);
      }
    }
    
  }
  //Extend the cylinder
  public void extend(double SelectedCylinder) {
    int cylinderIndex = (int) SelectedCylinder;
    if (cylinderIndex >= 1 && cylinderIndex < dSolenoids.length) { //PASS
      dSolenoids[cylinderIndex].set(DoubleSolenoid.Value.kForward);
    } else {
      System.out.println("Invalid cylinder selected when Extend Command Called :" + cylinderIndex);
    }
    
  }

 //retract the cylinder
  public void retract(double SelectedCylinder) {
    int cylinderIndex = (int) SelectedCylinder;
    if (cylinderIndex >= 1 && cylinderIndex < dSolenoids.length) { //PASS
      dSolenoids[cylinderIndex].set(DoubleSolenoid.Value.kReverse);
    } else {
      System.out.println("Invalid cylinder selected when Retract Command Called :" + cylinderIndex);
    }
  }
 


}
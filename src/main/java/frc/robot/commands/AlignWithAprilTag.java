package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.function.Supplier;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;




public class AlignWithAprilTag extends Command{
    private final SwerveSubsystem m_swerve;
    private final SwerveRequest.FieldCentric m_drive;

    private final PIDController turnController = new PIDController(0.5, .5, .0);
    int shottimeout = 0;
    final double MaxAngularRate = 0.4 * Math.PI;
    private final PhotonCamera m_camera;
    private final double yawTol = 2.0;
    private int CycleNeeded = 2;
    private int Cycle = 0;

    public AlignWithAprilTag(SwerveSubsystem swerve, SwerveRequest.FieldCentric drive,PhotonCamera picam) {
        m_swerve = swerve;
        m_drive = drive;
        m_camera = picam;



    }

    @Override
  public void initialize() {
    // Called when the command is initially scheduled.
    System.out.println("Called:AlignWithAprilTag");
    
  }

  @Override
  public void execute() {
    
      
    var result = m_camera.getLatestResult();

    if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        double rotationSpeed = turnController.calculate(target.getYaw(), 0);
        Supplier<SwerveRequest> requestSupplier =  () -> m_drive.withRotationalRate(rotationSpeed*MaxAngularRate);
        m_swerve.setControl(requestSupplier.get());
    }

          
      }
      

    // Called every time Command is scheduled

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
        System.out.println("AlignWithAprilTag stopped reason: interrupted");
    } else {
        System.out.println("AlignWithAprilTag stopped reason: Ended");
    }
    //Called when command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    //Called when Command is finished
    var result = m_camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      double yawError = target.getYaw();
      if (Math.abs(yawError) <= yawTol) {
        Cycle++;
      } else {
        Cycle = 0;
      }
      return Cycle >= CycleNeeded;
    }
    return false;
  }


    


    
}
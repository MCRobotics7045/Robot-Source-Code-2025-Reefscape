package frc.robot.subsystems.Swerve;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Constants;
import frc.robot.Constants.TunerConstants;
import static frc.robot.RobotContainer.VISION;
import static frc.robot.RobotContainer.SIMULATION_TELE;
import static frc.robot.Constants.Constants.SwerveConstants.*;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final LinearFilter speedSmoother = LinearFilter.movingAverage(5);
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();
    Field2d field = new Field2d();
    public Double SpeedMultipler = 1.0;
    private Optional<EstimatedRobotPose> estimated;
    double[] states = new double[8];
    
    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("GameFeild", field);
        getState().Pose = new Pose2d();
    }

   


   

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {

        final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(rotationalVelocity);

        this.setControl(driveRequest);
    }
    
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

     public Pose2d getPose() {
        return getState().Pose;
    }

    public void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> RobotContainer.IsRed(),
                this);

    }


    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    
    
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

   
   
    public void UpdatePose() {
        if (estimated.isPresent()){
            EstimatedRobotPose Given = estimated.get();
            addVisionMeasurement(Given.estimatedPose.toPose2d(),Given.timestampSeconds);
            
        }
    }

    @Override
    public void periodic() {

        //Assigns Pose to null if neded 
         if (getState().Pose != null) {
            field.setRobotPose(getState().Pose);
        } else {
            System.out.println("Warning Pose Not Detected");
        }

        for (int i = 0; i < 4; i++)
            states[i * 2] = getModule(i).getTargetState().angle.getRadians();

        Logger.recordOutput("Pose", getPose());
        Logger.recordOutput("Target States", states);
        // for (int i = 0; i < 4; i++) states[i * 2 + 1] = getModule(i).getCurrentState().speedMetersPerSecond;
        // for (int i = 0; i < 4; i++)
        //     states[i * 2] = getModule(i).getCurrentState().angle.getRadians();
        // Logger.recordOutput("Measured States", states);

        // // Logger.recordOutput("Rotation/Rotational", getPose().getRotation());
        // Logger.recordOutput("Speeds/Chassisspeeds", getCurrentRobotChassisSpeeds());
        
       
        // SmartDashboard.putString("pose", getPose().toString());
        estimated = VISION.EST_POSE_RETURN();
        UpdatePose();
        
       
         
         if (Robot.isSimulation()) {
            SIMULATION_TELE.visionSim.update(getPose());
            SIMULATION_TELE.visionSim.getDebugField();
        }
           
          
        
    }
}

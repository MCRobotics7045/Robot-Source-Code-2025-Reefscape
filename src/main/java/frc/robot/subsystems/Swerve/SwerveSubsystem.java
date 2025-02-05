package frc.robot.subsystems.Swerve;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.ApplyChassisSpeeds;
// import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.Robot;
import frc.robot.RobotContainer;
// import frc.robot.Constants.Constants;
// import frc.robot.Constants.TunerConstants;
import static frc.robot.RobotContainer.VISION;
// import static frc.robot.RobotContainer.SIMULATION_TELE;
// import static frc.robot.Constants.Constants.SwerveConstants.*;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class SwerveSubsystem extends LegacySwerveDrivetrain implements Subsystem {
    private static final LinearFilter speedSmoother = LinearFilter.movingAverage(5);
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    RobotConfig config;
    private final LegacySwerveRequest.ApplyChassisSpeeds m_pathApplyRobotSpeeds = new LegacySwerveRequest.ApplyChassisSpeeds();
    Field2d field = new Field2d();
    public Double SpeedMultipler = 1.0;
    private Optional<EstimatedRobotPose> estimated;
    double[] states = new double[8];
    
    public SwerveSubsystem(LegacySwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, LegacySwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveSubsystem(LegacySwerveDrivetrainConstants driveTrainConstants, LegacySwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        SmartDashboard.putData("GameFeild", field);
        getState().Pose = new Pose2d();
    }

   


   

    public Command applyRequest(Supplier<LegacySwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {

        final LegacySwerveRequest.FieldCentric driveRequest = new LegacySwerveRequest.FieldCentric()
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

    private void configurePathPlanner() {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception ex) {
            ex.printStackTrace();
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

            AutoBuilder.configure(
                this::getPose,   // Supplier of current robot pose
                this::seedFieldRelative,         // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds)),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> RobotContainer.IsRed(),
                this
                
        );
        
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
        estimated = VISION.getEstimatedGlobalPose(getPose());
        UpdatePose();
        
       
         
        //  if (Robot.isSimulation()) {
        //     SIMULATION_TELE.visionSim.update(getPose());
        //     SIMULATION_TELE.visionSim.getDebugField();
        // }
           
          
        
    }
}

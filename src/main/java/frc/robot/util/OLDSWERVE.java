// package frc.robot.util;
/*  WE ARE NOT TO USE THIS 
 * 
 * 
 *  YOU CAN PULL COMMANDS AND HOW TO WORK IT BUT DO NOT REFRENCE THIS 
 */




// package frc.robot.subsystems.Swerve;

// import static frc.robot.RobotContainer.SENSORS;

// import java.util.Optional;
// import java.util.function.Supplier;

// import org.photonvision.EstimatedRobotPose;

// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
// // import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.ApplyChassisSpeeds;
// // import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// // import org.photonvision.EstimatedRobotPose;

// import edu.wpi.first.math.filter.LinearFilter;
// import edu.wpi.first.math.geometry.Pose2d;
// // import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// // import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// // import frc.robot.Robot;
// import frc.robot.RobotContainer;
// // import frc.robot.Constants.Constants;
// // import frc.robot.Constants.TunerConstants;
// // import static frc.robot.RobotContainer.VISION;
// // import static frc.robot.RobotContainer.SIMULATION_TELE;
// // import static frc.robot.Constants.Constants.SwerveConstants.*;
// import org.littletonrobotics.junction.Logger;
// import frc.robot.subsystems.Swerve.SensorsIO;
// import frc.robot.subsystems.VisionSubsystem;
// /**
//  * Class that extends the Phoenix SwerveDrivetrain class and implements
//  * subsystem so it can be used in command-based projects easily.
//  */
// public class OLDSWERVE extends LegacySwerveDrivetrain implements Subsystem {
//     private double kP = 0.5; 
//     private double kI = 0.0;
//     private double kD = 0.0;
//     private double kV = 1.2;
//     private static final LinearFilter speedSmoother = LinearFilter.movingAverage(5);
//     private static final double kSimLoopPeriod = 0.005; // 5 ms
//     private Notifier m_simNotifier = null;
//     private double m_lastSimTime;
//     RobotConfig config;
//     private final LegacySwerveRequest.ApplyChassisSpeeds m_pathApplyRobotSpeeds = new LegacySwerveRequest.ApplyChassisSpeeds();
//     Field2d field = new Field2d();
//     public Double SpeedMultipler = 1.0;
//     private Optional<EstimatedRobotPose> estimated;
//     double[] states = new double[8];
//     public double Swerve_Speed;
//     public OLDSWERVE(LegacySwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, LegacySwerveModuleConstants... modules) {
//         super(driveTrainConstants, OdometryUpdateFrequency, modules);
//         configurePathPlanner();
//         if (Utils.isSimulation()) {
//             startSimThread();
//         }
//         SmartDashboard.putNumber("Drive kP", kP);
//         SmartDashboard.putNumber("Drive kI", kI);
//         SmartDashboard.putNumber("Drive kD", kD);
//         SmartDashboard.putNumber("Drive kV", kV);
//     }

//     public OLDSWERVE(LegacySwerveDrivetrainConstants driveTrainConstants, LegacySwerveModuleConstants... modules) {
//         super(driveTrainConstants, modules);
//         configurePathPlanner();
//         if (Utils.isSimulation()) {
//             startSimThread();
//         }
//         SmartDashboard.putData("GameFeild", field);
//         getState().Pose = new Pose2d();
//         SmartDashboard.putNumber("Drive kP", kP);
//         SmartDashboard.putNumber("Drive kI", kI);
//         SmartDashboard.putNumber("Drive kD", kD);
//         SmartDashboard.putNumber("Drive kV", kV);
//     }

   


//     public void updatePID() {
//         kP = SmartDashboard.getNumber("Drive kP", kP);
//         kI = SmartDashboard.getNumber("Drive kI", kI);
//         kD = SmartDashboard.getNumber("Drive kD", kD);
//         kV = SmartDashboard.getNumber("Drive kV", kV);
    
//         for (int i = 0; i < 4; i++) { 
//             getModule(i).getDriveMotor().getConfigurator().apply(new Slot0Configs()
//                 .withKP(kP)
//                 .withKI(kI)
//                 .withKD(kD)
//                 .withKV(kV)
//             );
//         }
//     }
    


//     public Command applyRequest(Supplier<LegacySwerveRequest> requestSupplier) {
//         return run(() -> this.setControl(requestSupplier.get()));
//     }

//     public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
        
//         final LegacySwerveRequest.FieldCentric driveRequest = new LegacySwerveRequest.FieldCentric()
//                 .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
//                 .withVelocityX(xVelocity)
//                 .withVelocityY(yVelocity)
//                 .withRotationalRate(rotationalVelocity);

//         this.setControl(driveRequest);
//     }
    
//     private void startSimThread() {
//         m_lastSimTime = Utils.getCurrentTimeSeconds();

//         /* Run simulation at a faster rate so PID gains behave more reasonably */
//         m_simNotifier = new Notifier(() -> {
//             final double currentTime = Utils.getCurrentTimeSeconds();
//             double deltaTime = currentTime - m_lastSimTime;
//             m_lastSimTime = currentTime;

//             /* use the measured time delta, get battery voltage from WPILib */
//             updateSimState(deltaTime, RobotController.getBatteryVoltage());
//         });
//         m_simNotifier.startPeriodic(kSimLoopPeriod);
//     }

//      public Pose2d getPose() {
//         return getState().Pose;
//     }

//     private void configurePathPlanner() {
//         try {
//             config = RobotConfig.fromGUISettings();
//         } catch (Exception ex) {
//             ex.printStackTrace();
//             DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
//         }

//             AutoBuilder.configure(
//                 this::getPose,   // Supplier of current robot pose
//                 this::seedFieldRelative,         // Consumer for seeding pose against auto
//                 this::getCurrentRobotChassisSpeeds, // Supplier of current robot speeds
//                 // Consumer of ChassisSpeeds and feedforwards to drive the robot
//                 (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds)),
//                 new PPHolonomicDriveController(
//                     new PIDConstants(10, 0, 0),
//                     new PIDConstants(7, 0, 0)
//                 ),
//                 config,
//                 () -> RobotContainer.IsRed(),
//                 this
                
//         );
        
//     }

//     public Command getAutoPath(String pathName) {
//         return new PathPlannerAuto(pathName);
//     }

    
    
//     public ChassisSpeeds getCurrentRobotChassisSpeeds() {
//         return m_kinematics.toChassisSpeeds(getState().ModuleStates);
//     }

   
   
//     // public void UpdatePose() {
//     //     Pose2d currentPose = getPose();
//     //     estimated = VISION.getEstimatedGlobalPose(getPose());
//     //     Pose2d pathPlannerPose = AutoBuilder.getCurrentPose(); 

//     //     if (estimated.isPresent()){
//     //         EstimatedRobotPose Given = estimated.get();
//     //         Pose2d visionPose = Given.estimatedPose.toPose2d();

//     //         Pose2d correctedPose = blendPoses(pathPlannerPose, visionPose, 0.8); 
//     //         seedFieldRelative(correctedPose);  

//     //         addVisionMeasurement(visionPose, Given.timestampSeconds);
//     //         field.setRobotPose(correctedPose);
//     //         Logger.recordOutput("Updated Pose (PathPlanner + Vision)", correctedPose);

//     //     } else {
//     //         seedFieldRelative(pathPlannerPose);
//     //         field.setRobotPose(pathPlannerPose);
//     //         Logger.recordOutput("Updated Pose (PathPlanner Only)", pathPlannerPose);
//     //     }
//     // }

//     private Pose2d blendPoses(Pose2d pathPose, Pose2d visionPose, double BLENDER) {
//         double x = (1 - BLENDER) * pathPose.getX() + BLENDER * visionPose.getX();
//         double y = (1 - BLENDER) * pathPose.getY() + BLENDER * visionPose.getY();
//         double rotation = (1 - BLENDER) * pathPose.getRotation().getRadians() + BLENDER * visionPose.getRotation().getRadians();
//         return new Pose2d(x, y, new edu.wpi.first.math.geometry.Rotation2d(rotation));
//     }

//     private void GraphMotorData() {
//         for (int i = 0; i < 4; i++) {

//             //Drive
//             double DriveMotorSpeed = getModule(i).getCurrentState().speedMetersPerSecond;
//             double DriveMotorVoltage = getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble();
//             double DriveMotorVelocity = getModule(i).getDriveMotor().getVelocity().getValueAsDouble();
//             double DriveMotorStatorCurrent = getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble();
//             double DriveMotorTorqueCurrent = getModule(i).getDriveMotor().getTorqueCurrent().getValueAsDouble();
//             double DriveMotorSetSpeed = getModule(i).getTargetState().speedMetersPerSecond;
//             double error = DriveMotorSetSpeed - DriveMotorSpeed;
//             Logger.recordOutput("Module:" + i + " DriveMotorSpeed", DriveMotorSpeed);
//             Logger.recordOutput("Module" + i + "DriveMotorSetSpeed", DriveMotorSetSpeed);
//             Logger.recordOutput("Module" + i + "Speed Error", error);
//             Logger.recordOutput("Module:" + i + " DriveMotorVoltage", DriveMotorVoltage);
//             Logger.recordOutput("Module:" + i + " DriveMotorVelocity", DriveMotorVelocity);
//             Logger.recordOutput("Module:" + i + " DriveMotorStatorCurrent", DriveMotorStatorCurrent);
//             Logger.recordOutput("Module:" + i + " DriveMotorTorqueCurrent", DriveMotorTorqueCurrent);

//             //Steer
//             double SteerMotorSpeed = getModule(i).getCurrentState().speedMetersPerSecond;
//             double SteerMotorTurnAngle = getModule(i).getSteerMotor().getPosition().getValueAsDouble();
//             double SteerMotorVoltage = getModule(i).getSteerMotor().getMotorVoltage().getValueAsDouble();
//             double SteerMotorVelocity = getModule(i).getSteerMotor().getVelocity().getValueAsDouble();
//             double SteerMotorStatorCurrent = getModule(i).getSteerMotor().getStatorCurrent().getValueAsDouble();
//             double SteerMotorTorqueCurrent = getModule(i).getSteerMotor().getTorqueCurrent().getValueAsDouble();
//             Logger.recordOutput("Module:" + i + " SteerMotorSpeed", SteerMotorSpeed);
//             Logger.recordOutput("Module:" + i + " SteerMotorVoltage", SteerMotorVoltage);
//             Logger.recordOutput("Module:" + i + " SteerMotorVelocity", SteerMotorVelocity);
//             Logger.recordOutput("Module:" + i + " SteerMotorStatorCurrent", SteerMotorStatorCurrent);
//             Logger.recordOutput("Module:" + i + " SteerMotorTorqueCurrent", SteerMotorTorqueCurrent);
//             Logger.recordOutput("Module:" + i + " SteerMotorPos", SteerMotorTurnAngle);
//         }

        
//     }

    
//     public boolean ObstcaleDetection(String IgnoredSide,int ShownTag) { // True is Obstcale False is No Good
//         double ObstcaleScore = 0;
//         String ObstaclePosition = SensorsIO.getObstaclePosition();
//         if (ObstaclePosition == "None" || ObstaclePosition == IgnoredSide) {
//             //Pass Here 
//             ObstcaleScore += 0.25;
//         } else {
//             ObstcaleScore -= 0.25;
//         }
//         if (VisionSubsystem.getAllSeenTags().contains(ShownTag)) {
//             //Pass
//             ObstcaleScore += 0.25;
//         } else {
//             ObstcaleScore -= 0.25;
//         }
//         @SuppressWarnings("removal")
//         double gyroRate = SensorsIO.PigeonIMU.getRate();
//         if (Math.abs(gyroRate) < 5.0) { 
//             ObstcaleScore += 0.25;
//         } else {
//             ObstcaleScore -= 0.25;
//         }
//         Pose2d currentPose = getPose();
//         boolean isMoving = currentPose.getTranslation().getNorm() > 0.1;
//         if (isMoving) {
//             ObstcaleScore += 0.25; 
//         } else {
//             ObstcaleScore -= 0.25; 
//         }
//         return ObstcaleScore >= 0;
//     }


//     @Override
//     public void periodic() {

//         //Assigns Pose to null if neded 
//          if (getState().Pose != null) {
//             field.setRobotPose(getState().Pose);
//         } else {
//             System.out.println("Warning Pose Not Detected");
//         }

//         for (int i = 0; i < 4; i++)
//             states[i * 2] = getModule(i).getTargetState().angle.getRadians();

//         // Logger.recordOutput("Bare Pose", getPose());
//         // Logger.recordOutput("Target States", states);
//         // for (int i = 0; i < 4; i++) states[i * 2 + 1] = getModule(i).getCurrentState().speedMetersPerSecond;
//         // for (int i = 0; i < 4; i++)
//         //     states[i * 2] = getModule(i).getCurrentState().angle.getRadians();
//         // Logger.recordOutput("Measured States", states);

//         // Logger.recordOutput("Rotation/Rotational", getPose().getRotation());
//         // // Logger.recordOutput("Speeds/Chassisspeeds", getCurrentRobotChassisSpeeds());
        
       
//         // SmartDashboard.putString("pose", getPose().toString());
        
//         // UpdatePose();
//         GraphMotorData();
//         // updatePID(); 
//         SmartDashboard.putData("GameFeild", field);
//         //  if (Robot.isSimulation()) {
//         //     SIMULATION_TELE.visionSim.update(getPose());
//         //     SIMULATION_TELE.visionSim.getDebugField();
//         // }
           
          
        
//     }
// }

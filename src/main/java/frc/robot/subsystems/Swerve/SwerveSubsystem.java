package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveSteerGains;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.Swerve.SensorsIO;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.RobotContainer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.k180deg;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.kZero;
    private boolean m_hasAppliedOperatorPerspective = false;
    private final SysIdSwerveTranslation m_translationCharacterization = new SysIdSwerveTranslation();
    private final SysIdSwerveSteerGains m_steerCharacterization = new SysIdSwerveSteerGains();
    private final SysIdSwerveRotation m_rotationCharacterization = new SysIdSwerveRotation();
    private final SysIdRoutine m_sysIdRoutineTranslation;
    private final SysIdRoutine m_sysIdRoutineSteer;
    private final SysIdRoutine m_sysIdRoutineRotation;
    private SysIdRoutine m_sysIdRoutineToApply;
    private Field2d field = new Field2d();


    public SwerveSubsystem(SwerveDrivetrainConstants dtConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(dtConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        m_sysIdRoutineTranslation = createTranslationSysIdRoutine();
        m_sysIdRoutineSteer = createSteerSysIdRoutine();
        m_sysIdRoutineRotation = createRotationSysIdRoutine();
        m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
        configurePathPlanner();
    }

    public SwerveSubsystem(SwerveDrivetrainConstants dtConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(dtConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        m_sysIdRoutineTranslation = createTranslationSysIdRoutine();
        m_sysIdRoutineSteer = createSteerSysIdRoutine();
        m_sysIdRoutineRotation = createRotationSysIdRoutine();
        m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
        configurePathPlanner();
    }

    public SwerveSubsystem(SwerveDrivetrainConstants dtConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStdDev, Matrix<N3, N1> visionStdDev, SwerveModuleConstants<?, ?, ?>... modules) {
        super(dtConstants, odometryUpdateFrequency, odometryStdDev, visionStdDev, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        m_sysIdRoutineTranslation = createTranslationSysIdRoutine();
        m_sysIdRoutineSteer = createSteerSysIdRoutine();
        m_sysIdRoutineRotation = createRotationSysIdRoutine();
        m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
        configurePathPlanner();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    public void drive(double xVelocity, double yVelocity, double rotationalVelocity) {
        SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationalVelocity);
        setControl(driveRequest);
    }

    public boolean ObstcaleDetection(String IgnoredSide,int ShownTag) {
        double ObstcaleScore = 0;
        String obstaclePosition = SensorsIO.getObstaclePosition();
        if (obstaclePosition.equals("None") || obstaclePosition.equals(IgnoredSide)) {
            ObstcaleScore += 0.25;
        } else {
            ObstcaleScore -= 0.25;
        }
        if (VisionSubsystem.getAllSeenTags().contains(ShownTag)) {
            ObstcaleScore += 0.25;
        } else {
            ObstcaleScore -= 0.25;
        }
        double gyroRate = SensorsIO.PigeonIMU.getRate();
        if (Math.abs(gyroRate) < 5.0) {
            ObstcaleScore += 0.25;
        } else {
            ObstcaleScore -= 0.25;
        }
        Pose2d currentPose = getState().Pose;
        boolean isMoving = currentPose.getTranslation().getNorm() > 0.1;
        if (isMoving) {
            ObstcaleScore += 0.25;
        } else {
            ObstcaleScore -= 0.25;
        }
        return ObstcaleScore >= 0;
    }


    public Pose2d getPose() {
        return getState().Pose;
    }


    private Pose2d blendPoses(Pose2d pathPose, Pose2d visionPose, double BLENDER) {
        double x = (1 - BLENDER) * pathPose.getX() + BLENDER * visionPose.getX();
        double y = (1 - BLENDER) * pathPose.getY() + BLENDER * visionPose.getY();
        double rotation = (1 - BLENDER) * pathPose.getRotation().getRadians() + BLENDER * visionPose.getRotation().getRadians();
        return new Pose2d(x, y, new Rotation2d(rotation));
    }

    private void GraphMotorData() {
        for (int i = 0; i < 4; i++) {
            double DriveMotorSpeed = getModule(i).getCurrentState().speedMetersPerSecond;
            double DriveMotorVoltage = getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble();
            double DriveMotorVelocity = getModule(i).getDriveMotor().getVelocity().getValueAsDouble();
            double DriveMotorSetSpeed = getModule(i).getTargetState().speedMetersPerSecond;
            double error = DriveMotorSetSpeed - DriveMotorSpeed;
            Logger.recordOutput("Module:" + i + " DriveMotorSpeed", DriveMotorSpeed);
            Logger.recordOutput("Module" + i + "DriveMotorSetSpeed", DriveMotorSetSpeed);
            Logger.recordOutput("Module" + i + "Speed Error", error);
            Logger.recordOutput("Module:" + i + " DriveMotorVoltage", DriveMotorVoltage);
            Logger.recordOutput("Module:" + i + " DriveMotorVelocity", DriveMotorVelocity);

            double SteerMotorTurnAngle = getModule(i).getSteerMotor().getPosition().getValueAsDouble();
            double SteerMotorVoltage = getModule(i).getSteerMotor().getMotorVoltage().getValueAsDouble();
            double SteerMotorVelocity = getModule(i).getSteerMotor().getVelocity().getValueAsDouble();
            Logger.recordOutput("Module:" + i + " SteerMotorPos", SteerMotorTurnAngle);
            Logger.recordOutput("Module:" + i + " SteerMotorVoltage", SteerMotorVoltage);
            Logger.recordOutput("Module:" + i + " SteerMotorVelocity", SteerMotorVelocity);
        }
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }


    private void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> RobotContainer.IsRed(),
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }


        Pose2d currentPose = getState().Pose;
        if (currentPose != null) {
            field.setRobotPose(currentPose);
        }

        GraphMotorData();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    private SysIdRoutine createTranslationSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> setControl(m_translationCharacterization.withVolts(output)),
                null,
                this
            )
        );
    }

    private SysIdRoutine createSteerSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> setControl(m_steerCharacterization.withVolts(volts)),
                null,
                this
            )
        );
    }

    private SysIdRoutine createRotationSysIdRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(Math.PI / 6).per(Second),
                Volts.of(Math.PI),
                null,
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> {
                    setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                },
                null,
                this
            )
        );
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}

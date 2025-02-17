package frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
// import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants;
// import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants.SteerFeedbackType;
import static frc.robot.Constants.Constants.*;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

//DOCS
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html

public class TunerConstants {
  

    //---------------------------------------------------PID CONTROL NO TOUCHY---------------------------------------------------------------------------
    //Notes -.withKP(100): Proportional gain. and .withKI(0): Integral gain. and .withKD(0.2): Derivative gain.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    //--------------------------------------------------------------Motor Voltage Control------------------------------------------------------------------
    
    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    
    //-------------------------------------------------------------CONSTANTS--------------------------------------------------------------------------------

    
    //############################################# Motor Control

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true)
        );
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2 leave this null 
    private static final Pigeon2Configuration pigeonConfigs = null;


    //########################################## Gear Ratios and Wheel Specs

    // Theoretical free speed (m/s) at 12v applied output;
    public static final double kSpeedAt12VoltsMps = 9.46;
    
    // The stator current at which the wheels start to slip;
    private static final double kSlipCurrentA = 170;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    private static final double kCoupleRatio = 3.5714285714285716;
    
    //Preset dont touch unless Wheel Swap
    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 4;

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "rio";


    // These are only used for simulation
    private static final double kSteerInertia = 0.004 ;
    private static final double kDriveInertia = 0.025;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    //---------------------------------------------------------------------Module Setup------------------------------

    //This Defines the DriveTrain EX.. Setup for Pigeon and CanBus
    private static final LegacySwerveDrivetrainConstants DrivetrainConstants = new LegacySwerveDrivetrainConstants()
            .withCANbusName(kCANbusName)
            .withPigeon2Id(Pigeon2Iid)
            .withPigeon2Configs(pigeonConfigs);

    //Setus up all Constants To the DriveTrain 
    private static final LegacySwerveModuleConstantsFactory ConstantCreator = new LegacySwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withCANcoderInitialConfigs(cancoderInitialConfigs);


    // Front Left Moudle
    private static final int kFrontLeftDriveMotorId = 6;
    private static final int kFrontLeftSteerMotorId = 5;
    private static final int kFrontLeftEncoderId = 28;
    private static final double kFrontLeftEncoderOffset = -0.765380859375;
    private static final boolean kFrontLeftSteerInvert = true;

    private static final double kFrontLeftXPosInches = 11.25;
    private static final double kFrontLeftYPosInches = 11.25;

    // Front Right Moudle
    private static final int kFrontRightDriveMotorId = 8;
    private static final int kFrontRightSteerMotorId = 7;
    private static final int kFrontRightEncoderId = 29;
    private static final double kFrontRightEncoderOffset = -0.416259765625;
    private static final boolean kFrontRightSteerInvert = true;

    private static final double kFrontRightXPosInches = 11.25;
    private static final double kFrontRightYPosInches = -11.25;

    // Back Left Moudle
    private static final int kBackLeftDriveMotorId = 4;
    private static final int kBackLeftSteerMotorId = 3;
    private static final int kBackLeftEncoderId = 27;
    private static final double kBackLeftEncoderOffset = -0.493896484375;
    private static final boolean kBackLeftSteerInvert = true;

    private static final double kBackLeftXPosInches = -11.25;
    private static final double kBackLeftYPosInches = 11.25;

    // Back Right Moudle
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 1;
    private static final int kBackRightEncoderId = 26;
    private static final double kBackRightEncoderOffset = -0.497802734375;
    private static final boolean kBackRightSteerInvert = true;

    private static final double kBackRightXPosInches = -11.25;
    private static final double kBackRightYPosInches = -11.25;


    private static final LegacySwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide)
            .withSteerMotorInverted(kFrontLeftSteerInvert);
    private static final LegacySwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide)
            .withSteerMotorInverted(kFrontRightSteerInvert);
    private static final LegacySwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide)
            .withSteerMotorInverted(kBackLeftSteerInvert);
    private static final LegacySwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide)
            .withSteerMotorInverted(kBackRightSteerInvert);

    public static final SwerveSubsystem DriveTrain = new SwerveSubsystem(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}

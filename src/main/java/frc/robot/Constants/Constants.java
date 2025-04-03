/*----------------------------------------------------------------------------*/
/* Copyright (c) 2024 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Unit;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other purpose. All constants
 * should be declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //unasined random stuff 
    public static final double robotWidthInches = 27;
    public static final double robotLengthInches  = 27;
    public static final double WheelRadiusInches = 2;
    public static final double robotWidthMeters = Units.inchesToMeters(robotWidthInches);
    public static final double robotLengthMeters = Units.inchesToMeters(robotLengthInches);
    public static final double WheelRadiusMeters = Units.inchesToMeters(WheelRadiusInches);


    public static final boolean kDebug = true;

    //Can IDS
    
    // public final class ReefSidePoses {
    //     public static AprilTagFieldLayout APRILTAG_FieldLayout = null;
                
    //     public static AprilTagFieldLayout getTagLayout() {
    //        if (APRILTAG_FieldLayout == null) {
    //         APRILTAG_FieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    //        }
    //            return APRILTAG_FieldLayout;
    //        }
    // public static final Pose2d Reef_B_A = new Pose2d(
    //     APRILTAG_FieldLayout.getTagPose(18).get().toPose2d().getX() +,
    //     APRILTAG_FieldLayout.getTagPose(18).get().toPose2d().getY(),
    //     APRILTAG_FieldLayout.getTagPose(18).get().toPose2d().getRotation()
    //     );
    // public static final Pose2d Reef_B_B = new Pose2d(5.0, 3.4, Rotation2d.fromDegrees(180));
    // public static final Pose2d Reef_B_C = new Pose2d(5.0, 4.6, Rotation2d.fromDegrees(180));
    // public static final Pose2d Reef_B_D = new Pose2d(5.0, 5.8, Rotation2d.fromDegrees(180));
    // public static final Pose2d Reef_B_E = new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(180));
    // public static final Pose2d Reef_B_F = new Pose2d(5.0, 8.2, Rotation2d.fromDegrees(180));

    // public static final List<Pose2d> ALL_SIDES = List.of(
    //     REEF_SIDE_1, REEF_SIDE_2, REEF_SIDE_3,
    //     REEF_SIDE_4, REEF_SIDE_5, REEF_SIDE_6
    // );
    // }
    // XBOX Buttons
    
    public static class SensorIOConstants {
        public static final int Pigeon2Iid = 54;
        public static final int CoralEnterSensorID = 2;
        public static final int CoralExitSensorID = 9;
        public static final int MaxHeightSensorID = 3;
        public static final int StowPostiontSensorID = 7;
        public static final int LiDARSensorID = 6;

        public static final int FrontRight = 0;
        public static final int FrontLeft = 1;
        public static final int RearLeft = 2;
        public static final int RearRight = 3;
        public static final int LeftSide = 4;
        public static final int RightSide = 5;
        
        public static final double HoldingTime = 0.0;
        //UltraSonic Threashold 

        public static final double Threashold = 10;
    }

    public static class PneumaticConstatns {
        //Solenoid 1
        public static final int DSOL1_PORT_FWD = 1;
        public static final int DSOL1_PORT_REV = 2;
        //Solenoid 2
        public static final int DSOL2_PORT_FWD = 3;
        public static final int DSOL2_PORT_REV = 4;
        //Solenoid 3
        public static final int DSOL3_PORT_FWD = 5;
        public static final int DSOL3_PORT_REV = 6;
        //Solenoid 4
        public static final int DSOL4_PORT_FWD = 7;
        public static final int DSOL4_PORT_REV = 0;
    }
    public static class SwerveConstants {
        public static final double MaxSpeed = Units.feetToMeters(20.1); 
        public static final double MaxRotationSpeed = 4 * Math.PI;
        public static final double angularSpeed = MaxSpeed / (Math.hypot(robotLengthMeters, robotWidthMeters) / 2) / MaxRotationSpeed;
        public static double speedMultiplier = 1;
        public static final double SlewRate = 20;
    } 

    public static class LEDConstants {
        public static final int Strip1PWM = 1;
        
    }
    
    public static class InputConstants {

        public static final int xboxGreenButton = 1;
        public static final int xboxRedButton = 2;
        public static final int xboxBlueButton = 3;
        public static final int xboxYellowButton = 4;
        public static final int xboxLBButton = 5;
        public static final int xboxRBButton = 6;
        public static final int xboxStartButton = 7;
        public static final int xboxMenuButton = 8;
        public static final int xboxLStickButton = 9;
        public static final int xboxRStickButton = 10;

        public static final double xboxLeftStickDeadband = 0.15;  
        public static final double xboxRightStickDeadband = 0.15;  


        // USB Ports
        public static final int DRIVER_XBOX_CONTROLLER_PORT = 0;
        public static final int OPERATOR_XBOX_CONTROLLER_PORT = 1;


    }

    public static class AlgeeManipulatorConstants {
        public static final int Algee_Motor_ID = 38;
        public static final int CoralReefSetpoint = 0;
        public static final int StowPostionSetpoint = 0;
        public static final double GrabAlgaeFromReefSetPoint = -3.5;
        public static final double HoldFromReefSetpoint = -4.5;
        public static final int HoldFromGroundSetpoint = 0;

        //Remebr this is NOT the Eleavtor Height This is the angle of attack on the Algee Manipulator 
        public static final double L1Setpoint = 0;
        public static final double L2Setpoint = .8; 
        public static final double L3Setpoint = 8;
        public static final double L4Setpoint = 8;
        
    }
    

    public static class ElevatorConstants {
        public static final int Elevator_MotorID = 35; //32 
        
        //All encoder values
        public static final double L1SetpointC = -143; //Pov Down
        public static final double L2SetpointC = -42.2;  // Pov Left
        public static final double L3SetpointC = -79.1; //Pov Right 
        public static final double L4SetpointC = -133.5; //Old -130.5
       

        public static final double HoldBelowL2 = -15;
        public static final double L2SetpointA = -20; 
        public static final double HoldBelowL3 = -55;
        public static final double L3SetpointA = -60;

 
        //Time for Algee Arms to deploy for each Level
        public static final double TimeToClearforL1 = 0;
        public static final double TimeToClearforL2 = 0;
        public static final double TimeToClearforL3 = 0;
        public static final double TimeToClearforL4 = 0;


          
        public static final double GEARBOX_RATIO = 75.0;  
        public static final double SPOOL_DIAMETER_METERS = 0.05;

        //stupid not important things

        public static final double MAX_HEIGHT_METERS = 1.0;
        public static final double STOW_POSITION_METERS = 0.0;
        public static final double L2_HEIGHT_METERS = 0.3;
        public static final double L3_HEIGHT_METERS = 0.6;
        public static final double Stage1Height = Units.inchesToMeters(35);
        public static final double Stage2Height = Units.inchesToMeters(35);
        public static final double Stage3Height = Units.inchesToMeters(35);
        public static final double ExtrustionThickness = 10; // Not Correct at all but it was too small so you get 10 

    }

    public static class EndEffectorConstants {
        public static final int Top_MotorID = 33; //33 
        public static final int Bottom_MotorID = 31; 
        public static final int ServoID = 2;
        public static final double MotorReverseSpeed = 1;
        public static final double MotorFowardSpeed = -1;
        public static final int MotorIntakeSpeed = 100;

        
    }

    public static class Vision {

        public static AprilTagFieldLayout kTagLayout = null;
                
         public static AprilTagFieldLayout getTagLayout() {
            if (kTagLayout == null) {
                kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            }
                return kTagLayout;
            }
        
        public static boolean UseFR = true;
        public static boolean UseFL = false;
        public static boolean UseBR = false;
        public static boolean UseBL = false;
        
        
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
        public static final Distance SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(4.5);


        public static double maxDistance = 10;
        public static final double kMaxAngularSpeed = Units.rotationsToRadians(2);
        public static final double ErrorMarginPostive = 2.0;
        public static final double ErrorMarginNegative = -2.0;
        public static final double VISION_TURN_kP = 0.5;

         //AprilTag Id THESE WILL BE NAMED TO THERE SPOTS ON FEILD FOR 2025
        public static final int Apriltag1 = 1;
        public static final int Apriltag2 = 2;
        public static final int Apriltag3 = 3;
        public static final int Apriltag4 = 4;
        public static final int Apriltag5 = 5;
        public static final int Apriltag6 = 6;
        public static final int Apriltag7 = 7;
        public static final int Apriltag8 = 8;
        public static final int Apriltag9 = 9;
        public static final int Apriltag10 = 10;
        public static final int Apriltag11 = 11;
        public static final int Apriltag12 = 12;
        public static final int Apriltag13 = 13;
        public static final int Apriltag14 = 14;
        public static final int Apriltag15 = 15;
        public static final int Apriltag16 = 16;
        
        public static int BestFoundTag = 0;
    }

}

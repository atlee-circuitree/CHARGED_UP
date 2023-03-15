// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Limelight;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
 
public final class Constants {

    public static double side = 1;

    public DriverStation drive;

    public final class BlueAutoCoordsTag6 {
    public double[] blueStartingPosition_1 = {6.495 * side, 0.920, 0};
    public double[] moveToNorthConeWaypoint = {2.750, 1.041, 0};
    public double[] rotate180_1 = {2.750, 1.041, 180};
    public double[] PickUpConePosition_4 = {1.250, 1.041, 180};
    public double[] rotate180_2 = {1.250, 1.041, 0};
    public double[] moveToConeWayPoint = {2.750, 1.041, 0};
    public double[] moveToScoreWayPoint = {6.000, 1.041, 0};
    public double[] moveToScoringPosition_3 = {6.495, 1.428, 0};
    public double[] moveToScoreWayPoint_2 = {6.000, 1.041, 0};
    public double[] moveToNorthConeWayPoint = {2.750, 1.041, 0};
    public double[] rotate180_3 = {2.750, 1.041, 180};
    public double[] moveToFinalConeWayPoint = {2.000, -0.191, 180};
    public double[] moveToPickupCone_3 = {6.000, -0.191, 180, 0.2};
    }
  
    public final static class BlueAutoCoordsTag8{
    public static final double[] blueStartingPosition_3 = {0, -6.495, -2.500, 0, 0.2};
    public static final double[] moveToSouthConeWayPoint = {1, -2.750, -2.600, 0, 0.2};
    public static final double[] rotate180_1 = {3, -2.750, -2.600, 180, 0.2};
    public static final double[] PickUpConePosition_1 = {4, -1.250, -2.600, 180, 0.2};
    public static final double[] rotate180_2 = {5, -1.250, -2.600, 0, 0.2};
    public static final double[] moveToConeWayPoint = {6, -2.750, -2.600, 0 , 0.2};
    public static final double[] moveToScoreWayPoint = {7, -6.000, -2.600, 0, 0.2};
    public static final double[] moveToSCoringPosition_1 = {8, -6.495, -2.500, 0, 0.2};
    //public static final double[]moveToScoreWayPoint_2 = 
    }

    public final static class BlueAutoCoordsTag7{
    public static final double[] blueStartingPosition_2 = {0, -6.495, -0.8, 0, 0.2};
    public static final double[] moveToScoringWayPoint = {1, -6.000, -0.8, 0, 0.2};
    public static final double[] moveToPlatformWayPoint_1 = {2, -4.400, -0.8, 0, 0.2};
    public static final double[] moveToMiddleConeWayPoint = {3, -2.750, -0.8, 0, 0.2};
    public static final double[] moveToPlatformWayPoint_2 = {4, -4.400, -0.8, 0, 0.2};
    }

    public final static class RedAutoCoordsTag2{
    public static final double[] redStartingPosition_2 = {0, 6.495, -0.8, 0, 0.2};
    public static final double[] moveToScoringWayPoint = {1, 6.000, -0.8, 0, 0.2};
    // public static final double[] moveToPlatformWayPoint_1 = {2, 4.400, -0.8, 0, 0.2};
    public static final double[] moveToPlatformWayPoint_1 = {2, 3.700, -0.8, 0, 0.2};
    public static final double[] moveToMiddleConeWayPoint = {3, 1.900, -0.8, 0, 0.2};
    public static final double[] moveToPlatformWayPoint_2 = {4, 3.700, -0.8, 0, 0.2};
    }

    public final static class RedAutoCoordsTag3 {
    public static final double[] redStartingPosition = {0, -6.495, 0.920, 0, 0.2};
    public static final double[] moveToNorthConeWaypoint = {1, -2.750, 1.041, 0, 0.2};
    public static final double[] rotate180_1 = {2, -2.750, 1.041, 180, 0.2};
    public static final double[] PickUpConePosition_4 = {3, -1.250, 1.041, 180, 0.2};
    public static final double[] rotate180_2 = {4, -1.250, 1.041, 0, 0.2};
    public static final double[] moveToConeWayPoint = {5, -2.750, 1.041, 0, 0.2};
    public static final double[] moveToScoreWayPoint = {6, -6.000, 1.041, 0, 0.2};
    public static final double[] moveToScoringPosition_3 = {7, -6.495, 1.428,0, 0.2};
    public static final double[] moveToScoreWayPoint_2 = {8, -6.000, 1.041, 0, 0.2};
    public static final double[] moveToNorthConeWayPoint = {9, -2.750, 1.041, 0, 0.2};
    public static final double[] rotate180_3 = {10, -2.750, 1.041, 180, 0.2};
    public static final double[] moveToFinalConeWayPoint = {11, -2.000, -0.191, 180, 0.2};
    public static final double[] moveToPickupCone_3 = {12, -6.000, -0.191, 180, 0.2};
    }

    //Declare coordinates in the form {u, x, y, angle, tolerance}
    public static final double[][] autoCoordinates = {{0, 4,2 ,0, 0.2}, {1, 4.5,2 ,-90, 0.2}, {2, 5,2 ,180, 0.05}};
    
    //public static final double[][] testCoords = {{0, -6.445, 0.92, 0, .2}, {1, -2.214, 1.041, 0, .1}};
    public static final double[][] testCoords = {{0, 4.445, 1.92, 0, .025}, {1, 4.445, 1.92, 0, .025}};

    //Blue Auto
    public static final double[][] blueAuto = {{0, 6.495, 0.92, 0, 0.2}, {1, 2.750, 1.041, 0, 0.2}};
    //public static final double[][] blueAutoTEST = {BlueAutoCoordsTag7.blueStartingPosition_2, BlueAutoCoordsTag7.moveToScoringWayPoint, BlueAutoCoordsTag7.moveToPlatformWayPoint_1, BlueAutoCoordsTag7.moveToMiddleConeWayPoint, BlueAutoCoordsTag7.moveToPlatformWayPoint_2};
 
    //Red Auto
    public static final double[][] redAuto = {{0, 6.495, 0.92, 0, 0.2}, {1, 6.495, 0.92, 0, 0.2}};
    public static final double[][] redAutoTEST = {RedAutoCoordsTag2.redStartingPosition_2, RedAutoCoordsTag2.moveToScoringWayPoint,RedAutoCoordsTag2.moveToPlatformWayPoint_1,RedAutoCoordsTag2.moveToMiddleConeWayPoint,RedAutoCoordsTag2.moveToPlatformWayPoint_2};
    public static final double[][] redBalance = {RedAutoCoordsTag2.moveToScoringWayPoint,RedAutoCoordsTag2.moveToPlatformWayPoint_1};
    public double[][] scoringWpToConeWp = {{0, RedAutoCoordsTag3.moveToScoreWayPoint[0] * side, RedAutoCoordsTag3.moveToScoreWayPoint[1], 0, 0.2},
    {1,RedAutoCoordsTag3.moveToNorthConeWayPoint[0] * side,RedAutoCoordsTag3.moveToNorthConeWayPoint[1], 0, 0.2}};
    

    //Create Mode Select
    public static SendableChooser<String> modeSelect;
    public static SendableChooser<String> autoSelect;

    //Offsets and Zeroing
    public static String angleZeroKey = "AngleZero";
    public static double angleZeroValue = 0;

    //Ports
    public static final int frontLeftDrvMotorPort = 4;
    public static final int frontRightDrvMotorPort = 0;
    public static final int rearLeftDrvMotorPort = 6;
    public static final int rearRightDrvMotorPort = 8;

    public static final int frontLeftRotMotorPort = 3;
    public static final int frontRightRotMotorPort = 1;
    public static final int rearLeftRotMotorPort = 5;
    public static final int rearRightRotMotorPort = 7;

    public static final int frontLeftRotEncoderPort = 11;
    public static final int frontRightRotEncoderPort = 9;
    public static final int rearLeftRotEncoderPort = 12;
    public static final int rearRightRotEncoderPort = 10;

    public static final int leftExtMotorPort = 2;
    public static final int rightExtMotorPort = 13;
    public static final int angMotorPort = 15;
    public static final int angleEncoderChannel = 15;

   // public static final int clawMotorPort = 25;
  //  public static final int rotateClawMotorPort = 21;
    public static final int leftFeederMotorPort = 25;
    public static final int rightFeederMotorPort = 21;
    public static final int rotationFeederMotorPort = 26;
    public static final int rotationEncoderChannel = 5;
    public enum GrabPosition {

        Init,
        Open,
        Cone,
        Cube

    }

    //Encoder Values
    public static final double frontLeftEncoderOffset = 284.58984375;
    public static final double frontRightEncoderOffset = 254.267578125;
    public static final double rearLeftEncoderOffset = 127.353515625;
    public static final double rearRightEncoderOffset = 72.509765625;

    public static final int angleEncoderDIO = 0;
    public static final int feederRotationEncoderDIO = 1;
    public static final int extensionEncoderDIO = 9;
 
    public static final double angleOffset = -11.5;
    public static final double maxAngleEncoderValue = 36.2;
    public static final double minAngleEncoderValue = -14.0;
    //public static final double maxAngleEncoderValue = 9999;
    //public static final double minAngleEncoderValue = -9999;

    //public static final double maxRotationEncoderValue = 0.886;
    //public static final double minRotationEncoderValue = 0.348;

    public static final double maxCounterClockwiseRotationEncoderValue = 0.742;
    public static final double middleRotationEncoderValue = .732; //when claw is horizontal
    public static final double maxClockwiseRotationEncoderValue = 0.225;
   
    public static final double maxGrabEncoderValue = .7;
    public static final double minGrabEncoderValue = .40;
 
    public static final double maxExtensionValue = 52;
    public static final double minExtensionValue = 0;

    public static final double limelightSingleTargetPoseLengthCutoff = 1.9;

    public static final double trackwidth = 22.5;
    public static final double wheelbase = 22.5;

    //Distance from center of robot to any module
    public static final double drivetrainRadius = Math.sqrt(Math.pow(trackwidth, 2) + Math.pow(wheelbase, 2)); 

    public static final int xboxControllerPort = 0;

    //Phoenix's Falcon FX Motor Sensor Units Per Rotation
    public final static int kSensorUnitsPerRotation = 4096;

    //Number of rotations to drive when performing Distance Closed Loop
    public final static double kRotationsToTravel = 6;

    //Drive PIDs
    public static final double rotPID_P = 1;
    public static final double rotPID_I = 0;
    public static final double rotPID_D = 0.00;
    public static final double rotPIDMinValue = 0.07;

    //Auto PIDs
    public static final double xControllerP = 8;
    public static final double xControllerI = 2;
    public static final double xControllerD = 0;

    public static final double yControllerP = 8;
    public static final double yControllerI = 2;
    public static final double yControllerD = 0;

    public static final double zControllerProportion = 10;

    public static final double aBalanceXConstant = .2;
    public static final double aBalanceTurnConstant = .2;


    //Instansiated in this order:
    //FrontLeft, FrontRight, RearLeft, RearRight
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(-trackwidth / 2, wheelbase / 2),
            new Translation2d(trackwidth / 2, wheelbase / 2),
            new Translation2d(-trackwidth / 2, -wheelbase / 2),
            new Translation2d(trackwidth / 2, -wheelbase / 2));


    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);

    

    public static double smoothUsingError(double Encoder, double Range, double SetPoint, double MinSpeed, double MaxSpeed) {

        double error = (Math.abs(Encoder - SetPoint) / Range);

        if (error < MinSpeed) {

            error = MinSpeed;

        }
        
        if (error > MaxSpeed) {

            error = MaxSpeed;

        }

        return error;

    }
 
}
 
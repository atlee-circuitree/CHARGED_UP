// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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

    //Declare coordinates in the form {u, x, y, angle}
    public static final double[][] autoCoordinates = {{0, 0,0 ,0}, {1, 0,0.5 ,20}, {2, 0,1 ,40}, {3, 0,1.5 ,60}};

    //Create Mode Select
    public static SendableChooser<String> modeSelect;

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

    public static final int clawMotorPort = 25;
    public static final int rotateClawMotorPort = 21;
    public static final int rotationEncoderChannel = 5;
    public enum GrabPosition {

        Init,
        Open,
        Cone,
        Cube

    }

    //Encoder Values
    public static final double frontLeftEncoderOffset = 286.259765625;
    public static final double frontRightEncoderOffset = 250.224609375;
    public static final double rearLeftEncoderOffset = 128.759765625;
    //public static final double rearLeftEncoderOffset = 89.38476;
    public static final double rearRightEncoderOffset = 76.201171875;
 
    public static final int angleEncoderDIO = 0;
    public static final double angleOffset = -8;
    public static final double maxAngleEncoderValue = 33;
    public static final double minAngleEncoderValue = -19.6;
    //public static final double maxAngleEncoderValue = 9999;
    //public static final double minAngleEncoderValue = -9999;

    public static final int clawRotationEncoderDIO = 1;
    //public static final double maxRotationEncoderValue = 0.886;
    //public static final double minRotationEncoderValue = 0.348;

    public static final double maxCounterClockwiseRotationEncoderValue = 0.742;
    public static final double middleRotationEncoderValue = .732; //when claw is horizontal
    public static final double maxClockwiseRotationEncoderValue = 0.225;

    public static final int clawGrabEncoderDIO = 2;
    //public static final double maxGrabEncoderValue = 80;
    //public static final double minGrabEncoderValue = -47;
    public static final double maxGrabEncoderValue = .7;
    public static final double minGrabEncoderValue = .45;

    public static final int extensionEncoderDIO = 9;
 
    public static final double maxExtensionInchValue = 44;
    public static final double minExtensionInchValue = 4;

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
    public static final double xControllerP = 7;
    public static final double xControllerI = 2;
    public static final double xControllerD = 0;

    public static final double yControllerP = 7;
    public static final double yControllerI = 2;
    public static final double yControllerD = 0;

    public static final double zControllerP = 0.5;
    public static final double zControllerI = 0;
    public static final double zControllerD = 0;

    public static final double aBalanceValue = .25;


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
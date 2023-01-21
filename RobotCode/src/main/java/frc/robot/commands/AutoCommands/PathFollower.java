

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PathEQ;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;

import java.lang.Math;

public class PathFollower extends CommandBase {

  private final Drivetrain drivetrain;
  
  private PIDController xPID = new PIDController(Constants.xControllerP, Constants.xControllerI, Constants.xControllerD);
  private PIDController yPID = new PIDController(Constants.yControllerP, Constants.yControllerI, Constants.yControllerD);
  private PIDController zPID = new PIDController(Constants.zControllerP, Constants.zControllerI, Constants.zControllerD);

  private ProfiledPIDController placeholderPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(1, 1));

  private HolonomicDriveController driveController = new HolonomicDriveController(xPID, yPID, placeholderPID);
  
  private PathEQ pathEQ;

  private double targetUValue = 0;
  private double uIncrement;

  private double speedMod;
  private double pTolerance;
  private double aTolerance;

  private double forward;
  private double strafe;
  private double rotation;

  private boolean isFinished = false;
 
  public PathFollower(Drivetrain dt, PathEQ pathEquation, double speed, double pointTolerance, double angleTolerance) {
    
    drivetrain = dt;
    pathEQ = pathEquation;
    speedMod = speed;
    pTolerance = pointTolerance;
    aTolerance = angleTolerance;

    zPID.enableContinuousInput(-180, 180);
    driveController.setTolerance(new Pose2d(pointTolerance, pointTolerance, new Rotation2d(angleTolerance)));

    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {

    targetUValue = 0;
    uIncrement = 0.1;

    isFinished = false;

  }

  @Override
  public void execute() {

    //Auto calculations
    
  
    //Rotation values are just placeholders bc we don't want the HolonomicDriveController to know that we are rotating
    //We deal with the rotation separately since it is waaaay easier
    //However, because of this, if you want to know the current robot Z, DO NOT USE CURRENTPOS IT WILL JUST RETURN ZERO
    //Use drivertain.getOdometryZ() instead
    Pose2d currentPos = new Pose2d(drivetrain.getOdometryX(), drivetrain.getOdometryY(),
    new Rotation2d(0));

    Pose2d targetPos = new Pose2d(pathEQ.solvePoint(targetUValue)[0], pathEQ.solvePoint(targetUValue)[1], 
      new Rotation2d(0));


    //zPID.setSetpoint(targetPos.getRotation().getDegrees());
    zPID.setSetpoint(pathEQ.solveAngle(targetUValue));

    ChassisSpeeds chassisSpeeds = driveController.calculate(currentPos, targetPos, 1, new Rotation2d(0));

    forward = chassisSpeeds.vyMetersPerSecond;
    strafe = chassisSpeeds.vxMetersPerSecond;
    rotation = (pathEQ.solveAngle(targetUValue) - drivetrain.getOdometryZ())/10;

    //Normalize calculated vx and vy velocities
    if(Math.abs(forward) > 1 && Math.abs(forward) >= Math.abs(strafe)){
      forward = forward/Math.abs(forward);
      strafe = strafe/Math.abs(forward);
    }
    else if(Math.abs(strafe) > 1 && Math.abs(strafe) > Math.abs(forward)){
      forward = forward/Math.abs(strafe);
      strafe = strafe/Math.abs(strafe);
    }

    //Invert fwd so it goes the correct direction
    forward = -forward;
    //Invert rot so it rotates in the correct direction
    //rotation = -rotation;

    //Clamp calculated vz velocity between -1 and 1
    if(rotation > 1){
      rotation = 1;
    }
    else if(rotation < -1){
      rotation = -1;
    }

    

    //Modify target values for field orientation (temp used to save calculations before original forward and strafe values are modified)
    //double temp = forward * Math.cos(-drivetrain.getNavXOutputRadians()) + strafe * Math.sin(-drivetrain.getNavXOutputRadians()); 
    //strafe = -forward * Math.sin(-drivetrain.getNavXOutputRadians()) + strafe * Math.cos(-drivetrain.getNavXOutputRadians()); 
    //forward = temp;

    //Alphabet vars
    double A = strafe - (rotation * (Constants.wheelbase/Constants.drivetrainRadius));
    double B = strafe + (rotation * (Constants.wheelbase/Constants.drivetrainRadius));
    double C = forward - (rotation * (Constants.trackwidth/Constants.drivetrainRadius));
    double D = forward + (rotation * (Constants.trackwidth/Constants.drivetrainRadius));

    double frontLeftAngle = Math.atan2(B, C)*(180/Math.PI);
    double frontRightAngle = Math.atan2(B, D)*(180/Math.PI);
    double rearLeftAngle = Math.atan2(A, C)*(180/Math.PI);
    double rearRightAngle = Math.atan2(A, D)*(180/Math.PI);

    double frontLeftSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
    double frontRightSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
    double rearLeftSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
    double rearRightSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));

    //Normalizes speeds (makes sure that none are > 1)
    double max = frontLeftSpeed;
    if(max < frontRightSpeed){
      max = frontRightSpeed;
    }
    if(max < rearLeftSpeed){
      max = rearLeftSpeed;
    } 
    if(max < rearRightSpeed){
      max = rearRightSpeed;
    }
    if(max > 1){
      frontLeftSpeed = frontLeftSpeed / max;
      frontRightSpeed = frontRightSpeed / max;
      rearLeftSpeed = rearLeftSpeed / max;
      rearRightSpeed = rearRightSpeed / max;
    }



 
    //Set angles for modules (change speed mod later if needed)
    drivetrain.rotateModule(SwerveModule.FRONT_LEFT, frontLeftAngle, 1);
    drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, frontRightAngle, 1);
    drivetrain.rotateModule(SwerveModule.REAR_LEFT, rearLeftAngle, 1);
    drivetrain.rotateModule(SwerveModule.REAR_RIGHT, rearRightAngle, 1);

    //Set speeds for modules
    drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, frontLeftSpeed * speedMod);
    drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, frontRightSpeed * speedMod);
    drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, rearLeftSpeed * speedMod);
    drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, rearRightSpeed * speedMod);
  

    //If robot at target position, increment the target u value
    if(driveController.atReference()){
      if(pathEQ.solveAngle(targetUValue) - aTolerance <= drivetrain.getOdometryZ() && 
        drivetrain.getOdometryZ() <= pathEQ.solveAngle(targetUValue) + aTolerance){
        
        targetUValue = targetUValue + uIncrement;

      }
    }
    //If the target u value is greater than the final u value, the robot has finished moving
    if(targetUValue > pathEQ.getFinalUValue()){
      isFinished = true;
    }


    SmartDashboard.putNumber("frontLeftSpeed", frontLeftSpeed);
    SmartDashboard.putNumber("frontRightSpeed", frontRightSpeed);
    SmartDashboard.putNumber("rearLeftSpeed", rearLeftSpeed);
    SmartDashboard.putNumber("rearRightSpeed", rearRightSpeed);

    SmartDashboard.putNumber("frontLeftAngle", frontLeftAngle);
    SmartDashboard.putNumber("frontRightAngle", frontRightAngle);
    SmartDashboard.putNumber("rearLeftAngle", rearLeftAngle);
    SmartDashboard.putNumber("rearRightAngle", rearRightAngle);

    SmartDashboard.putNumber("ChassisSpeeds X Velocity", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds Y Velocity", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeeds Z Velocity", chassisSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Forward", forward);
    SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Rotation", rotation);

    SmartDashboard.putNumber("Current X", currentPos.getX());
    SmartDashboard.putNumber("Current Y", currentPos.getY());
    SmartDashboard.putNumber("Fake Current Z", currentPos.getRotation().getDegrees());
    SmartDashboard.putNumber("Actual Current Z", drivetrain.getOdometryZ());

    SmartDashboard.putNumber("Target X", targetPos.getX());
    SmartDashboard.putNumber("Target Y", targetPos.getY());
    SmartDashboard.putNumber("Target Z", pathEQ.solveAngle(targetUValue));
    
    SmartDashboard.putNumber("Target U", targetUValue);
    SmartDashboard.putBoolean("Drivecontroller Within Tolerance", driveController.atReference());
    SmartDashboard.putBoolean("IsFinished", isFinished);

  }  

  @Override
  public void end(boolean interrupted){


    drivetrain.rotateMotor(Motors.FRONT_LEFT_DRV, 0);
    drivetrain.rotateMotor(Motors.FRONT_RIGHT_DRV, 0);
    drivetrain.rotateMotor(Motors.REAR_LEFT_DRV, 0);
    drivetrain.rotateMotor(Motors.REAR_RIGHT_DRV, 0);

    drivetrain.rotateModule(SwerveModule.FRONT_LEFT, Math.atan2(0, 0)*(180/Math.PI), 0);
    drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, Math.atan2(0, 0)*(180/Math.PI), 0);
    drivetrain.rotateModule(SwerveModule.REAR_LEFT, Math.atan2(0, 0)*(180/Math.PI), 0);
    drivetrain.rotateModule(SwerveModule.REAR_RIGHT, Math.atan2(0, 0)*(180/Math.PI), 0);

  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}


package frc.robot.commands.MiscCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PathEQ;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;

import java.lang.Math;

public class TestPathFollower extends CommandBase {

  private final Drivetrain drivetrain;
  
  private PathEQ pathEQ;
  private double slope;
  private double rise;
  private double run;

  private double targetUValue = 0;
  private double[] targetPoint = new double[2];
  private double uIncrement = 0.5;

  private double forward = 0;
  private double strafe = 0;
  private double rotation = 0;

  private double speedMod = 1;
  private double tolerance = 0.1;

  private boolean isFinished = false;

  private double artificialY;
 
  public TestPathFollower(Drivetrain dt, PathEQ pathEquation, double speed, double pointTolerance) {
    
    drivetrain = dt;
    pathEQ = pathEquation;
    speedMod = speed;
    tolerance = pointTolerance;

    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {
    artificialY = 0;
  }

  @Override
  public void execute() {

    
    //Auto calculations

    double[] currentPos = {0, artificialY};
    targetPoint = pathEQ.solvePoint(targetUValue);
    
    slope = pathEQ.slope(currentPos, targetPoint);
    rise = pathEQ.slopeRiseRun(currentPos, targetPoint)[1];
    run = pathEQ.slopeRiseRun(currentPos, targetPoint)[0];

  
    //Moving Forward
    if(targetPoint[1] > currentPos[1]){
      
      forward = Math.abs(rise);

      if(slope >= 0){
        strafe = Math.abs(run);
      }
      else{
        strafe = Math.abs(run) * -1;
      }
    }

    //Moving Backward
    else if(targetPoint[1] < currentPos[1]){
      
      forward = Math.abs(rise) * -1;

      if(slope >= 0){
        strafe = Math.abs(run) * -1;
      }
      else{
        strafe = Math.abs(run);
      }
    }

    //Normalize speeds and catch exception slopes
    if(slope == Double.POSITIVE_INFINITY || slope == Double.NEGATIVE_INFINITY){
      
      if(targetPoint[1] > currentPos[1]){
        forward = 1;
        strafe = 0;
      }
      else{
        forward = -1;
        strafe = 0;
      }
    
    }
    else{

      if(Math.abs(forward) > Math.abs(strafe)){
        forward = 1 * (Math.abs(forward)/forward);
        strafe = strafe/Math.abs(forward);
      }
      else{
        strafe = 1 * (Math.abs(strafe)/strafe);
        forward = forward/Math.abs(strafe);
      }

    }

    forward = -forward;

    double[] fsrArray = {forward, strafe, rotation};
    SmartDashboard.putNumberArray("FWD, STR, ROT", fsrArray);



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SmartDashboard.putString("Breakpoint 1", "tripped");

    SmartDashboard.putNumberArray("TargetPoint", targetPoint);

    

    SmartDashboard.putNumber("speedMod", speedMod);

    SmartDashboard.putNumberArray("current Pos", currentPos);

    SmartDashboard.putBoolean("Is Finished", isFinished);

    SmartDashboard.putNumber("Target U Value", targetUValue);

    SmartDashboard.putNumber("getFinalUValue", pathEQ.getFinalUValue());

    SmartDashboard.putNumberArray("xCoefs", pathEQ.getLastRowofXCoefs());

    SmartDashboard.putNumber("Slope", slope);

    SmartDashboard.putNumber("Artifical Y", artificialY);

    //If we have reached the current target X value
    if(targetPoint[0] - tolerance < currentPos[0] && currentPos[0] < targetPoint[0] + tolerance){
      //And the current target Y value
      if(targetPoint[1] - tolerance < currentPos[1] && currentPos[1] < targetPoint[1] + tolerance){
        //Set the next target u value and point
        targetUValue = targetUValue + uIncrement;
      }
    }


    artificialY = artificialY + 0.01;

    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      SmartDashboard.putString("Exc", "TRIGGERED");
    }
    
    //If the target u value is greater than the final u value, the robot has finished moving
    if(targetUValue > pathEQ.getFinalUValue()){
      isFinished = true;
    }
  }  

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
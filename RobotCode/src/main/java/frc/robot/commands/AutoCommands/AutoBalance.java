// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */

  private Drivetrain drivetrain = new Drivetrain();
  private XboxController xbox;
  private double pitchTolerance;
  private double yawTolerance;
  
  private double horizontalSpeed;
  private double turnSpeed;

  private double state = 0;

  private String movementDirection = "Level";


  public AutoBalance(Drivetrain dt, XboxController xboxController, double YawTolerance, double PitchTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
    pitchTolerance = PitchTolerance;
    yawTolerance = YawTolerance;
    xbox = xboxController;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //state = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Telemetry
    SmartDashboard.putNumber("Robot Straight Speed", horizontalSpeed);
    SmartDashboard.putNumber("Robot Turn Speed", turnSpeed);
    SmartDashboard.putString("Is robot level", movementDirection);

    //Speed PID calculations
    horizontalSpeed = Math.abs(drivetrain.getNavXPitchOutput()) * Constants.aBalanceXConstant;
    turnSpeed = Math.abs(drivetrain.getNavXYawOutput()) * Constants.aBalanceTurnConstant;

     //Speed clamps
    if (horizontalSpeed < -1) {
      horizontalSpeed = -1;
    } else if (horizontalSpeed > 1) {
      horizontalSpeed = 1;
    }

    if (turnSpeed < -1) {
      turnSpeed = -1;
    } else if (turnSpeed > 1) {
      turnSpeed = 1;
    }

    //AutoBalance Movement
    //Rotates robot until heading is perpendicular with ramp
   /*if (state == 0) {
       
    //When NavX thinks yaw is less than 2, turns right.
    if (drivetrain.getNavXYawOutput() < -yawTolerance) {
     //Sets wheel angle to turn position
     drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 45, 1);
     drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 315, 1);
     drivetrain.rotateModule(SwerveModule.REAR_LEFT, 135, 1);
     drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 225, 1);
        
     drivetrain.driveAllModules(turnSpeed);
     movementDirection = "Turning Right";

     //When NavX thinks yaw is greater than 2, turns left.
    } else if (drivetrain.getNavXYawOutput() > yawTolerance) {

     //Sets wheel angle to turn position
     drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 45, 1);
     drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 315, 1);
     drivetrain.rotateModule(SwerveModule.REAR_LEFT, 135, 1);
     drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 225, 1);
        
     drivetrain.driveAllModules(-turnSpeed);
     movementDirection = "Turning Left";

    } else {

     drivetrain.driveAllModules(0);
     state = 1;
        
    }
  }*/

   //if (state == 1) {
     //Sets wheel angle to forward position
     drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 0, 1);
     drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 0, 1);
     drivetrain.rotateModule(SwerveModule.REAR_LEFT, 0, 1);
     drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 0, 1);

     //When NavX thinks tilted back, drive motors forward
     if (drivetrain.getNavXPitchOutput() > pitchTolerance) {
        
       drivetrain.driveAllModules(horizontalSpeed);
       movementDirection = "Moving Forwards";
       
     //When NavX thinks tilted forward, drive motors backwards
     } else if (drivetrain.getNavXPitchOutput() < -pitchTolerance) {

       drivetrain.driveAllModules(-horizontalSpeed);
       movementDirection = "Moving Backwards";
  
     } else {

       drivetrain.driveAllModules(0);
       movementDirection = "Level";    
       //state = 0;
    
      }
    }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xbox.getXButton() == false) {
     
      return true;

    } else if (drivetrain.getNavXPitchOutput() < pitchTolerance && drivetrain.getNavXPitchOutput() > -pitchTolerance) {

      return true;
      
    } else {

      return false;

    }
  }
}

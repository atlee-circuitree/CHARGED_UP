// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class DriveBackwardsToDistance extends CommandBase {
  
  Drivetrain drivetrain;
  Limelight limelight;
  double targetDistance;
  double speed;
  double stage = 1;
  Timer rotateTimer;
  Pose2d startingPose;
  Rotation2d startingRotation;

  public DriveBackwardsToDistance(Drivetrain dt, Limelight lm, double TargetDistance, double Speed) {
    
    drivetrain = dt;
    limelight = lm;
    targetDistance = TargetDistance;
    speed = Speed;

    addRequirements(drivetrain, limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //startingRotation = new Rotation2d(0);
    //startingPose = new Pose2d(6.3, -1.1, startingRotation);
    rotateTimer = new Timer();
    drivetrain.resetOdometryToLimelight();
    //drivetrain.resetOdometry(startingPose);
    rotateTimer.reset();
    rotateTimer.start();
    stage = 1;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println("In Loop");
 
    if (stage == 1) {

      System.out.println("Stage 1");

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 0, 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 0, 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, 0, 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 0, 1);

      if (rotateTimer.get() >= 1) {

        stage = 2;

      }
 
    }

    if (stage == 2) {

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 0, 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 0, 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, 0, 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 0, 1);

      drivetrain.driveAllModules(speed);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

      stage = 1;
      drivetrain.driveAllModules(0);
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (drivetrain.getOdometryX() <= targetDistance) {

      return true;

    } else {

      return false;

    }

  }
}

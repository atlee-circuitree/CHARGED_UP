// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SlideCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slide;

public class GoToAngleAndExtension extends CommandBase {
  
  Slide slide;
  double targetAngle;
  double targetExtension;
  double tolerance;
  boolean doubleTap;
  double stage;
  double endStage;
 
  public GoToAngleAndExtension(Slide sl, double TargetAngle, double TargetExtension, double Tolerance, boolean DoubleTap) {
 
    slide = sl;
    targetAngle = TargetAngle;
    targetExtension = TargetExtension;
    tolerance = Tolerance;
    doubleTap = DoubleTap;

    addRequirements(slide);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (doubleTap == true) {

      if (slide.getExtensionEncoderInches() > 2) {

        // Retract arm
        stage = 1;
        endStage = 2;

      } else if (slide.WithinTolerence(slide.getAngle(), targetAngle, tolerance) == false) {

        // Go to angle
        stage = 2;
        endStage = 3;

      } else {

        // Go to extension
        stage = 3;
        endStage = 4;

      }

    } else {

      stage = 1;
      endStage = 4;

    }

  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    SmartDashboard.putNumber("Current Stage", stage);
    SmartDashboard.putNumber("Target Stage", endStage);
    System.out.println(slide.WithinTolerence(slide.getAngle(), targetAngle, tolerance));

    // Retract Back
    if (stage == 1) {

      System.out.println("Stage 1 Running");
      if (slide.getExtensionEncoderInches() < 0 - tolerance) {

        slide.extendArmUsingPower(1);
  
      } else if (slide.getExtensionEncoderInches() > 0 + tolerance) {
  
        slide.extendArmUsingPower(-1);
  
      } else {
  
        System.out.println("Stage 2 Finished");
        slide.extendArmUsingPower(0);
        stage = 2;
  
      }

    // Go to angle
    } else if (stage == 2) {

      System.out.println("Stage 2 Running");
      if (slide.getAngle() < targetAngle - tolerance) {
        
        slide.changeAngleUsingPower(1);
   
      } else if (slide.getAngle() > targetAngle + tolerance) {
  
        slide.changeAngleUsingPower(-1);
  
      } else {
 
        System.out.println("Stage 2 Finished");
        slide.changeAngleUsingPower(0);
        stage = 3;
  
      }

    } else if (stage == 3) {

      System.out.println("Stage 3 Running");
      if (slide.getExtensionEncoderInches() < targetExtension - tolerance) {

        slide.extendArmUsingPower(1);
  
      } else if (slide.getExtensionEncoderInches() > targetExtension + tolerance) {
  
        slide.extendArmUsingPower(-1);
  
      } else {
  
        System.out.println("Stage 3 Finished");
        slide.extendArmUsingPower(0);
        stage = 4;
  
      }

    } else if (stage == 4) {

      slide.changeAngleUsingPower(0);
      slide.extendArmUsingPower(0);
      end(false);

    }
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    slide.changeAngleUsingPower(0);
    slide.extendArmUsingPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    if (stage >= endStage) {

      return true;

    } else {

      return false;

    }

  }

}



// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SlideCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slide;

public class GoToAngleAndExtension extends CommandBase {
  
  Slide slide;
  double targetAngle;
  double targetExtension;
  double tolerance;
  double stage = 1;
 
  public GoToAngleAndExtension(Slide sl, double TargetAngle, double TargetExtension, double Tolerance) {
 
    slide = sl;
    targetAngle = TargetAngle;
    targetExtension = TargetExtension;
    tolerance = Tolerance;

    addRequirements(slide);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    System.out.print("runnign");

    // Retract Back
    if (stage == 1) {

      if (slide.getExtensionEncoderInches() < 0 - tolerance) {

        slide.extendArmUsingPower(.65);
  
      } else if (slide.getExtensionEncoderInches() > 0 + tolerance) {
  
        slide.extendArmUsingPower(-1);
  
      } else {
  
        slide.extendArmUsingPower(0);
        stage = 2;
  
      }

    // Go to angle
    } else if (stage == 2) {

      if (slide.getAngle() < targetAngle - tolerance) {
        
        slide.changeAngleUsingPower(1);
   
      } else if (slide.getAngle() > targetAngle + tolerance) {
  
        slide.changeAngleUsingPower(-1);
  
      } else {
 
        slide.changeAngleUsingPower(0);
        stage = 3;
  
      }

    } else if (stage == 3) {

      if (slide.getExtensionEncoderInches() < targetExtension - tolerance) {

        slide.extendArmUsingPower(.65);
  
      } else if (slide.getExtensionEncoderInches() > targetExtension + tolerance) {
  
        slide.extendArmUsingPower(-1);
  
      } else {
  
        slide.extendArmUsingPower(0);
        end(true);
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
 
    if (stage == 4) {

      return true;

    } else {

      return false;

    }

  }

}



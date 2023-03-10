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
  double endWhenFinished = 0;
 
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
 
    if (slide.WithinTolerence(slide.getAngle(), targetAngle, tolerance) == true && slide.getExtension() > 8) {
 
      System.out.print("Extended, retracting");
      endWhenFinished = 1;
      targetAngle = slide.getAngle();
      targetExtension = 0;

    } else if (slide.WithinTolerence(slide.getAngle(), targetAngle, tolerance) == false && slide.getExtension() < 2) {
 
      System.out.print("Retracting, moving to angle");
      endWhenFinished = 2;
      // targetAngle stays the same
      targetExtension = slide.getExtensionEncoderInches();

    } else if (slide.WithinTolerence(slide.getAngle(), targetAngle, tolerance) == true) {

      System.out.print("At angle, extending");
      endWhenFinished = 1;
      targetAngle = slide.getAngle();
      // targetExtension stays the same

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    if (slide.getExtensionEncoderInches() < targetExtension - tolerance) {

      slide.extendArmUsingPower(.65);

    } else if (slide.getExtensionEncoderInches() > targetExtension + tolerance) {

      slide.extendArmUsingPower(-1);

    } else {

      if (endWhenFinished == 1) {

        end(true);

      }
      slide.extendArmUsingPower(0);

    }
 
    if (slide.getAngle() < targetAngle - tolerance) {
        
      slide.changeAngleUsingPower(1);
 
    } else if (slide.getAngle() > targetAngle + tolerance) {

      slide.changeAngleUsingPower(-1);

    } else {

      if (endWhenFinished == 2) {

        end(true);

      }
      slide.changeAngleUsingPower(0);

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
 
    return false;

  }

}



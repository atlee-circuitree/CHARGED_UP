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
  boolean isRunningExtension;

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

    if (slide.WithinTolerence(slide.getAngle(), targetAngle, tolerance) == false) {

      isRunningExtension = false;

    } else {

      isRunningExtension = true;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (isRunningExtension = true) {

      if (slide.getExtensionEncoderInches() < targetExtension - tolerance) {

        slide.extendArmUsingPower(.65);

      } else if (slide.getExtensionEncoderInches() > targetExtension + tolerance) {

        slide.extendArmUsingPower(-1);

      } else {

        slide.extendArmUsingPower(0);

      }

    }

    if (isRunningExtension = false) {

      if (slide.getAngle() < targetAngle - .5) {
        
        slide.changeAngleUsingPower(1);
 
      } else if (slide.getAngle() > targetAngle + .5) {

        slide.changeAngleUsingPower(-1);

      } else {

        slide.changeAngleUsingPower(0);

      }

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

    if (isRunningExtension = false) {

      if (slide.WithinTolerence(slide.getAngle(), targetAngle, tolerance)) {

        return true;

      } else {

        return false;

      }

    } else {

      if (slide.WithinTolerence(slide.getExtension(), targetExtension, tolerance)) {

        return true;

      } else {

        return false;

      }

    }

  }



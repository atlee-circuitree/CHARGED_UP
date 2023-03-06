// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Slide;

public class AngleAndExtendInAuto extends CommandBase {
 
  Slide slide;
  Feeder feeder;

  double targetAngle;
  double targetExtension;

  public AngleAndExtendInAuto(Slide sl, Feeder cl, double TargetAngle, double TargetExtension) {
 
    slide = sl;
    feeder = cl;

    targetAngle = TargetAngle;
    targetExtension = TargetExtension;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (slide.getAngle() < targetAngle - .5) {

      if (slide.getAngle() > 30) {

        slide.changeAngleUsingPower(-1);

      } else {

        slide.changeAngleUsingPower(-.6);

      }

    } else if (slide.getAngle() > targetAngle + .5) {

      slide.changeAngleUsingPower(1);

    } else {

      slide.changeAngleUsingPower(0);

    }

    if (slide.getExtension() < targetExtension - .25) {

      slide.extendArmUsingPower(.65);

    } else if (slide.getExtension() > targetExtension + .25) {

      slide.extendArmUsingPower(-.65);

    } else {

      slide.extendArmUsingPower(0);

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

    if (Math.abs(slide.getAngle() - targetAngle) <= 1 && Math.abs(slide.getExtension() - targetAngle) <= .5) {

      return true;

    } else {

      return false;

    }

  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class RotateClawToAngle extends CommandBase {
 
  Claw claw;
  double speed;
  double angle;
  double targetAngle;

  public RotateClawToAngle(Claw cw, double spd) {
     
    claw = cw;
    speed = spd;
    addRequirements(claw);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //claw.rotateToAngle(angle, speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    claw.rotateClaw(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (claw.getRotation() < targetAngle + 1 && claw.getRotation() > targetAngle - 1) {

      return true;

    } else {

      return false;

    }

  }
}

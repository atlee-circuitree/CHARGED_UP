// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class GrabClawToAngle extends CommandBase {
 
  Claw claw;
  double speed;
  double targetAngle;

  public GrabClawToAngle(Claw cw, double spd, double ang) {
     
    claw = cw;
    speed = spd;
    targetAngle = ang;
    addRequirements(claw);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    claw.grabToAngle(targetAngle, speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    claw.runClaw(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (claw.getGrabRotation() < targetAngle + .5 && claw.getGrabRotation() > targetAngle - .5) {

      return true;

    } else {

      return false;

    }

  }
}

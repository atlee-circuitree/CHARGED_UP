// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class RunClawUntilClamp extends CommandBase {
 
  Claw claw;
  double speed;
  double currentPosition;
  double lastPosition;
  double velocity;
  Timer StartTimer;

  public RunClawUntilClamp(Claw cw, double sp) {
     
    claw = cw;
    speed = sp;

    addRequirements(claw);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    StartTimer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    lastPosition = currentPosition;

    currentPosition = claw.getGrabRotation();

    velocity = Math.abs(currentPosition - lastPosition);

    claw.runClaw(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    claw.runClaw(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (velocity > .05 || StartTimer.get() < .3) {

      return false;

    } else {

      return true;

    } 

  }
}

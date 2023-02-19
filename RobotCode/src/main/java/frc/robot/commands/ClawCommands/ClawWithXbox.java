// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;

public class ClawWithXbox extends CommandBase {
  
  Claw claw;
  XboxController xbox;

  XboxController xbox1;
  XboxController xbox2;
  
  public ClawWithXbox(Claw cl, XboxController xb1, XboxController xb2) {
    
    claw = cl;
    xbox1 = xb1;
    xbox2 = xb2;

    addRequirements(claw);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (Constants.modeSelect.getSelected() == "Player_Two") {

      xbox = xbox1;
      
    } else {

      xbox = xbox2;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    if (xbox.getLeftTriggerAxis() > .5 && xbox.getRightTriggerAxis() < .5) {

      claw.runClaw(-.3);

    } else if (xbox.getLeftTriggerAxis() < .5 && xbox.getRightTriggerAxis() > .5) {

      claw.runClaw(.3);

    } else {

      claw.runClaw(0);

    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    claw.runClaw(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

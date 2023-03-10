// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SlideCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Slide;

public class SlideWithXbox extends CommandBase {

  XboxController primaryXbox;
  XboxController secondaryXbox;

  XboxController xbox1;
  XboxController xbox2;

  Slide slide;
 
  double targetAngle;
  double targetExtension;
 
  public SlideWithXbox(XboxController xb1, XboxController xb2, Slide sl) {
  
    xbox1 = xb1;
    xbox2 = xb2;
    slide = sl;

    addRequirements(slide);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    if (Constants.modeSelect.getSelected() == "Player_Two") {

      primaryXbox = xbox2;
      secondaryXbox = xbox1;
      
    } else {

      primaryXbox = xbox1;
      secondaryXbox = xbox2;

    }

    targetAngle = slide.getAngle();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    slide.changeAngleUsingPower(-secondaryXbox.getLeftY());
 
    if (secondaryXbox.getStartButton()) {

      slide.extendArmUsingPowerNoLimit(-secondaryXbox.getRightY() / 1);
      slide.resetExtensionEncoder();
  
    } else {
  
      slide.extendArmUsingPower(-secondaryXbox.getRightY() / 1);
  
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

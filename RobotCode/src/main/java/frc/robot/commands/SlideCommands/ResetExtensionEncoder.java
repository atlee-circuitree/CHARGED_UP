// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SlideCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Slide;

public class ResetExtensionEncoder extends CommandBase {
 
  Slide slide;

  public ResetExtensionEncoder(Slide sl) {

    slide = sl;

    addRequirements(slide);
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    slide.setExtensionOffset();

  }
 
}

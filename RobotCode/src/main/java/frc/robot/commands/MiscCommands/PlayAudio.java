// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MiscCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Audio;

public class PlayAudio extends CommandBase {

  private final Audio audio;
  private int Selection;
  private int Loops;

  public PlayAudio(Audio ad, int selection, int loops) {

    audio = ad;

    Selection = selection;
    Loops = loops;

    addRequirements(audio);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    audio.playAudio(1, 2);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    audio.stopAudio();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

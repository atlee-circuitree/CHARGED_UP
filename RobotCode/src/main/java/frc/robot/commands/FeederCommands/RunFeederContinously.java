// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunFeederContinously extends InstantCommand {

  Feeder feeder;
  double speed;

  /** Creates a new Feeder. */
  public RunFeederContinously(Feeder fdr, double Speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    feeder = fdr;
    speed = Speed;
    addRequirements(feeder);

  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  feeder.runFeeder(speed);

  }
}

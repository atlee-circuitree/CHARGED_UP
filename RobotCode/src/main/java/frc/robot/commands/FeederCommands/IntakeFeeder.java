// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;

public class IntakeFeeder extends CommandBase {

  Feeder feeder;
  double speed;

  /** Creates a new Feeder. */
  public IntakeFeeder(Feeder fdr) {
    // Use addRequirements() here to declare subsystem dependencies.
    feeder = fdr;
    addRequirements(feeder);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (feeder.absoluteClawPosition() < Constants.CONE_POSITION - 0.1) {

      //speed = .25;
      speed = 0.4;
      //speed = .5;

    } else {

      speed = .5;
      //speed = 0.25;
      //speed = 0.4;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    feeder.runFeeder(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    feeder.runFeeder(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class GoToFeederPosition extends CommandBase {
 
  Feeder feeder;
  double speed;
  double tolerance = .04;
 
  public GoToFeederPosition(Feeder Feeder, double Speed) {

    feeder = Feeder;
    speed = Speed;
  
    addRequirements(feeder);
 
  }
 
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    feeder.rotateFeeder(speed);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    feeder.rotateFeeder(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (feeder.absoluteClawPosition() < .81 + tolerance && feeder.absoluteClawPosition() > .81 - tolerance && speed < 0) {

      System.out.println("Cone Position Met");
      return true;

    } else if (feeder.absoluteClawPosition() < .15 + tolerance && feeder.absoluteClawPosition() > .15 - tolerance && speed > 0) {

      System.out.println("Cube Position Met");
      return true;

    } else {
      
      return false;

    }

  }
}

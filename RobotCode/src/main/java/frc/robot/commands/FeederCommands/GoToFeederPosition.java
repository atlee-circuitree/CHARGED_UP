// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederPosition;
import frc.robot.subsystems.Feeder;

public class GoToFeederPosition extends CommandBase {
 
  Feeder feeder;
  double speed;
  double tolerance = .02;
  FeederPosition position;
 
  public GoToFeederPosition(Feeder Feeder, double Speed, FeederPosition Position) {

    position = Position;
    feeder = Feeder;
    speed = Speed;
  
    addRequirements(feeder);
 
  }
 
  @Override
  public void initialize() {
    
    if (position == FeederPosition.Cone && feeder.absoluteClawPosition() > Constants.CONE_POSITION) {
      speed = -speed;
    } else if (position == FeederPosition.Crush) {
      speed = -speed;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    feeder.rotateFeeder(speed);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if(speed < 0){
      speed = -speed;
    }

    feeder.rotateFeeder(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (feeder.withinAbsoluteTolerance(feeder.absoluteClawPosition(), Constants.CUBE_POSITION, tolerance) == true && speed > 0 && position == Constants.FeederPosition.Cube) {

      System.out.println("Cube Position Met");
      return true;

    } else if (feeder.withinAbsoluteTolerance(feeder.absoluteClawPosition(), Constants.CONE_POSITION, tolerance) == true && position == Constants.FeederPosition.Cone) {

      System.out.println("Cone Position Met");
      return true;

    } else if (feeder.withinAbsoluteTolerance(feeder.absoluteClawPosition(), Constants.CRUSH_POSITION, tolerance) == true && speed < 0 && position == Constants.FeederPosition.Crush) {

      System.out.println("Crush Position Met");
      return true;

    } else {
      
      return false;

    }
  }
}

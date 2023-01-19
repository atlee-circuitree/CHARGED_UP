// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */

  private Drivetrain drivetrain = new Drivetrain();

  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //When button is Pressed
    
    //Lock all wheels forward
    drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 0, 1);
    drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 0, 1);
    drivetrain.rotateModule(SwerveModule.REAR_LEFT, 0, 1);
    drivetrain.rotateModule(SwerveModule.REAR_LEFT, 0, 1);
    //When navx thinks unbalanced forward

    //drive motors back

    //When navx thinks unbalanced backwards

    //drive morots forward

    //repeat until let go of button



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

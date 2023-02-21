// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class CenterOnConeNode extends CommandBase {
  
  int stage;

  Drivetrain drivetrain;
  Limelight limelight;

  XboxController xbox;

  XboxController xbox1;
  XboxController xbox2;

  public CenterOnConeNode(Drivetrain dt, Limelight lm, XboxController xb1, XboxController xb2) {
    
    xbox1 = xb1;
    xbox2 = xb2;
    drivetrain = dt;
    limelight = lm;

    addRequirements(drivetrain, limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    stage = 1;

    if (Constants.modeSelect.getSelected() == "Player_Two") {

      xbox = xbox2;

    } else {

      xbox = xbox1;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (stage == 1) {

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 45, 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 135, 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, 315, 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 225, 1);

      if (limelight.HorizontalOffset() > 1) {

        drivetrain.driveAllModules(-.2);

      } else if (limelight.HorizontalOffset() < -1) {

        drivetrain.driveAllModules(.2);

      } else {

        drivetrain.driveAllModules(0);
        stage = 2;

      }

    }

    if (stage == 2) {

      drivetrain.rotateModule(SwerveModule.FRONT_LEFT, 0, 1);
      drivetrain.rotateModule(SwerveModule.FRONT_RIGHT, 0, 1);
      drivetrain.rotateModule(SwerveModule.REAR_LEFT, 0, 1);
      drivetrain.rotateModule(SwerveModule.REAR_RIGHT, 0, 1);

      if (limelight.TargetArea() > 20) {

        drivetrain.driveAllModules(-.2);

      } else if (limelight.HorizontalOffset() < 10) {

        drivetrain.driveAllModules(.2);

      } else {

        drivetrain.driveAllModules(0);
        stage = 3;

      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (stage == 3) {

      return true;

    } else {

      return false;

    }

  }
  
}

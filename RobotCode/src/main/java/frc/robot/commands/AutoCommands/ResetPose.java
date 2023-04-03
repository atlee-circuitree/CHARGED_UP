// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetPose extends CommandBase {
  /** Creates a new ResetPose. */
  Drivetrain drivetrain;
  Pose2d pose;
  Rotation2d rotation;
  double x;
  double y;
  double rot;

  public ResetPose(Drivetrain dt, double X, double Y, double Rot) {
    
    drivetrain = dt;
    x = -X;
    y = Y;
    rot = Rot;

    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drivetrain.zeroNavXYaw();
    rotation = new Rotation2d(rot);
    pose = new Pose2d(x, y, rotation);
    drivetrain.resetOdometry(pose);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    end(false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

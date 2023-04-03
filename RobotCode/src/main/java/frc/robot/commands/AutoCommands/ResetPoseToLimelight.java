// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import java.text.DecimalFormat;

public class ResetPoseToLimelight extends CommandBase {
  /** Creates a new ResetPose. */
  Drivetrain drivetrain;
  Limelight limelight;
  Pose2d pose;
  double limelightAngle;
  double rotation;
  double rot;

  DecimalFormat odometryRounder = new DecimalFormat("###");

  public ResetPoseToLimelight(Drivetrain dt, Limelight lt, double Rot) {
    
    drivetrain = dt;
    limelight = lt;
    rot = Math.toRadians(Rot);
    

    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drivetrain.zeroNavXYaw();
    limelightAngle = -(limelight.BotPose()[5]);
    rotation = Math.toRadians(Double.valueOf(odometryRounder.format(limelightAngle)));
    pose = new Pose2d(drivetrain.getOdometryX(), drivetrain.getOdometryY(), new Rotation2d(rotation));
    drivetrain.resetOdometry(pose);
    

    SmartDashboard.putNumber("getOdometryX()", drivetrain.getOdometryX());
    SmartDashboard.putNumber("getOdometryY()", drivetrain.getOdometryY());
    SmartDashboard.putNumber("getOdometryZ()", drivetrain.getOdometryZ());

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

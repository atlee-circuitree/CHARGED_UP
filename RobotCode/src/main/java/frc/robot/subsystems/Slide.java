// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slide extends SubsystemBase {

  TalonFX leftExtMotor;
  TalonFX rightExtMotor;
  TalonFX angMotor;

  PIDController anglePID;
  PIDController extensionPID;

  SimpleMotorFeedforward angleFeed;

  DutyCycleEncoder angEncoder;

  double angleBore;
  double angle;
  double tolerance = 100;
  double CurrentStage = 1;
 
 
  public Slide() {

    leftExtMotor = new TalonFX(Constants.leftExtMotorPort);
    rightExtMotor = new TalonFX(Constants.rightExtMotorPort);
    angMotor = new TalonFX(Constants.angMotorPort);

    anglePID = new PIDController(.01, 0, .01);
    extensionPID = new PIDController(.01, 0, .01);
    angleFeed = new SimpleMotorFeedforward(.01, .01);
 
    angEncoder = new DutyCycleEncoder(Constants.angleEncoderChannel);
  
    leftExtMotor.setInverted(true);
    rightExtMotor.setInverted(false);

    leftExtMotor.setNeutralMode(NeutralMode.Brake);
    rightExtMotor.setNeutralMode(NeutralMode.Brake);
    angMotor.setNeutralMode(NeutralMode.Brake);
 
  }

  @Override
  public void periodic() {
 
    SmartDashboard.putNumber("Angle Encoder Maximum", Constants.maxAngleEncoderValue);
    SmartDashboard.putNumber("Angle Encoder Minimum", Constants.minAngleEncoderValue);
    SmartDashboard.putNumber("Angle Encoder Reading", angEncoder.getAbsolutePosition());
 
    SmartDashboard.putNumber("Extension Stage", CurrentStage);
 
  }

  public void changeAngleUsingPower(double speed) {
 
    angMotor.set(ControlMode.PercentOutput, speed);
 
  }

  public void changeAngleUsingVoltage(double speed) {
 
    angMotor.set(ControlMode.Current, angleFeed.calculate(speed));
 
  }

  public void extendArmUsingPower(double speed) {

    leftExtMotor.set(ControlMode.PercentOutput, speed);
    rightExtMotor.set(ControlMode.PercentOutput, speed);

  }

  public void extendToStage(int Stage) {
    

  }

}

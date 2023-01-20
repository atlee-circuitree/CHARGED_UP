// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slide extends SubsystemBase {

  TalonFX leftAngMotor;
  TalonFX rightAngMotor;
  TalonFX leftExtMotor;
  TalonFX rightExtMotor;

  DutyCycleEncoder leftAngleBore;
  DutyCycleEncoder rightAngleBore;
 
  public Slide() {

    leftAngMotor = new TalonFX(Constants.leftAngMotorPort);
    rightAngMotor = new TalonFX(Constants.rightAngMotorPort);
    leftExtMotor = new TalonFX(Constants.leftExtMotorPort);
    rightExtMotor = new TalonFX(Constants.rightExtMotorPort);

    //leftAngleBore = new DutyCycleEncoder(0);
    //rightAngleBore = new DutyCycleEncoder(0);

    leftAngMotor.setInverted(true);
    rightAngMotor.setInverted(false);
    leftExtMotor.setInverted(true);
    rightExtMotor.setInverted(false);

    leftAngMotor.setNeutralMode(NeutralMode.Brake);
    rightAngMotor.setNeutralMode(NeutralMode.Brake);
    leftExtMotor.setNeutralMode(NeutralMode.Brake);
    rightExtMotor.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    
    //SmartDashboard.putNumber("Left Angle Encoder Position", leftAngleBore.get());
    //SmartDashboard.putNumber("Right Angle Encoder Position", rightAngleBore.get());

  }

  public void changeAngleUsingPower(double speed) {

    leftAngMotor.set(TalonFXControlMode.PercentOutput, speed);
    rightAngMotor.set(TalonFXControlMode.PercentOutput, speed);

  }

  public void extendArmUsingPower(double speed) {

    leftExtMotor.set(TalonFXControlMode.PercentOutput, speed);
    rightExtMotor.set(TalonFXControlMode.PercentOutput, speed);

  }

}

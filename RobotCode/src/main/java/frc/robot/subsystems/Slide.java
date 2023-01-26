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

  //DutyCycleEncoder angleBore;
 
  public Slide() {

    leftAngMotor = new TalonFX(Constants.leftAngMotorPort);
    rightAngMotor = new TalonFX(Constants.rightAngMotorPort);
    leftExtMotor = new TalonFX(Constants.leftExtMotorPort);
    rightExtMotor = new TalonFX(Constants.rightExtMotorPort);

    //angleBore = new DutyCycleEncoder(Constants.angleEncoderChannel);
 
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
 
    //SmartDashboard.putNumber("Angle Encoder Position", angleBore.get());
    SmartDashboard.putNumber("Angle Encoder Maximum", Constants.maxAngleEncoderValue);
    SmartDashboard.putNumber("Angle Encoder Minimum", Constants.minAngleEncoderValue);

  }

  public void changeAngleUsingPower(double speed) {

    /*
    if (angleBore.get() < Constants.maxAngleEncoderValue && angleBore.get() > Constants.minAngleEncoderValue) {
      
    leftAngMotor.set(TalonFXControlMode.PercentOutput, speed);
    rightAngMotor.set(TalonFXControlMode.PercentOutput, speed);

    } else {

    leftAngMotor.set(TalonFXControlMode.PercentOutput, 0);
    rightAngMotor.set(TalonFXControlMode.PercentOutput, 0);

    }
    */

  }

  public void extendArmUsingPower(double speed) {

    leftExtMotor.set(TalonFXControlMode.PercentOutput, speed);
    rightExtMotor.set(TalonFXControlMode.PercentOutput, speed);

  }

}

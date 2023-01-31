// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slide extends SubsystemBase {

  TalonFX leftAngMotor;
  TalonFX rightAngMotor;
  TalonFX extMotor;
 
  //DutyCycleEncoder angleBore;
 
  public Slide() {

    leftAngMotor = new TalonFX(Constants.leftAngMotorPort);
    rightAngMotor = new TalonFX(Constants.rightAngMotorPort);
    extMotor = new TalonFX(Constants.extMotorPort);
 
    //angleBore = new DutyCycleEncoder(Constants.angleEncoderChannel);
 
    leftAngMotor.setInverted(true);
    rightAngMotor.setInverted(false);

    leftAngMotor.setNeutralMode(NeutralMode.Brake);
    rightAngMotor.setNeutralMode(NeutralMode.Brake);
    extMotor.setNeutralMode(NeutralMode.Brake);
 
  }

  @Override
  public void periodic() {
 
    //SmartDashboard.putNumber("Angle Encoder Position", angleBore.get());
    SmartDashboard.putNumber("Angle Encoder Maximum", Constants.maxAngleEncoderValue);
    SmartDashboard.putNumber("Angle Encoder Minimum", Constants.minAngleEncoderValue);
    SmartDashboard.putNumber("Extension Encoder", extMotor.getSelectedSensorPosition());
 
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

    extMotor.set(TalonFXControlMode.PercentOutput, speed);

  }

}

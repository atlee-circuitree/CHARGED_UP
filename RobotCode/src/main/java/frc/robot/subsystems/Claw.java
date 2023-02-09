// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  
  CANSparkMax clawMotor = null;
  CANSparkMax rotateClawMotor = null;
  SparkMaxAbsoluteEncoder rotateEncoder;
 
  public Claw() {
 
    clawMotor = new CANSparkMax(Constants.clawMotorPort, MotorType.kBrushless);
    rotateClawMotor = new CANSparkMax(Constants.rotateClawMotorPort, MotorType.kBrushless);
 
    rotateEncoder = rotateClawMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    clawMotor.setIdleMode(IdleMode.kBrake);
    rotateClawMotor.setIdleMode(IdleMode.kBrake);
 
  }

  @Override
  public void periodic() {
 
    SmartDashboard.putNumber("Rotation Encoder Maximum", Constants.maxRotationEncoderValue);
    SmartDashboard.putNumber("Rotation Encoder Minimum", Constants.minRotationEncoderValue);

    SmartDashboard.putNumber("Rotation Encoder Position", rotateEncoder.getPosition());

  }

  public void runClaw(double speed) {

    clawMotor.set(speed);

  }

  public void rotateClaw(double speed) {

    rotateClawMotor.set(speed);

  }

}

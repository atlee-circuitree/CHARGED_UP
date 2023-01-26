// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  
  //CANSparkMax clawMotor = null;
  //CANSparkMax rotateClawMotor = null;

  DutyCycleEncoder rotationBore;

  public Claw() {
 
    //clawMotor = new CANSparkMax(Constants.clawMotorPort, MotorType.kBrushless);
    //rotateClawMotor = new CANSparkMax(Constants.rotateClawMotorPort, MotorType.kBrushless);

    //clawMotor.setIdleMode(IdleMode.kBrake);
    //rotateClawMotor.setIdleMode(IdleMode.kBrake);
 
    rotationBore = new DutyCycleEncoder(Constants.rotationEncoderChannel);

  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Rotation Encoder Position", rotationBore.get());
    SmartDashboard.putNumber("Rotation Encoder Maximum", Constants.maxRotationEncoderValue);
    SmartDashboard.putNumber("Rotation Encoder Minimum", Constants.minRotationEncoderValue);

  }

  public void runClaw(double speed) {

    //clawMotor.set(speed);

  }

  public void rotateClaw(double speed) {

    if (rotationBore.get() < Constants.maxRotationEncoderValue && rotationBore.get() > Constants.minRotationEncoderValue) {
      
      //rotateClawMotor.set(speed);
      
      } else {
  
      //rotateClawMotor.set(0);
       
      }

  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  DutyCycleEncoder rotationEncoder;
  DutyCycleEncoder grabEncoder;
  double currentGrabPosition;
  double lastGrabPosition;
  double velocity;
  double timeStuck;
  double targetAngle;
  double rotation;
  double claw;
 
  public Claw() {
 
    clawMotor = new CANSparkMax(Constants.clawMotorPort, MotorType.kBrushless);
    rotateClawMotor = new CANSparkMax(Constants.rotateClawMotorPort, MotorType.kBrushless);
 
    rotationEncoder = new DutyCycleEncoder(Constants.clawRotationEncoderDIO);
    grabEncoder = new DutyCycleEncoder(Constants.clawGrabEncoderDIO);

    clawMotor.setIdleMode(IdleMode.kBrake);
    rotateClawMotor.setIdleMode(IdleMode.kBrake);
 
  }

  @Override
  public void periodic() {
 
    rotation = rotationEncoder.getAbsolutePosition();

    SmartDashboard.putNumber("Claw Velocity", velocity);
    SmartDashboard.putNumber("Claw Rotation", rotation);
    SmartDashboard.putNumber("Claw Rotation Encoder Abs Position", rotationEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Claw Grab", claw);
    SmartDashboard.putNumber("Claw Grab Encoder Abs Pos", grabEncoder.getAbsolutePosition());

  }

  public void runClaw(double speed) {
 
    if (speed > 0 && grabEncoder.getAbsolutePosition() > Constants.maxGrabEncoderValue) {

      clawMotor.set(0);

    } else if (speed < 0 && grabEncoder.getAbsolutePosition() < Constants.minGrabEncoderValue) {

      clawMotor.set(0);

    } else {

      clawMotor.set(speed);

    }

  }

  public void rotateClaw(double speed) {
 
    if (speed < 0 && rotation < Constants.maxClockwiseRotationEncoderValue) {

      rotateClawMotor.set(0);   //turn Clockwise

    } 
    else if (speed > 0 && rotation > Constants.maxCounterClockwiseRotationEncoderValue) {
 
      rotateClawMotor.set(0);   // turn Counter Clockwise
  
    } 
    else {

      rotateClawMotor.set(speed);

    }  
    
  }
 
  public void grabToAngle(double angle, double speed) {

    if (claw > angle + 1) {
 
     clawMotor.set(-speed);
 
    } else if (claw < angle - 1) {
 
     clawMotor.set(speed);
 
    }
 
   }

  public double getRotation() {

    return rotation;

  }

  public double getGrabRotation() {

    return claw;

  }

}

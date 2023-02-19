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
  double targetAngle;
  double rotation;
  double claw;
  enum clawPosition {

  LEFT,
  RIGHT,
  CENTER

  }
  clawPosition currentPosition = clawPosition.LEFT;
  clawPosition targetPosition = clawPosition.LEFT;
 
  public Claw() {
 
    clawMotor = new CANSparkMax(Constants.clawMotorPort, MotorType.kBrushless);
    rotateClawMotor = new CANSparkMax(Constants.rotateClawMotorPort, MotorType.kBrushless);
 
    rotationEncoder = new DutyCycleEncoder(Constants.clawRotationEncoderDIO);
    grabEncoder = new DutyCycleEncoder(Constants.clawGrabEncoderDIO);

    clawMotor.setIdleMode(IdleMode.kBrake);
    rotateClawMotor.setIdleMode(IdleMode.kCoast);
 
  }

  @Override
  public void periodic() {
    //rotation = ((rotationEncoder.getAbsolutePosition() - .354) / .003033333) + 76;
    rotation = rotationEncoder.getAbsolutePosition();
    claw = (grabEncoder.getAbsolutePosition() - .321) / .00242222222;
    /* 
    if (rotation < 241 && rotation > 225) {

    currentPosition = clawPosition.LEFT;

    } else if (rotation > 85 && rotation < 100) {

    currentPosition = clawPosition.RIGHT;

    } else if (rotation < 5 && rotation > -5) {

    currentPosition = clawPosition.CENTER;

    }

    if (targetPosition != currentPosition && targetPosition == clawPosition.LEFT) {

      clawMotor.set(.3);

    } else if (targetPosition != currentPosition && targetPosition == clawPosition.RIGHT) {

      clawMotor.set(-.3);

    } else if (targetPosition != currentPosition && targetPosition == clawPosition.CENTER && targetPosition == clawPosition.LEFT) {

      clawMotor.set(.3);

    } else if (targetPosition != currentPosition && targetPosition == clawPosition.CENTER && targetPosition == clawPosition.RIGHT) {

      clawMotor.set(-.3);

    } else {

      clawMotor.set(0);
`
    
    }

    */
    SmartDashboard.putNumber("Claw Rotation", rotation);
    SmartDashboard.putNumber("Claw Rotation Encoder Abs Position", rotationEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Claw Grab", claw);
    SmartDashboard.putNumber("Claw Grab Encoder Abs Pos", grabEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Claw Voltage Compensation", clawMotor.getVoltageCompensationNominalVoltage());
    SmartDashboard.putNumber("Claw Motor Temperture", clawMotor.getMotorTemperature());
    SmartDashboard.putNumber("Claw Ramp Rate", clawMotor.getClosedLoopRampRate());

  }

  public void runClaw(double speed) {
 
    if (speed > 0 && claw > Constants.maxGrabEncoderValue) {

      clawMotor.set(0);

    } else if (speed < 0 && claw < Constants.minGrabEncoderValue) {

      clawMotor.set(0);
  
    } else {

      clawMotor.set(speed);

    }

  }

  public void rotateClaw(double speed) {
 
    if (speed > 0 && rotation < Constants.maxClockwiseRotationEncoderValue) {

      rotateClawMotor.set(speed);   //turn Clockwise

    } 
    else if (speed < 0 && rotation > Constants.maxCounterClockwiseRotationEncoderValue) {

      rotateClawMotor.set(speed);   // turn Counter Clockwise
  
    } 
    else {

      rotateClawMotor.set(0);

    }  
    
  }

  public void rotateToPosition(clawPosition position, double speed) {

    targetPosition = position;

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

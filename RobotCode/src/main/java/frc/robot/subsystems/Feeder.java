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

public class Feeder extends SubsystemBase {

  CANSparkMax leftFeederMotor;
  CANSparkMax rightFeederMotor;
  CANSparkMax rotationFeederMotor;
  DutyCycleEncoder feederEncoder;

  /** Creates a new Feeder. */
  public Feeder() {
    
    leftFeederMotor = new CANSparkMax(Constants.leftFeederMotorPort, MotorType.kBrushless);
    rightFeederMotor = new CANSparkMax(Constants.rightFeederMotorPort, MotorType.kBrushless);
    rotationFeederMotor = new CANSparkMax(Constants.rotationFeederMotorPort, MotorType.kBrushless);

    rotationFeederMotor.setIdleMode(IdleMode.kBrake);

    feederEncoder = new DutyCycleEncoder(Constants.feederRotationEncoderDIO);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left Feeder Motor Speed", leftFeederMotor.get());
    SmartDashboard.putNumber("Right Feeder Motor Speed", rightFeederMotor.get());
    SmartDashboard.putNumber("Feeder Absolute Encoder", feederEncoder.getAbsolutePosition());

  }
  
  //Postive equals intake 
  //Negative equals outtake
  public void runFeeder(double speed) {
    leftFeederMotor.set(-speed);
    rightFeederMotor.set(speed);
  }

  public void rotateFeeder(double speed) {

    /*
    if (speed > 0 && feederEncoder.getAbsolutePosition() > .1 && feederEncoder.getAbsolutePosition() < .5) {

      rotationFeederMotor.set(0);

    } else if (speed < 0 && feederEncoder.getAbsolutePosition() < .8 && feederEncoder.getAbsolutePosition() > .5) {

      rotationFeederMotor.set(0);

    } else {

      rotationFeederMotor.set(speed);

    }
    */

    rotationFeederMotor.set(speed);
   

  }

  public double absoluteClawPosition() {

    return feederEncoder.getAbsolutePosition();

  }

}

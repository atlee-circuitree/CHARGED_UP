// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  CANSparkMax leftFeederMotor;
  CANSparkMax rightFeederMotor;
  DutyCycleEncoder leftFeederEncoder;
  DutyCycleEncoder rightFeederEncoder;

  /** Creates a new Feeder. */
  public Feeder() {
    
    leftFeederMotor = new CANSparkMax(Constants.leftFeederMotorPort, MotorType.kBrushless);
    rightFeederMotor = new CANSparkMax(Constants.rightFeederMotorPort, MotorType.kBrushless);

    leftFeederEncoder = new DutyCycleEncoder(Constants.leftFeederMotorEncoderDIO);
    rightFeederEncoder = new DutyCycleEncoder(Constants.rightFeederMotorEncoderDIO);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left Feeder Motor Speed", leftFeederMotor.get());
    SmartDashboard.putNumber("Right Feeder Motor Speed", rightFeederMotor.get());

  }

  public void runFeeder(double speed) {
    leftFeederMotor.set(-speed);
    rightFeederMotor.set(speed);
  }

}

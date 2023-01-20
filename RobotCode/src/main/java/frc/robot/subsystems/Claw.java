// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  
  CANSparkMax leftClawMotor = null;
  CANSparkMax rightClawMotor = null;

  public Claw() {

    leftClawMotor = new CANSparkMax(Constants.leftClawMotorPort, MotorType.kBrushless);
    rightClawMotor = new CANSparkMax(Constants.rightClawMotorPort, MotorType.kBrushless);

    leftClawMotor.setIdleMode(IdleMode.kBrake);
    rightClawMotor.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runClaw(int speed) {

    leftClawMotor.set(speed);
    rightClawMotor.set(speed);

  }

}

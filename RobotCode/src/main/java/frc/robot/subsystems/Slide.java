// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
 
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slide extends SubsystemBase {

  TalonFX leftExtMotor;
  TalonFX rightExtMotor;
  TalonFX angMotor;

  public String Competition;
  public String One_Controller;
  String modeSelected;

  double angleBore;
  double angle;
  double tolerance = 100;
  double CurrentStage = 1;
 
 
  public Slide() {

    leftExtMotor = new TalonFX(Constants.leftExtMotorPort);
    rightExtMotor = new TalonFX(Constants.rightExtMotorPort);
    angMotor = new TalonFX(Constants.angMotorPort);
  
    leftExtMotor.setInverted(true);
    rightExtMotor.setInverted(false);

    leftExtMotor.setNeutralMode(NeutralMode.Brake);
    rightExtMotor.setNeutralMode(NeutralMode.Brake);
    angMotor.setNeutralMode(NeutralMode.Brake);
 
  }

  @Override
  public void periodic() {
 
    angleBore = angMotor.getSelectedSensorPosition();
    angle = ((angleBore / -3352)) + 180;
  
    SmartDashboard.putNumber("Angle Encoder Maximum", Constants.maxAngleEncoderValue);
    SmartDashboard.putNumber("Angle Encoder Minimum", Constants.minAngleEncoderValue);
    SmartDashboard.putNumber("Angle Encoder Position", angle);

    if (angleBore > Constants.extensionStage1EncoderValue + tolerance && 
    angleBore < Constants.extensionStage1EncoderValue - tolerance ) {

    CurrentStage = 1;

    } else if (angleBore > Constants.extensionStage2EncoderValue + tolerance && 
    angleBore < Constants.extensionStage2EncoderValue - tolerance ) {

    CurrentStage = 2;

    } else if (angleBore > Constants.extensionStage3EncoderValue + tolerance && 
    angleBore < Constants.extensionStage3EncoderValue - tolerance ) {

    CurrentStage = 3;

    } else {

    CurrentStage = 0;

    }

    SmartDashboard.putNumber("Extension Stage", CurrentStage);
 
  }

  public void changeAngleUsingPower(double speed) {
 
    angMotor.set(ControlMode.PercentOutput, speed);

  }

  public void extendArmUsingPower(double speed) {

    leftExtMotor.set(ControlMode.PercentOutput, speed);
    rightExtMotor.set(ControlMode.PercentOutput, speed);

  }

  public void extendToStage(int Stage) {

    if (Stage == 1) {

      if (angleBore < (Constants.extensionStage1EncoderValue - tolerance)) {

        angMotor.set(TalonFXControlMode.PercentOutput, .3);

      } else if (angleBore > (Constants.extensionStage1EncoderValue + tolerance)) {

        angMotor.set(TalonFXControlMode.PercentOutput, -.3);

      } else {

        angMotor.set(TalonFXControlMode.PercentOutput, 0);

      }

    }

    if (Stage == 2) {

      if (angleBore < (Constants.extensionStage2EncoderValue - tolerance)) {

        angMotor.set(TalonFXControlMode.PercentOutput, .3);

      } else if (angleBore > (Constants.extensionStage2EncoderValue + tolerance)) {

        angMotor.set(TalonFXControlMode.PercentOutput, -.3);

      } else {

        angMotor.set(TalonFXControlMode.PercentOutput, 0);

      }

    }

    if (Stage == 3) {

      if (angleBore < (Constants.extensionStage3EncoderValue - tolerance)) {

        angMotor.set(TalonFXControlMode.PercentOutput, .3);

      } else if (angleBore > (Constants.extensionStage3EncoderValue + tolerance)) {

        angMotor.set(TalonFXControlMode.PercentOutput, -.3);

      } else {

        angMotor.set(TalonFXControlMode.PercentOutput, 0);

      }

    }

  }

}

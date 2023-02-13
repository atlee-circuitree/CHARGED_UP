// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Slide extends SubsystemBase {

  TalonFX leftExtMotor;
  TalonFX rightExtMotor;
  TalonFX angMotor;

  PIDController anglePID;
  PIDController extensionPID;

  DutyCycleEncoder angleEncoder;
  DutyCycleEncoder extEncoder;

  SimpleMotorFeedforward angleFeed;
 
  double angleBore;
  double angle;
  double tolerance = 100;
  double CurrentStage = 1;
 
  public Slide() {

    leftExtMotor = new TalonFX(Constants.leftExtMotorPort);
    rightExtMotor = new TalonFX(Constants.rightExtMotorPort);
    angMotor = new TalonFX(Constants.angMotorPort);

    rightExtMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightExtMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

    angleEncoder = new DutyCycleEncoder(0);
    extEncoder = new DutyCycleEncoder(9);

    anglePID = new PIDController(.01, 0, .01);
    extensionPID = new PIDController(.01, 0, .01);
    angleFeed = new SimpleMotorFeedforward(.01, .01);
 
    leftExtMotor.setInverted(true);
    rightExtMotor.setInverted(false);

    leftExtMotor.setNeutralMode(NeutralMode.Brake);
    rightExtMotor.setNeutralMode(NeutralMode.Brake);
    angMotor.setNeutralMode(NeutralMode.Brake);
 
  }

  @Override
  public void periodic() {

    angle = (angleEncoder.getAbsolutePosition() - .3903) * 384.6153;
  
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Extension", extEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Custom Angle", 0);
 
  }

  public void changeAngleUsingPower(double speed) {
 
    if (speed > 0 && angle > Constants.maxAngleEncoderValue) {

      angMotor.set(ControlMode.PercentOutput, 0);

    } else if (speed < 0 && angle < Constants.minAngleEncoderValue) {

      angMotor.set(ControlMode.PercentOutput, 0);
  
    } else {

      angMotor.set(ControlMode.PercentOutput, speed);

    }
 
  }

  public double getAngle() {

    return angle;

  }

  public void setZeroAngle() {

    Preferences.initDouble(Constants.angleZeroKey, angleEncoder.getAbsolutePosition());

  }

  public void changeAngleUsingVoltage(double speed) {
 
    angMotor.set(ControlMode.Current, angleFeed.calculate(speed));
 
  }

  public void extendArmUsingPower(double speed) {

    leftExtMotor.set(ControlMode.PercentOutput, speed);
    rightExtMotor.set(ControlMode.PercentOutput, speed);

  }

  public void extendToStage(int Stage) {
    

  }

}

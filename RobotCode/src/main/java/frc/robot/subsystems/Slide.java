// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
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

  double extOffset = 0;
  boolean runOnce = false;

  SimpleMotorFeedforward angleFeed;
  SlewRateLimiter slowSlew;
 
  double angle;
  double extension;
  double setExtensionOffset;
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
 
    anglePID = new PIDController(.01, 0, 0);
    extensionPID = new PIDController(.01, 0, .01);
    angleFeed = new SimpleMotorFeedforward(.01, .01);

    slowSlew = new SlewRateLimiter(10);
 
    leftExtMotor.setInverted(true);
    rightExtMotor.setInverted(false);

    leftExtMotor.setNeutralMode(NeutralMode.Brake);
    rightExtMotor.setNeutralMode(NeutralMode.Brake);
    angMotor.setNeutralMode(NeutralMode.Brake);

    //Configure Extension Offset
    
    //Inital Position
    //Offset
    extOffset = extEncoder.getAbsolutePosition();

  }

  @Override
  public void periodic() {

    angle = ((angleEncoder.getAbsolutePosition() - .3903) * 384.6153) - Constants.angleOffset;
    extension = extEncoder.getDistance();
 
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Extension", extEncoder.getDistance());
    SmartDashboard.putNumber("Extension Absolute", extEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Extension Inches", getExtensionEncoderInches());
    SmartDashboard.putNumber("Extension Offset", extOffset);
    SmartDashboard.putBoolean("Extension Calibrated", runOnce);
    SmartDashboard.getNumber("Custom Angle", 0);
 
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

  public void changeAngleUsingPowerNoLimit(double speed) {
 
    angMotor.set(ControlMode.PercentOutput, speed);
 
  }

  public double getAngle() {

    return angle;

  }

  public double getExtension() {

    return extEncoder.getDistance();

  }

  public boolean WithinTolerence(double value, double targetValue, double tolerance) {

    if (value > targetValue - tolerance && value < targetValue + tolerance) {

      return true;

    } else {

      return false;

    }

  }

  public void setZeroAngle() {

    Preferences.initDouble(Constants.angleZeroKey, angleEncoder.getAbsolutePosition());

  }

  public void changeAngleUsingVoltage(double speed) {
 
    angMotor.set(ControlMode.Current, anglePID.calculate(speed));
 
  }

  public void extendArmUsingPower(double speed) {

    if (speed > 0 && getExtensionEncoderInches() > Constants.maxExtensionValue - extOffset) {

      leftExtMotor.set(ControlMode.PercentOutput, 0);
      rightExtMotor.set(ControlMode.PercentOutput, 0);
      System.out.print("Max Limit Hit");

    } else if(speed < 0 && getExtensionEncoderInches() < Constants.minExtensionValue) {

      leftExtMotor.set(ControlMode.PercentOutput, 0);
      rightExtMotor.set(ControlMode.PercentOutput, 0);
      System.out.print("Min Limit Hit");
        
    } else {

      leftExtMotor.set(ControlMode.PercentOutput, speed);
      rightExtMotor.set(ControlMode.PercentOutput, speed);

    }

    //leftExtMotor.set(ControlMode.PercentOutput, speed);
    //rightExtMotor.set(ControlMode.PercentOutput, speed);
    
  }

  public void extendArmUsingPowerNoLimit(double speed) {
     
    leftExtMotor.set(ControlMode.PercentOutput, speed);
    rightExtMotor.set(ControlMode.PercentOutput, speed);
 
  }

  public void setExtensionOffset() {

    extEncoder.setPositionOffset(extEncoder.getDistance());

  }

  public void resetExtensionEncoder() {

    extEncoder.reset();

    //extOffset = (extEncoder.getAbsolutePosition() - 1);

  }

  //Extension Inches stuff

  public double getExtensionEncoderInches(){
    
    double encoderInches = extEncoder.getDistance() * 1.5 * Math.PI;

    return encoderInches + extOffset;

  }

}

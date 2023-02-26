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
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

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

  Rev2mDistanceSensor distance;
  
  double angle;
  double extension;
  double tolerance = 100;
  double CurrentStage = 1;
 
  public Slide() {

    leftExtMotor = new TalonFX(Constants.leftExtMotorPort);
    rightExtMotor = new TalonFX(Constants.rightExtMotorPort);
    angMotor = new TalonFX(Constants.angMotorPort);

    distance = new Rev2mDistanceSensor(Port.kMXP);
    distance.setEnabled(true);
    distance.setRangeProfile(RangeProfile.kDefault);
 
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
 
  }

  @Override
  public void periodic() {

    angle = ((angleEncoder.getAbsolutePosition() - .3903) * 384.6153) - Constants.angleOffset;
    extension = extEncoder.getDistance();
    distance.setAutomaticMode(true);
   
    if(distance.getRange() != -1 && distance.getRange() <= 5 && runOnce == false){
      extOffset = distance.getRange() - extEncoder.getDistance();
      runOnce = true;
    }

    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Extension", extEncoder.getDistance());
    SmartDashboard.putNumber("Extension Inches", getExtensionEncoderInches());
    SmartDashboard.putBoolean("Extension Calibrated", runOnce);
    SmartDashboard.getNumber("Custom Angle", 0);
    SmartDashboard.putNumber("Distance", distance.getRange());
 
  }

  public void changeAngleUsingPower(double speed) {
 
    if (speed > 0 && angle > Constants.maxAngleEncoderValue) {

      angMotor.set(ControlMode.PercentOutput, 0);

    } else if (speed < 0 && angle < Constants.minAngleEncoderValue) {

      angMotor.set(ControlMode.PercentOutput, 0);
  
    } else if (distance.getRange() < .5 && distance.getRange() != -1) {

      angMotor.set(ControlMode.PercentOutput, 0);

    } else {

      angMotor.set(ControlMode.PercentOutput, speed);

    }
 
  }

  public double getAngle() {

    return angle;

  }

  public double getExtension() {

    return extEncoder.getDistance();

  }

  public void setZeroAngle() {

    Preferences.initDouble(Constants.angleZeroKey, angleEncoder.getAbsolutePosition());

  }

  public void changeAngleUsingVoltage(double speed) {
 
    angMotor.set(ControlMode.Current, anglePID.calculate(speed));
 
  }

  /*public void changeAngleUsingVelocity(double targetPos, double speed) {
 
    angMotor.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);
 
  }*/

  public void extendArmUsingPower(double speed) {
     
    if (speed > 0 && getExtensionEncoderInches() > Constants.maxExtensionInchValue) {

      leftExtMotor.set(ControlMode.PercentOutput, 0);
      rightExtMotor.set(ControlMode.PercentOutput, 0);

    } else if(speed < 0 && getExtensionEncoderInches() < Constants.minExtensionInchValue) {

      leftExtMotor.set(ControlMode.PercentOutput, 0);
      rightExtMotor.set(ControlMode.PercentOutput, 0);
        
    } else {

      leftExtMotor.set(ControlMode.PercentOutput, speed);
      rightExtMotor.set(ControlMode.PercentOutput, speed);

    }
    
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

  }

  //Extension Inches stuff

  public double getExtensionEncoderInches(){
    
    double encoderInches = extEncoder.getDistance() * 1.5 * Math.PI;

    return encoderInches + extOffset;
  }

}

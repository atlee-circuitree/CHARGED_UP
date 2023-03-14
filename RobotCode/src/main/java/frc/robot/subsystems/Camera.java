// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
 
  public Camera() {
    //usbCamera.setResolution(200, 100);
  
    CameraServer.startAutomaticCapture();
 
    //CameraServer.putVideo("Rear Camera", 640 , 480);
     
  }

  public void initRearCamera() {

    CameraServer.startAutomaticCapture("Rear Camera", 0);
    
  }

  @Override
  public void periodic() {
 
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Electronics extends SubsystemBase {
 
  PowerDistribution powerDistribution;
  PneumaticHub pneumaticHub;

  public Electronics() {

  powerDistribution = new PowerDistribution(0, ModuleType.kRev);
  pneumaticHub = new PneumaticHub(0);

  }

  @Override
  public void periodic() {
     
    powerDistribution.clearStickyFaults();
    pneumaticHub.clearStickyFaults();

  }
}

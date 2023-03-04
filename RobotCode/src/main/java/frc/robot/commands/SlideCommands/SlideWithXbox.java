// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SlideCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Slide;

public class SlideWithXbox extends CommandBase {

  XboxController xbox;

  XboxController xbox1;
  XboxController xbox2;

  Slide slide;

  enum AngleState {

    AUTOMATIC_CONTROL,
    MANUAL_CONTROL

  }

  enum ExtensionState {

    AUTOMATIC_CONTROL,
    MANUAL_CONTROL

  }
 
  double targetAngle = 0;
  double targetExtension = 0;

  AngleState currentAngleState = AngleState.MANUAL_CONTROL;
  ExtensionState currentExtensionState = ExtensionState.MANUAL_CONTROL;

  public SlideWithXbox(XboxController xb1, XboxController xb2, Slide sl) {
  
    xbox1 = xb1;
    xbox2 = xb2;
    slide = sl;

    addRequirements(slide);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if (Constants.modeSelect.getSelected() == "Player_Two") {

      xbox = xbox1;
      
    } else {

      xbox = xbox2;

    }

    targetAngle = slide.getAngle();

    currentAngleState = AngleState.MANUAL_CONTROL;
    currentExtensionState = ExtensionState.MANUAL_CONTROL;
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    System.out.println(currentAngleState + " - " + currentExtensionState);
 
    if (xbox.getLeftY() > .4 || xbox.getLeftY() < -.1) {

      currentAngleState = AngleState.MANUAL_CONTROL;

    }

    if (xbox.getRightY() > .4 || xbox.getRightY() < -.1) {

      currentExtensionState = ExtensionState.MANUAL_CONTROL;

    }

    if (currentAngleState == AngleState.MANUAL_CONTROL) {

      slide.changeAngleUsingPower(-xbox.getLeftY() / .5);
   
    }

    if (currentExtensionState == ExtensionState.MANUAL_CONTROL) {

      if (xbox.getStartButton()) {

        slide.extendArmUsingPowerNoLimit(-xbox.getRightY() / 1);
        slide.resetExtensionEncoder();
  
      } else {
  
        slide.extendArmUsingPower(-xbox.getRightY() / 1);
  
      }
   
    }

    if (currentAngleState == AngleState.AUTOMATIC_CONTROL) {

      if (slide.getAngle() < targetAngle - .5) {

        if (slide.getAngle() > targetAngle - 10 || slide.getAngle() < targetAngle + 10) {

          slide.changeAngleUsingPower(.6);

        } else {

          slide.changeAngleUsingPower(1);

        }

      } else if (slide.getAngle() > targetAngle + .5) {

        if (slide.getAngle() > targetAngle - 10 || slide.getAngle() < targetAngle + 10) {

          slide.changeAngleUsingPower(-.6);

        } else {

          slide.changeAngleUsingPower(-1);

        }

      } else {

        slide.changeAngleUsingPower(0);

      }

    }

    if (currentExtensionState == ExtensionState.AUTOMATIC_CONTROL) {
    
      if (slide.getExtensionEncoderInches() < targetExtension - 1) {

        slide.extendArmUsingPower(.65);

      } else if (slide.getExtensionEncoderInches() > targetExtension + 1) {

        slide.extendArmUsingPower(-.65);

      } else {

        slide.extendArmUsingPower(0);

      }

    }

    if (xbox.getYButtonPressed()) {

      currentAngleState = AngleState.AUTOMATIC_CONTROL;
      currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
      targetAngle = Constants.maxAngleEncoderValue - 4;
      targetExtension = 44;

    }

    if (xbox.getAButtonPressed()) {

      currentAngleState = AngleState.AUTOMATIC_CONTROL;
      currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
      targetAngle = Constants.minAngleEncoderValue + 1; //- .5;
      targetExtension = .5;

    }

    if (xbox.getBButtonPressed()) {

      currentAngleState = AngleState.AUTOMATIC_CONTROL;
      currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
      targetAngle = 26;
      targetExtension = 20;

    }

    if (xbox.getXButtonPressed()) {

      currentAngleState = AngleState.AUTOMATIC_CONTROL;
      currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
      targetAngle = SmartDashboard.getNumber("Custom Angle", 0);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    slide.changeAngleUsingPower(0);
    slide.extendArmUsingPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

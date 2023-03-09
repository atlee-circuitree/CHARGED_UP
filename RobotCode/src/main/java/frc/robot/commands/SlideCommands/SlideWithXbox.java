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

  XboxController primaryXbox;
  XboxController secondaryXbox;

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
 
  double targetAngle;
  double targetExtension;

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
    
    currentAngleState = AngleState.MANUAL_CONTROL;
    currentExtensionState = ExtensionState.MANUAL_CONTROL;

    if (Constants.modeSelect.getSelected() == "Player_Two") {

      primaryXbox = xbox2;
      secondaryXbox = xbox1;
      
    } else {

      primaryXbox = xbox1;
      secondaryXbox = xbox2;

    }

    targetAngle = slide.getAngle();

    currentAngleState = AngleState.MANUAL_CONTROL;
    currentExtensionState = ExtensionState.MANUAL_CONTROL;
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    if (primaryXbox.getLeftY() > .4 || primaryXbox.getLeftY() < -.1) {

      currentAngleState = AngleState.MANUAL_CONTROL;

    }

    if (primaryXbox.getRightY() > .4 || primaryXbox.getRightY() < -.1) {

      currentExtensionState = ExtensionState.MANUAL_CONTROL;

    }

    if (currentAngleState == AngleState.MANUAL_CONTROL) {

      slide.changeAngleUsingPower(-secondaryXbox.getLeftY());
   
    }

    if (currentExtensionState == ExtensionState.MANUAL_CONTROL) {

      if (secondaryXbox.getStartButton()) {

        slide.extendArmUsingPowerNoLimit(-secondaryXbox.getRightY() / 1);
        slide.resetExtensionEncoder();
  
      } else {
  
        slide.extendArmUsingPower(-secondaryXbox.getRightY() / 1);
  
      }
   
    }

    if (currentAngleState == AngleState.AUTOMATIC_CONTROL) {

      if (slide.getAngle() < targetAngle - .5) {

        if (slide.getAngle() > targetAngle - 10 || slide.getAngle() < targetAngle + 10) {

          slide.changeAngleUsingPower(1);

        } else {

          slide.changeAngleUsingPower(1);

        }

      } else if (slide.getAngle() > targetAngle + .5) {

        if (slide.getAngle() > targetAngle - 10 || slide.getAngle() < targetAngle + 10) {

          slide.changeAngleUsingPower(-1);

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

    if (primaryXbox.getYButtonPressed()) {

      targetAngle = Constants.maxAngleEncoderValue - 1;
 
      if (slide.getAngle() > targetAngle - .5 && slide.getAngle() < targetAngle + .5) {

        currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
        targetExtension = Constants.maxExtensionValue;

      } else if (slide.getExtension() > 10) {

        currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
        targetExtension = 1;

      } else {

        currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
        targetExtension = 3;
        currentAngleState = AngleState.AUTOMATIC_CONTROL;
        targetAngle = Constants.maxAngleEncoderValue - 1;

      }
     
    }

    if (primaryXbox.getAButtonPressed()) {

      targetAngle = Constants.minAngleEncoderValue;

      if (slide.getAngle() > targetAngle - .5 && slide.getAngle() < targetAngle + .5) {

        currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
        targetExtension = 3;

      } else if (slide.getExtension() > 10) {

        currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
        targetExtension = 1;

      } else {

        currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
        targetExtension = 3;
        currentAngleState = AngleState.AUTOMATIC_CONTROL;
        targetAngle = Constants.minAngleEncoderValue; //- .5;

      }

    }

    if (primaryXbox.getBButtonPressed()) {

      targetAngle = 30;
 
      if (slide.getAngle() > targetAngle - .5 && slide.getAngle() < targetAngle + .5) {

        currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
        targetExtension = 25;

      } else if (slide.getExtension() > 10) {

        currentExtensionState = ExtensionState.AUTOMATIC_CONTROL;
        targetExtension = 1;

      } else {

        currentAngleState = AngleState.AUTOMATIC_CONTROL;
        targetAngle = 30;

      }

    }

    if (primaryXbox.getXButtonPressed()) {

      currentAngleState = AngleState.MANUAL_CONTROL;
      currentExtensionState = ExtensionState.MANUAL_CONTROL;

    }
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    slide.changeAngleUsingPower(0);
    slide.extendArmUsingPower(0);
    currentAngleState = AngleState.MANUAL_CONTROL;
    currentExtensionState = ExtensionState.MANUAL_CONTROL;


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

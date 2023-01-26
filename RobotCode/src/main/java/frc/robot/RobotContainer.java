// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveWithXbox;
<<<<<<< Updated upstream
import frc.robot.commands.PathFollower;
import frc.robot.commands.PathGenerator;
import frc.robot.commands.PlayAudio;
import frc.robot.commands.RecalibrateModules;
import frc.robot.commands.TestPathFollower;
=======
import frc.robot.commands.AutoCommands.PathFollower;
import frc.robot.commands.ClawCommands.RotateClaw;
import frc.robot.commands.ClawCommands.RunClaw;
import frc.robot.commands.MiscCommands.PlayAudio;
import frc.robot.commands.MiscCommands.RecalibrateModules;
import frc.robot.commands.MiscCommands.TestPathFollower;
import frc.robot.commands.SlideCommands.SlideWithXbox;
>>>>>>> Stashed changes
import frc.robot.subsystems.Audio;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drivetrain drivetrain;
  private final Audio audio;

  private final DriveWithXbox driveWithXbox;
<<<<<<< Updated upstream
=======
  private final SlideWithXbox slideWithXbox;
  private Command GenerateClawCommand(double PercentSpeed) {
    Command runClaw = new RunClaw(claw, PercentSpeed);
    return runClaw;
  }
  private Command GenerateRotateClawCommand(double PercentSpeed) {
    Command rotateClaw = new RotateClaw(claw, PercentSpeed);
    return rotateClaw;
  }
>>>>>>> Stashed changes
  private final RecalibrateModules recalibrateModules;

  //private final PathGenerator pathGenerator;
  private final PathFollower pathFollower;
  //private final PathFollower pathFollower1;
  //private final PathFollower pathFollower2;
  //private final PathFollower pathFollower3;
  //private final PathFollower pathFollower4;
  //private final PathFollower pathFollower5;

  //private final TestPathFollower testPathFollower;
  private final PathEQ pathEQ; 
  private final PathEQ pathEQ1;
  private final PathEQ pathEQ2;
  private final PathEQ pathEQ3;
  private final PathEQ pathEQ4;
  private final PathEQ pathEQ5;

  private final PlayAudio playAudio;

  public XboxController xbox = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    drivetrain = new Drivetrain();
    audio = new Audio();

    playAudio = new PlayAudio(audio, 2, 2);

    pathEQ = new PathEQ(Constants.autoCoordinates, true);
    pathEQ1 = new PathEQ(Constants.autoCoordinates1, true);
    pathEQ2 = new PathEQ(Constants.autoCoordinates2, true);
    pathEQ3 = new PathEQ(Constants.autoCoordinates3, true);
    pathEQ4 = new PathEQ(Constants.autoCoordinates4, true);
    pathEQ5 = new PathEQ(Constants.autoCoordinates5, true);

    //Teleop commands
    driveWithXbox = new DriveWithXbox(drivetrain, xbox, false);
 
    driveWithXbox.addRequirements(drivetrain);
    drivetrain.setDefaultCommand(driveWithXbox);

    recalibrateModules = new RecalibrateModules(drivetrain, xbox);
    //recalibrateModules.addRequirements(drivetrain);
    //drivetrain.setDefaultCommand(recalibrateModules);
    
    //pathGenerator = new PathGenerator();

    pathFollower = new PathFollower(drivetrain, pathEQ, 0.2, 0.2, 5);
    //pathFollower1 = new PathFollower(drivetrain, pathEQ1, 0.2, 0.2, 5);
    //pathFollower2 = new PathFollower(drivetrain, pathEQ2, 0.2, 0.2, 5);
    //pathFollower3 = new PathFollower(drivetrain, pathEQ3, 0.2, 0.2, 5);
    //pathFollower3 = new PathFollower(drivetrain, pathEQ4, 0.2, 0.2, 5);
    //pathFollower3 = new PathFollower(drivetrain, pathEQ5, 0.2, 0.2, 5);
    //testPathFollower = new TestPathFollower(drivetrain, pathEQ, 0.1, 0.05);
    
    

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
<<<<<<< Updated upstream
    
    JoystickButton driver1L3 = new JoystickButton(xbox, XboxController.Button.kB.value);
    driver1L3.onTrue(playAudio);
=======

    JoystickButton driver1A = new JoystickButton(xbox1, XboxController.Button.kA.value);
    JoystickButton driver1B = new JoystickButton(xbox1, XboxController.Button.kB.value);
    JoystickButton driver1X = new JoystickButton(xbox1, XboxController.Button.kX.value);
    JoystickButton driver1Y = new JoystickButton(xbox1, XboxController.Button.kY.value);
    JoystickButton driver1LB = new JoystickButton(xbox1, XboxController.Button.kLeftBumper.value);
    JoystickButton driver1RB = new JoystickButton(xbox1, XboxController.Button.kRightBumper.value);
    JoystickButton driver1LT = new JoystickButton(xbox1, XboxController.Axis.kLeftTrigger.value);
    JoystickButton driver1RT = new JoystickButton(xbox1, XboxController.Axis.kRightTrigger.value);

    JoystickButton driver2A = new JoystickButton(xbox2, XboxController.Button.kA.value);
    JoystickButton driver2B = new JoystickButton(xbox2, XboxController.Button.kB.value);
    JoystickButton driver2X = new JoystickButton(xbox2, XboxController.Button.kX.value);
    JoystickButton driver2Y = new JoystickButton(xbox2, XboxController.Button.kY.value);
    JoystickButton driver2LB = new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);
    JoystickButton driver2RB = new JoystickButton(xbox2, XboxController.Button.kRightBumper.value);
    JoystickButton driver2LT = new JoystickButton(xbox2, XboxController.Axis.kLeftTrigger.value);
    JoystickButton driver2RT = new JoystickButton(xbox2, XboxController.Axis.kRightTrigger.value);


    driver1A.onTrue(playAudio);

    driver2LT.onTrue(GenerateClawCommand(-.3));
    driver2RT.onTrue(GenerateClawCommand(.3));

    driver2LB.onTrue(GenerateRotateClawCommand(-.3));
    driver2RB.onTrue(GenerateRotateClawCommand(.3));
>>>>>>> Stashed changes

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return pathFollower;
    //return testPathFollower;
  }
}

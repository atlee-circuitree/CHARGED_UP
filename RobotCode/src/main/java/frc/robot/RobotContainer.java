// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveWithXbox;
import frc.robot.commands.AutoCommands.AutoBalance;
import frc.robot.commands.AutoCommands.PathFollower;
import frc.robot.commands.ClawCommands.RotateClaw;
import frc.robot.commands.ClawCommands.RunClaw;
import frc.robot.commands.MiscCommands.PlayAudio;
import frc.robot.commands.MiscCommands.RecalibrateModules;
import frc.robot.commands.MiscCommands.TestPathFollower;
import frc.robot.commands.SlideCommands.SlideWithXbox;
import frc.robot.subsystems.Audio;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Slide;
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
  private final Slide slide;
  private final Claw claw;

  private final DriveWithXbox driveWithXbox;
  private final SlideWithXbox slideWithXbox;
  private final AutoBalance autoBalance;
  private Command GenerateClawCommand(double PercentSpeed) {
    Command runClaw = new RunClaw(claw, PercentSpeed);
    return runClaw;
  }
  private Command GenerateRotateClawCommand(double PercentSpeed) {
    Command runClaw = new RotateClaw(claw, PercentSpeed);
    return runClaw;
  }
  private final RecalibrateModules recalibrateModules;

  //private final PathGenerator pathGenerator;
  private final PathFollower pathFollower;
  //private final TestPathFollower testPathFollower;
  private final PathEQ pathEQ; 

  private final PlayAudio playAudio;

  public XboxController xbox1 = new XboxController(0);
  public XboxController xbox2 = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    drivetrain = new Drivetrain();
    claw = new Claw();
    slide = new Slide();
    audio = new Audio();

    playAudio = new PlayAudio(audio, 2, 2);

    autoBalance = new AutoBalance(drivetrain, xbox1);

    pathEQ = new PathEQ(Constants.autoCoordinates, true);

    //Teleop commands
    driveWithXbox = new DriveWithXbox(drivetrain, xbox1, false);
    slideWithXbox = new SlideWithXbox(xbox2, slide);
 
    driveWithXbox.addRequirements(drivetrain);
    slideWithXbox.addRequirements(slide);
    autoBalance.addRequirements(drivetrain);
    drivetrain.setDefaultCommand(driveWithXbox);
    slide.setDefaultCommand(slideWithXbox);

    recalibrateModules = new RecalibrateModules(drivetrain, xbox1);
    //recalibrateModules.addRequirements(drivetrain);
    //drivetrain.setDefaultCommand(recalibrateModules);
    
    //pathGenerator = new PathGenerator();

    pathFollower = new PathFollower(drivetrain, pathEQ, 0.2, 0.2, 5);
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

    JoystickButton driver1A = new JoystickButton(xbox1, XboxController.Button.kA.value);
    JoystickButton driver1B = new JoystickButton(xbox1, XboxController.Button.kB.value);
    JoystickButton driver1X = new JoystickButton(xbox1, XboxController.Button.kX.value);
    JoystickButton driver1Y = new JoystickButton(xbox1, XboxController.Button.kY.value);
    JoystickButton driver1LB = new JoystickButton(xbox1, XboxController.Button.kLeftBumper.value);
    JoystickButton driver1RB = new JoystickButton(xbox1, XboxController.Button.kRightBumper.value);

    JoystickButton driver2A = new JoystickButton(xbox2, XboxController.Button.kA.value);
    JoystickButton driver2B = new JoystickButton(xbox2, XboxController.Button.kB.value);
    JoystickButton driver2X = new JoystickButton(xbox2, XboxController.Button.kX.value);
    JoystickButton driver2Y = new JoystickButton(xbox2, XboxController.Button.kY.value);
    JoystickButton driver2LB = new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);
    JoystickButton driver2RB = new JoystickButton(xbox2, XboxController.Button.kRightBumper.value);

    driver1A.onTrue(playAudio);

    driver2LB.onTrue(GenerateClawCommand(-.3)).debounce(.3);
    driver2RB.onTrue(GenerateClawCommand(.3)).debounce(.3);

    if (xbox1.getLeftTriggerAxis() > .5) {

      GenerateRotateClawCommand(-.5);

    }

    if (xbox1.getRightTriggerAxis() > .5) {

      GenerateRotateClawCommand(.5);
  
    }

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

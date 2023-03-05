// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoCommands.AngleAndExtendInAuto;
import frc.robot.commands.AutoCommands.AutoBalance;
import frc.robot.commands.AutoCommands.PathFollower;
//import frc.robot.commands.ClawCommands.RotateClaw;
//import frc.robot.commands.ClawCommands.RunClaw;
//import frc.robot.commands.ClawCommands.RunClawUntilClamp;
import frc.robot.commands.DriveCommands.DriveWithXbox;
import frc.robot.commands.FeederCommands.RunFeeder;
import frc.robot.commands.MiscCommands.PlayAudio;
import frc.robot.commands.MiscCommands.RecalibrateModules;
import frc.robot.commands.MiscCommands.TestPathFollower;
import frc.robot.commands.SlideCommands.ResetExtensionEncoder;
import frc.robot.commands.SlideCommands.SlideWithXbox;
import frc.robot.subsystems.Audio;
//import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Slide;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
 // private final Claw claw;
  private final Feeder feeder;
  private final Limelight limelight;

  private DriveWithXbox driveWithXbox;
  private SlideWithXbox slideWithXbox;
  private final AutoBalance autoBalance;
  private final ResetExtensionEncoder resetExtensionEncoder;
 
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

    Constants.modeSelect = new SendableChooser<>();
 
    Constants.modeSelect.setDefaultOption("Competition", "Competition");
    Constants.modeSelect.addOption("Player_Two", "Player_Two");

    SmartDashboard.putData("Select Mode", Constants.modeSelect);

    // Configure the button bindings
    drivetrain = new Drivetrain();
    //claw = new Claw();
    feeder = new Feeder();
    slide = new Slide();
    audio = new Audio();
    limelight = new Limelight();

    limelight.EnableLED();

    resetExtensionEncoder = new ResetExtensionEncoder(slide);
 
    autoBalance = new AutoBalance(drivetrain, xbox1);

    playAudio = new PlayAudio(audio, 0, 0);

    pathEQ = new PathEQ(Constants.autoCoordinates, true);

    //Teleop commands
    driveWithXbox = new DriveWithXbox(drivetrain, limelight, xbox1, xbox2, false);
    slideWithXbox = new SlideWithXbox(xbox1, xbox2, slide);
 
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
    JoystickButton driver1LS = new JoystickButton(xbox1, XboxController.Button.kLeftStick.value);
    JoystickButton driver1RS = new JoystickButton(xbox1, XboxController.Button.kRightStick.value);
    JoystickButton driver1Start = new JoystickButton(xbox1, XboxController.Button.kStart.value);
    JoystickButton driver1Back = new JoystickButton(xbox1, XboxController.Button.kBack.value);

    JoystickButton driver2A = new JoystickButton(xbox2, XboxController.Button.kA.value);
    JoystickButton driver2B = new JoystickButton(xbox2, XboxController.Button.kB.value);
    JoystickButton driver2X = new JoystickButton(xbox2, XboxController.Button.kX.value);
    JoystickButton driver2Y = new JoystickButton(xbox2, XboxController.Button.kY.value);
    JoystickButton driver2LB = new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);
    JoystickButton driver2RB = new JoystickButton(xbox2, XboxController.Button.kRightBumper.value);
    JoystickButton driver2LS = new JoystickButton(xbox2, XboxController.Button.kLeftStick.value);
    JoystickButton driver2RS = new JoystickButton(xbox2, XboxController.Button.kRightStick.value);
    JoystickButton driver2Start = new JoystickButton(xbox2, XboxController.Button.kStart.value);
    JoystickButton driver2Back = new JoystickButton(xbox2, XboxController.Button.kBack.value);

    driver2LB.whileTrue(new RunFeeder(feeder, SmartDashboard.getNumber("Speed", -.25)));
    driver2RB.whileTrue(new RunFeeder(feeder, SmartDashboard.getNumber("Speed", .25)));

    //Trigger Setup
    BooleanSupplier driver1LTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(xbox1.getLeftTriggerAxis() > 0.2){
          return true;
        }
        else{
          return false;
        }
      }
    };
    Trigger driver1LT = new Trigger(driver1LTSupplier);

    BooleanSupplier driver1RTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(xbox1.getRightTriggerAxis() > 0.2){
          return true;
        }
        else{
          return false;
        }
      }
    };
    Trigger driver1RT = new Trigger(driver1RTSupplier);

    BooleanSupplier driver2LTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(xbox2.getLeftTriggerAxis() > 0.2){
          return true;
        }
        else{
          return false;
        }
      }
    };
    Trigger driver2LT = new Trigger(driver2LTSupplier);

    BooleanSupplier driver2RTSupplier = new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        if(xbox2.getRightTriggerAxis() > 0.2){
          return true;
        }
        else{
          return false;
        }
      }
    };
    Trigger driver2RT = new Trigger(driver2RTSupplier);

    //All four face button already used by SlideWithXbox

    /*driver2LB.whileTrue(new RotateClaw(claw, 1));
    driver2RB.whileTrue(new RotateClaw(claw, -1));
    driver2LT.whileTrue(new RunClaw(claw, .55));
    driver2RT.whileTrue(new RunClaw(claw, -.55));*/
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return pathFollower;
    //return testPathFollower;
    return new AngleAndExtendInAuto(slide, feeder, 20, 4); //Removed claw to test feeder

  }
  
}
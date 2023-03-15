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
import frc.robot.commands.AutoCommands.CenterToDistance;
import frc.robot.commands.AutoCommands.DriveBackwardsToDistance;
import frc.robot.commands.AutoCommands.DriveForwardsToDistance;
import frc.robot.commands.AutoCommands.PathFollower;
import frc.robot.commands.AutoCommands.ResetPose;
//import frc.robot.commands.ClawCommands.RotateClaw;
//import frc.robot.commands.ClawCommands.RunClaw;
//import frc.robot.commands.ClawCommands.RunClawUntilClamp;
import frc.robot.commands.DriveCommands.DriveWithXbox;
import frc.robot.commands.FeederCommands.GoToFeederPosition;
import frc.robot.commands.FeederCommands.IntakeFeeder;
import frc.robot.commands.FeederCommands.RotateFeeder;
import frc.robot.commands.FeederCommands.RunFeeder;
import frc.robot.commands.MiscCommands.PlayAudio;
import frc.robot.commands.MiscCommands.RecalibrateModules;
import frc.robot.commands.MiscCommands.TestPathFollower;
import frc.robot.commands.SlideCommands.GoToAngleAndExtension;
import frc.robot.commands.SlideCommands.KillArm;
import frc.robot.commands.SlideCommands.ResetExtensionEncoder;
import frc.robot.commands.SlideCommands.SlideWithXbox;
import frc.robot.subsystems.Audio;
import frc.robot.subsystems.Camera;
//import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Slide;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Camera camera;

  private DriveWithXbox driveWithXbox;
  private SlideWithXbox slideWithXbox;
  private final AutoBalance autoBalance;
  private final ResetExtensionEncoder resetExtensionEncoder;
 
  private final RecalibrateModules recalibrateModules;

  private GoToAngleAndExtension TopPosition;
  private GoToAngleAndExtension TopPositionAuto;
  private GoToAngleAndExtension MiddlePosition;

  //private final PathGenerator pathGenerator;
  private final PathFollower pathFollower;

  private final PathFollower pathFollowerRedSide;
  private final PathFollower pathFollowerRedSideTEST;
  private final PathFollower pathFollowerBlueSide;
  private final PathFollower pathFollowerBlueSideTEST;
  //private final TestPathFollower testPathFollower;
  private final PathEQ pathEQ;

  private final PathEQ pathEQRedSide; 
  private final PathEQ pathEQBlueSide; 
  private final PathEQ pathEQRedSideTEST; 
  private final PathEQ pathEQBlueSideTEST; 

  private final DriveBackwardsToDistance GoPastStartingLine;

  private SequentialCommandGroup ScoreOpeningCube;
  
  private SequentialCommandGroup RedAuto;
  private SequentialCommandGroup TestRedAuto;
  private SequentialCommandGroup BlueAuto;
  private SequentialCommandGroup TestBlueAuto;

  private final CenterToDistance CenterToCubeNode;

  private final PlayAudio playAudio;
 
  public XboxController xbox1 = new XboxController(0);
  public XboxController xbox2 = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Constants.modeSelect = new SendableChooser<>();
    Constants.autoSelect = new SendableChooser<>();

    Constants.modeSelect.setDefaultOption("Competition", "Competition");
    Constants.modeSelect.addOption("Player_Two", "Player_Two");
    Constants.autoSelect.setDefaultOption("Red Side", "Red Side");
    Constants.autoSelect.addOption("Blue Side", "Blue Side");
    Constants.autoSelect.addOption("TEST Red Side", "TEST Red Side");
    Constants.autoSelect.addOption("TEST Blue Side", "TEST Blue Side");

    SmartDashboard.putData("Select Mode", Constants.modeSelect);
    SmartDashboard.putData("Select Auto", Constants.autoSelect);

    // Configure the button bindings
    drivetrain = new Drivetrain();
    feeder = new Feeder();
    slide = new Slide();
    audio = new Audio();
    limelight = new Limelight();
    camera = new Camera();

    limelight.EnableLED();

    resetExtensionEncoder = new ResetExtensionEncoder(slide);
 
    autoBalance = new AutoBalance(drivetrain, xbox1);

    playAudio = new PlayAudio(audio, 0, 0);

    pathEQ = new PathEQ(Constants.autoCoordinates, false);
 
    pathEQRedSide = new PathEQ(Constants.redAuto, true);
    pathEQRedSideTEST = new PathEQ(Constants.redAutoTEST, true);
    pathEQBlueSide = new PathEQ(Constants.redAuto, true);
    pathEQBlueSideTEST = new PathEQ(Constants.redAuto, true);
    //pathEQ = new PathEQ(Constants.testCoords, true);

    GoPastStartingLine = new DriveBackwardsToDistance(drivetrain, limelight, 3, .2);

    CenterToCubeNode = new CenterToDistance(drivetrain, limelight, 3, .1, .5);

    //Teleop commands
    driveWithXbox = new DriveWithXbox(drivetrain, limelight, xbox1, xbox2, false);
    slideWithXbox = new SlideWithXbox(xbox1, xbox2, slide);

    TopPosition = new GoToAngleAndExtension(slide, 31, Constants.maxExtensionValue, 1, true);
    TopPositionAuto = new GoToAngleAndExtension(slide, 31, Constants.maxExtensionValue, 1, false);
    MiddlePosition = new GoToAngleAndExtension(slide, 20, 20, 1, true);

    ScoreOpeningCube = new SequentialCommandGroup(
    //new RunFeeder(feeder, .2).withTimeout(.5),
    //TopPositionAuto, 
    //new RunFeeder(feeder, -.2).withTimeout(1),
    //new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1),
    //new DriveBackwardsToDistance(drivetrain, limelight, 2.9, .2),
    new DriveBackwardsToDistance(drivetrain, limelight, 4.5, .2),
    new DriveForwardsToDistance(drivetrain, limelight, 5.5, .2));

    driveWithXbox.addRequirements(drivetrain);
    slideWithXbox.addRequirements(slide);
    autoBalance.addRequirements(drivetrain);
    drivetrain.setDefaultCommand(driveWithXbox);
    slide.setDefaultCommand(slideWithXbox);

    recalibrateModules = new RecalibrateModules(drivetrain, xbox1);
    //recalibrateModules.addRequirements(drivetrain);
    //drivetrain.setDefaultCommand(recalibrateModules);
 
    //pathGenerator = new PathGenerator();

    pathFollower = new PathFollower(drivetrain, limelight, pathEQ, 0.3, 5);

    pathFollowerRedSide = new PathFollower(drivetrain, limelight, pathEQRedSide, 0.3, 5);
    pathFollowerRedSideTEST = new PathFollower(drivetrain, limelight, pathEQRedSideTEST, 0.3, 5);
    pathFollowerBlueSide = new PathFollower(drivetrain, limelight, pathEQBlueSide, 0.3, 5);
    pathFollowerBlueSideTEST = new PathFollower(drivetrain, limelight, pathEQBlueSideTEST, 0.3, 5);
    //testPathFollower = new TestPathFollower(drivetrain, pathEQ, 0.1, 0.05);

    RedAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, -0.920, 0).withTimeout(.1), pathFollowerRedSide);
    TestRedAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, -0.920, 0).withTimeout(.1), pathFollowerRedSideTEST);
    BlueAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, -0.920, 0).withTimeout(.1), pathFollowerBlueSide);
    TestBlueAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, -0.920, 0).withTimeout(.1), pathFollowerBlueSideTEST);

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

    driver2Y.onTrue(TopPosition);
    driver2B.onTrue(MiddlePosition);
    driver2A.onTrue(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue + 1, 1, true));
    driver2X.whileTrue(new KillArm(slide));

    driver2LB.onTrue(new GoToFeederPosition(feeder, -.2));
    driver2RB.onTrue(new GoToFeederPosition(feeder, .2));

    driver1RT.whileTrue(new IntakeFeeder(feeder));
    driver1LT.whileTrue(new RunFeeder(feeder, -1));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return pathFollower;
    //return ScoreOpeningCube;
    /* 
    if (Constants.autoSelect.getSelected() == "Red Side") {

      return RedAuto;

    } else if (Constants.autoSelect.getSelected() == "Blue Side") {

      return BlueAuto;

    } else if (Constants.autoSelect.getSelected() == "TEST Red Side") {

      return TestRedAuto;

    } else if (Constants.autoSelect.getSelected() == "TEST Blue Side") {

      return TestBlueAuto;

    } else {

      return RedAuto;

    }
    */
    //return testPathFollower;
    //return new AngleAndExtendInAuto(slide, feeder, 20, 4);
    //return new GoToAngleAndExtension(slide, 30, 40, 1);
  }
  
}
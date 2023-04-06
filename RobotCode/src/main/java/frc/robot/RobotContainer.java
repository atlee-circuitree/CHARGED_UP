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
import frc.robot.commands.AutoCommands.AutoBalance;
import frc.robot.commands.AutoCommands.CenterToDistance;
import frc.robot.commands.AutoCommands.DriveBackwardsToDistance;
import frc.robot.commands.AutoCommands.DriveForwardsToDistance;
import frc.robot.commands.AutoCommands.PathFollower;
import frc.robot.commands.AutoCommands.ResetPose;
import frc.robot.commands.AutoCommands.ResetPoseToLimelight;
import frc.robot.commands.DriveCommands.DriveWithXbox;
import frc.robot.commands.FeederCommands.GoToFeederPosition;
import frc.robot.commands.FeederCommands.IntakeFeeder;
import frc.robot.commands.FeederCommands.RotateFeeder;
import frc.robot.commands.FeederCommands.RunFeeder;
import frc.robot.commands.FeederCommands.RunFeederContinously;
import frc.robot.commands.MiscCommands.PlayAudio;
import frc.robot.commands.MiscCommands.RecalibrateModules;
import frc.robot.commands.MiscCommands.TestPathFollower;
import frc.robot.commands.SlideCommands.GoToAngleAndExtension;
import frc.robot.commands.SlideCommands.KillArm;
import frc.robot.commands.SlideCommands.ResetExtensionEncoder;
import frc.robot.commands.SlideCommands.SlideWithXbox;
import frc.robot.subsystems.Audio;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Slide;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Paths;
import frc.robot.Constants.FeederPosition;
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
  private final Feeder feeder;
  private final Limelight limelight;
  private final Camera camera;

  private DriveWithXbox driveWithXbox;
  private SlideWithXbox slideWithXbox;
  private final AutoBalance AutoBalance;
  private final ResetExtensionEncoder resetExtensionEncoder;
 
  private final RecalibrateModules recalibrateModules;

  private GoToAngleAndExtension TopPosition;
  private GoToAngleAndExtension TopPositionAuto;
  private GoToAngleAndExtension SubstationPosition;
  private GoToAngleAndExtension BottomPosition;

 
  private final DriveBackwardsToDistance GoPastStartingLine;

  private SequentialCommandGroup ScoreOpeningCube;

  private final CenterToDistance CenterToCubeNode;

  private final PlayAudio playAudio;
 
  public XboxController xbox1 = new XboxController(0);
  public XboxController xbox2 = new XboxController(1);

  PathFollower GeneratePath(double[][] Cords) {

    PathEQ path;
    PathFollower pathFollower;
    path = new PathEQ(Cords, true);
    pathFollower = new PathFollower(drivetrain, limelight, path, .3, 5);

    return pathFollower;

  }

  SequentialCommandGroup GenerateScoreHigh() {

    return new SequentialCommandGroup(
      new ParallelCommandGroup(new RunFeeder(feeder, .3).withTimeout(.5), new GoToAngleAndExtension(slide, Constants.maxAngleEncoderValue, Constants.maxExtensionValue, 1, false)),
      new RunFeeder(feeder, -1).withTimeout(.5));

  }

  SequentialCommandGroup GenerateCollectPosition() {

    return new SequentialCommandGroup(new GoToAngleAndExtension(slide, 27, 2.2, 1, false, 2.2));
 
  }

  AutoBalance GenerateAutoBalance() {

    return new AutoBalance(drivetrain, xbox1, 5);

  }

  //Command Groups
  SequentialCommandGroup Tag1GrabBottomCone;
  SequentialCommandGroup Tag1GrabBottomCube;
  
  SequentialCommandGroup Tag2JustBalance;
  SequentialCommandGroup Tag2BehindTheLineBalance;
 
  SequentialCommandGroup Tag3GrabTopCone;

  SequentialCommandGroup Tag6GrabTopCone;
  
  SequentialCommandGroup Tag7BehindTheLineBalance;

  SequentialCommandGroup Tag8GrabBottomCone;

  SequentialCommandGroup StraightBackTest;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Constants.modeSelect = new SendableChooser<>();
    Constants.autoSelect = new SendableChooser<>();

    Constants.modeSelect.setDefaultOption("Competition", "Competition");
    Constants.autoSelect.addOption("Nothing", "Nothing");
    Constants.modeSelect.addOption("Testing", "Testing");
    Constants.modeSelect.addOption("Player_Two", "Player_Two");

    Constants.autoSelect.addOption("AutoBalance", "AutoBalance");

    Constants.autoSelect.addOption("Red Left Grab Bottom Cone", "Tag1GrabBottomCone");

    Constants.autoSelect.addOption("Red Middle pass Line Balance", "Tag2BehindTheLineBalance");

    Constants.autoSelect.addOption("Red Right Grab Top Cone", "Tag3GrabTopCone");


    Constants.autoSelect.addOption("Blue Right Grab Bottom Cone", "Tag8GrabBottomCone");

    Constants.autoSelect.addOption("Blue Middle Pass Line Balance", "Tag7BehindTheLineBalance");

    Constants.autoSelect.addOption("Blue Left Grab Top Cone", "Tag6GrabTopCone");


    
    

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
 
    AutoBalance = new AutoBalance(drivetrain, xbox1, 5); //Before .5 3/25/23

    playAudio = new PlayAudio(audio, 0, 0);
 
    //pathEQ = new PathEQ(Constants.testCoords, true);

    GoPastStartingLine = new DriveBackwardsToDistance(drivetrain, limelight, 3, .2);

    CenterToCubeNode = new CenterToDistance(drivetrain, limelight, 3, .1, .5);
 
    //Teleop commands
    driveWithXbox = new DriveWithXbox(drivetrain, limelight, xbox1, xbox2, false);
    slideWithXbox = new SlideWithXbox(xbox1, xbox2, slide);
 
    TopPosition = new GoToAngleAndExtension(slide, Constants.maxAngleEncoderValue, Constants.maxExtensionValue, 1, true);
    SubstationPosition = new GoToAngleAndExtension(slide, 15, Constants.maxExtensionValue, 1, false, 2.2);
    BottomPosition = new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, 2.2, 1, false, Constants.minExtensionValue);

    driveWithXbox.addRequirements(drivetrain);
    slideWithXbox.addRequirements(slide);
    AutoBalance.addRequirements(drivetrain);
    drivetrain.setDefaultCommand(driveWithXbox);
    slide.setDefaultCommand(slideWithXbox);

    recalibrateModules = new RecalibrateModules(drivetrain, xbox1);
    //recalibrateModules.addRequirements(drivetrain);
    //drivetrain.setDefaultCommand(recalibrateModules);
 
//---------------------------------
//Red Autos
//---------------------------------

    //Scores preload then grabs the bottom cone (does not score second cone)
    Tag1GrabBottomCone = new SequentialCommandGroup(new ResetPoseToLimelight(drivetrain, limelight, 0).withTimeout(.1),
      //Score preload 
      GenerateScoreHigh(),
      
      //Head to bottom cone
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false, 2.2),  
      new GoToFeederPosition(feeder, 0.5, FeederPosition.Cone),  
      GeneratePath(Paths.Tag1.GrabBottomCone.GridToBottomCone)),
      
      //Pick up bottom cone
      new ParallelCommandGroup(new RunFeeder(feeder, 0.5).withTimeout(2.5), GeneratePath(Paths.Tag1.GrabBottomCone.BottomConePickUp)),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false)),
      new GoToFeederPosition(feeder, 0.5, FeederPosition.Crush)    
    );

    //Scores preload then grabs the bottom cone (does not score second cone)
    Tag1GrabBottomCube = new SequentialCommandGroup(new ResetPoseToLimelight(drivetrain, limelight, 0).withTimeout(.1),
      //Score preload 
      GenerateScoreHigh(),
      
      //Head to bottom cone
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false, 2.2),  
      new GoToFeederPosition(feeder, 0.5, FeederPosition.Cube),  
      GeneratePath(Paths.Tag1.GrabBottomCone.GridToBottomCone)),
      
      //Pick up bottom cone
      new ParallelCommandGroup(new RunFeeder(feeder, 0.5).withTimeout(2), GeneratePath(Paths.Tag1.GrabBottomCone.BottomConePickUp)),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false))
    );

    //Places one cone on high pole and balances
    Tag2JustBalance = new SequentialCommandGroup(new ResetPoseToLimelight(drivetrain, limelight, 180).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, -17, Constants.minExtensionValue, 1, false), 
      GeneratePath(Paths.Tag2.JustBalance.GridToChargeStation)),
      GenerateAutoBalance()
    );

    //Places one cone on high pole, drives pass line, and balances. Does not use limelight readings
    Tag2BehindTheLineBalance = new SequentialCommandGroup(new ResetPose(drivetrain, 6.25, -1.2, 0).withTimeout(0.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, -17, Constants.minExtensionValue, 1, false), 
      GeneratePath(Paths.Tag2.BehindTheLineBalance.GridToOverChargeStation)),
      GeneratePath(Paths.Tag2.BehindTheLineBalance.BackUpOntoChargeStation),
      GenerateAutoBalance()
    );     

    //Scores preload the grabs the top cone
    Tag3GrabTopCone = new SequentialCommandGroup(new ResetPoseToLimelight(drivetrain, limelight, 0).withTimeout(.1),  
      //Score preload
      GenerateScoreHigh(),
      
      //Head to top cone
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false, 2.2),
      new GoToFeederPosition(feeder, 0.5, FeederPosition.Cone),  
      GeneratePath(Paths.Tag3.GrabTopCone.GridToTopCone)),
            
      //Pick up top cone
      new ParallelCommandGroup(new RunFeeder(feeder, 0.5).withTimeout(2), GeneratePath(Paths.Tag3.GrabTopCone.TopConePickUp)),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false)),
      new GoToFeederPosition(feeder, 0.5, FeederPosition.Crush)  
    );  

    //Scores preload the grabs the top cone
    StraightBackTest = new SequentialCommandGroup(new ResetPoseToLimelight(drivetrain, limelight, 0).withTimeout(.1),  
      GeneratePath(Paths.StraightBackTest)
    );  


//---------------------------------
//Blue Autos
//---------------------------------

    //Scores preload the grabs the top cone
    Tag6GrabTopCone = new SequentialCommandGroup(new ResetPoseToLimelight(drivetrain, limelight, 180).withTimeout(.1),  
      //Score preload
      GenerateScoreHigh(),
      
      //Head to top cone
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false, 2.2),
      new GoToFeederPosition(feeder, 0.5, FeederPosition.Cone),  
      GeneratePath(Paths.Tag6.GrabTopCone.GridToTopCone)),
            
      //Pick up top cone
      new ParallelCommandGroup(new RunFeeder(feeder, 0.5).withTimeout(2), GeneratePath(Paths.Tag6.GrabTopCone.TopConePickUp)),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false)),
      new GoToFeederPosition(feeder, 0.5, FeederPosition.Crush)
    
    );

    //Places one cone on high pole, drives pass line, and balances. Does not use limelight readings
    Tag7BehindTheLineBalance = new SequentialCommandGroup(new ResetPose(drivetrain, -6.25, -1.2, 180).withTimeout(0.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, -17, Constants.minExtensionValue, 1, false), 
      GeneratePath(Paths.Tag7.BehindTheLineBalance.GridToOverChargeStation)),
      GeneratePath(Paths.Tag7.BehindTheLineBalance.BackUpOntoChargeStation),
      GenerateAutoBalance()
    );     


    //Score preload, then grab bottom cone (doesn't score)
    Tag8GrabBottomCone = new SequentialCommandGroup(new ResetPoseToLimelight(drivetrain, limelight, 180).withTimeout(.1),
      //Score preload 
      GenerateScoreHigh(),
        
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false, 2.2),  
      GeneratePath(Paths.Tag8.GrabBottomCone.GridToBottomCone)),
      new GoToFeederPosition(feeder, -.5, FeederPosition.Cone).withTimeout(.5),
      
      new ParallelCommandGroup(new RunFeederContinously(feeder, .75), GeneratePath(Paths.Tag8.GrabBottomCone.GridToBottomCone).withTimeout(5)),
      new RunFeederContinously(feeder, 0),  
      new ParallelCommandGroup( new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false))
    );

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
    driver2B.onTrue(SubstationPosition);
    driver2A.onTrue(BottomPosition);
    driver2X.whileTrue(new KillArm(slide));

    //Feeder Positions
    driver1LB.onTrue(new GoToFeederPosition(feeder, 0.5, FeederPosition.Cube)); 
    driver1RB.onTrue(new GoToFeederPosition(feeder, 0.5, FeederPosition.Cone)); 
    driver1RS.onTrue(new GoToFeederPosition(feeder, 0.5, FeederPosition.Crush));

    driver1RT.whileTrue(new IntakeFeeder(feeder));
    driver1LT.whileTrue(new RunFeeder(feeder, -1));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (Constants.autoSelect.getSelected() == "Tag1GrabBottomCone") {

      return Tag1GrabBottomCone;
      
    } else if (Constants.autoSelect.getSelected() == "Tag2BehindTheLineBalance") {

      return Tag2BehindTheLineBalance;

    } else if (Constants.autoSelect.getSelected() == "Tag3GrabTopCone") {

      return Tag3GrabTopCone;

    } else if (Constants.autoSelect.getSelected() == "Tag6GrabTopCone") {

      return Tag6GrabTopCone;

    } else if (Constants.autoSelect.getSelected() == "Tag7BehindTheLineBalance") {

      return Tag7BehindTheLineBalance;

    } else if (Constants.autoSelect.getSelected() == "Tag8GrabBottomCone") {

      return Tag8GrabBottomCone;

    } else if (Constants.autoSelect.getSelected() == "AutoBalance") {

      return AutoBalance;

    }else if (Constants.autoSelect.getSelected() == "Nothing") {

      return null;

    }  else {

      return null;
      
    }
  }
}
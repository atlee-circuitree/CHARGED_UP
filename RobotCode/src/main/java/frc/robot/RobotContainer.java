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
import frc.robot.Constants.CoordsTags1and8;
import frc.robot.Constants.CoordsTags2and7;
import frc.robot.Constants.CoordsTags3and6;
import frc.robot.commands.AutoCommands.AngleAndExtendInAuto;
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
  private GoToAngleAndExtension MiddlePosition;
 
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

  SequentialCommandGroup GenerateScoreHigh(){

    return new SequentialCommandGroup(
      new ParallelCommandGroup(new RunFeeder(feeder, .3).withTimeout(.5), new GoToAngleAndExtension(slide, Constants.maxAngleEncoderValue, Constants.maxExtensionValue, 1, false)),
      new RunFeeder(feeder, -1).withTimeout(.5));

  }

  SequentialCommandGroup GenerateCollectPosition(){

    return new SequentialCommandGroup(new GoToAngleAndExtension(slide, 27, 2.2, 1, false, 2.2));
 
  }

  AutoBalance GenerateAutoBalance(){
    return new AutoBalance(drivetrain, xbox1, 5);
  }

  //Command Groups
  SequentialCommandGroup RedAuto;
  SequentialCommandGroup BlueAuto;
  SequentialCommandGroup TestRedAuto;
  SequentialCommandGroup TestBlueAuto;
  SequentialCommandGroup RedBalance;

  SequentialCommandGroup JustScore;
  SequentialCommandGroup JustScoreAndDriveBack;

  SequentialCommandGroup GoToBottom;
  SequentialCommandGroup RedOneConeLineTag1and8;
  SequentialCommandGroup RedTwoConeBalanceTag1and8;
  SequentialCommandGroup RedTwoConeCollectTag1and8;
  SequentialCommandGroup RedTwoConeCollectBalanceTag1and8;

  SequentialCommandGroup RedOneConeLineTag2and7;
  SequentialCommandGroup RedOneConeBalanceTag2and7;
  SequentialCommandGroup RedTwoConeBalanceTag2and7;
  SequentialCommandGroup RedTwoConeCollectTag2and7;
  SequentialCommandGroup RedTwoConeCollectBalanceTag2and7;

  SequentialCommandGroup RedOneConeLineTag3and6;
  SequentialCommandGroup RedTwoConeBalanceTag3and6;
  SequentialCommandGroup RedTwoConeCollectTag3and6;
  SequentialCommandGroup RedTwoConeCollectBalanceTag3and6;
  
  SequentialCommandGroup BlueOneConeLineTag1and8;
  SequentialCommandGroup BlueTwoConeChargeTag1and8;
  SequentialCommandGroup BlueTwoConeCollectTag1and8;
  SequentialCommandGroup BlueTwoConeCollectBalanceTag1and8;

  SequentialCommandGroup BlueOneConeLineTag2and7;
  SequentialCommandGroup BlueOneConeBalanceTag2and7;
  SequentialCommandGroup BlueTwoConeChargeTag2and7;
  SequentialCommandGroup BlueTwoConeCollectTag2and7;
  SequentialCommandGroup BlueTwoConeCollectBalanceTag2and7;

  SequentialCommandGroup BlueOneConeLineTag3and6;
  SequentialCommandGroup BlueTwoConeBalanceTag3and6;
  SequentialCommandGroup BlueTwoConeCollectTag3and6;
  SequentialCommandGroup BlueTwoConeCollectBalanceTag3and6;

 
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Constants.modeSelect = new SendableChooser<>();
    Constants.autoSelect = new SendableChooser<>();

    Constants.modeSelect.setDefaultOption("Competition", "Competition");
    Constants.autoSelect.addOption("Nothing", "Nothing");
    Constants.modeSelect.addOption("Testing", "Testing");
    Constants.modeSelect.addOption("Player_Two", "Player_Two");
    //Constants.autoSelect.setDefaultOption("Red Side", "Red Side");
    //Constants.autoSelect.addOption("Blue Side", "Blue Side");
    //Constants.autoSelect.addOption("TEST Red Side", "TEST Red Side");
    //Constants.autoSelect.addOption("TEST Blue Side", "TEST Blue Side");
    //Constants.autoSelect.addOption("Red Side Balance", "Red Side Balance");

    //Constants.autoSelect.addOption("AutoBalance", "AutoBalance");

    Constants.autoSelect.addOption("RedOneConeLineTag1and8", "RedOneConeLineTag1and8");
    //Constants.autoSelect.addOption("RedOneConeLineTag2and7", "RedOneConeLineTag2and7");
    Constants.autoSelect.addOption("RedOneConeLineTag3and6", "RedOneConeLineTag3and6");
    Constants.autoSelect.addOption("RedOneConeBalanceTag2and7", "RedOneConeBalanceTag2and7");

    //Constants.autoSelect.addOption("RedTwoConeChargeTag1and8", "RedTwoConeChargeTag1and8");
    //Constants.autoSelect.addOption("RedTwoConeCollectBalanceTag1and8", "RedTwoConeCollectBalanceTag1and8");

    //Constants.autoSelect.addOption("RedTwoConeCollectTag1and8", "RedTwoConeCollectTag1and8");

    //Constants.autoSelect.addOption("RedTwoConeChargeTag2and7", "RedTwoConeChargeTag2and7");
    //Constants.autoSelect.addOption("RedTwoConeCollectBalanceTag2and7", "RedTwoConeCollectBalanceTag2and7");
    //Constants.autoSelect.addOption("RedTwoConeCollectTag2and7", "RedTwoConeCollectTag2and7");
    //Constants.autoSelect.addOption("RedTwoConeChargeTag3and6", "RedTwoConeChargeTag3and6");
    //Constants.autoSelect.addOption("RedTwoConeCollectBalanceTag3and6", "RedTwoConeCollectBalanceTag3and6");
    //Constants.autoSelect.addOption("RedTwoConeCollectTag3and6", "RedTwoConeCollectTag3and6");

  /* 
    Constants.autoSelect.addOption("BlueOneConeLineTag1and8", "BlueOneConeLineTag1and8");
    Constants.autoSelect.addOption("BlueOneConeLineTag2and7", "BlueOneConeLineTag2and7");
    Constants.autoSelect.addOption("BlueOneConeLineTag3and6", "BlueOneConeLineTag3and6");
    Constants.autoSelect.addOption("BlueOneConeBalanceTag2and7", "BlueOneConeBalanceTag2and7");

    Constants.autoSelect.addOption("BlueTwoConeChargeTag1and8", "BlueTwoConeChargeTag1and8");
    Constants.autoSelect.addOption("BlueTwoConeCollectBalanceTag1and8", "BlueTwoConeCollectBalanceTag1and8");
    Constants.autoSelect.addOption("BlueTwoConeCollectTag1and8", "BlueTwoConeCollectTag1and8");
    Constants.autoSelect.addOption("BlueTwoConeChargeTag2and7", "BlueTwoConeChargeTag2and7");
    Constants.autoSelect.addOption("BlueTwoConeCollectBalanceTag2and7", "BlueTwoConeCollectBalanceTag2and7");
    Constants.autoSelect.addOption("BlueTwoConeCollectTag2and7", "BlueTwoConeCollectTag2and7");
    Constants.autoSelect.addOption("BlueTwoConeChargeTag3and6", "BlueTwoConeChargeTag3and6");
    Constants.autoSelect.addOption("BlueTwoConeCollectBalanceTag3and6", "BlueTwoConeCollectBalanceTag3and6");
    Constants.autoSelect.addOption("BlueTwoConeCollectTag3and6", "BlueTwoConeCollectTag3and6");

    Constants.autoSelect.addOption("JustScoreAndDriveBack", "JustScoreAndDriveBack");
    Constants.autoSelect.addOption("JustScore", "JustScore");
    Constants.autoSelect.addOption("Nothing", "Nothing");

    */

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
 
    TopPosition = new GoToAngleAndExtension(slide, 27.5, Constants.maxExtensionValue, 1, true);
    TopPositionAuto = new GoToAngleAndExtension(slide, Constants.maxExtensionValue, Constants.maxExtensionValue, 1, false, 2.2);
    MiddlePosition = new GoToAngleAndExtension(slide, 26.4, 29.5, 1, true, 2.2);

    
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
    AutoBalance.addRequirements(drivetrain);
    drivetrain.setDefaultCommand(driveWithXbox);
    slide.setDefaultCommand(slideWithXbox);

    recalibrateModules = new RecalibrateModules(drivetrain, xbox1);
    //recalibrateModules.addRequirements(drivetrain);
    //drivetrain.setDefaultCommand(recalibrateModules);
 
    //pathGenerator = new PathGenerator();

    // Auto Options
    PathFollower RedAutoPath = GeneratePath(Constants.redAuto);
    PathFollower BlueAutoPath = GeneratePath(Constants.blueAuto);

    PathFollower Cone1PickUp = GeneratePath(Constants.RedCone1PickUp);
    PathFollower Cone2PickUp = GeneratePath(Constants.RedCone2PickUp);
    PathFollower Cone3PickUp = GeneratePath(Constants.RedCone3PickUp);
    PathFollower Cone4PickUp = GeneratePath(Constants.RedCone4PickUp);

    PathFollower ScoringWpToConeWpTag3and6 = GeneratePath(Constants.RedScoringWpToConeWpTag3and6);
    PathFollower ConeWpToScoringWpTag3and6 = GeneratePath(Constants.RedConeWpToScoringWpTag3and6);
    PathFollower ScoringWpToConeWpTag2and7 = GeneratePath(Constants.RedScoringWpToConeWpTag2and7);
    PathFollower ConeWpToScoringWpTag2and7 = GeneratePath(Constants.RedConeWpToScoringWpTag2and7);
    PathFollower ScoringWpToConeWpTag1and8 = GeneratePath(Constants.RedScoringWpToConeWpTag1and8);
    PathFollower ConeWpToScoringWpTag1and8 = GeneratePath(Constants.RedConeWpToScoringWpTag1and8);

    PathFollower ScoringBalanceToBalanceWpTag1and8 = GeneratePath(Constants.RedScoringBalanceToBalanceWpTag1and8);
    PathFollower ConeBalanceToBalanceWpTag1and8 = GeneratePath(Constants.RedConeBalanceToBalanceWpTag1and8);
    PathFollower ScoringBalanceToBalanceWpTag2and7 = GeneratePath(Constants.RedScoringBalanceToBalanceWpTag2and7);
    PathFollower ConeBalanceToBalanceWpTag2and7 = GeneratePath(Constants.RedConeBalanceToBalanceWpTag2and7);
    PathFollower ScoringBalanceToBalanceWpTag3and6 = GeneratePath(Constants.RedScoringBalanceToBalanceWpTag3and6);
    PathFollower ConeBalanceToBalanceWpTag3and6 = GeneratePath(Constants.RedConeBalanceToBalanceWpTag3and6);
  
    PathFollower ScoreTag1and8South = GeneratePath(Constants.RedScoreTag1and8South);
    PathFollower ScoreTag1and8WestEast = GeneratePath(Constants.RedScoreTag1and8WestEast);
    PathFollower ScoreTag1and8North = GeneratePath(Constants.RedScoreTag1and8North);
    PathFollower ScoreTag2and7South = GeneratePath(Constants.RedScoreTag2and7South);
    PathFollower ScoreTag2and7WestEast = GeneratePath(Constants.RedScoreTag2and7WestEast);
    PathFollower ScoreTag2and7North = GeneratePath(Constants.RedScoreTag2and7North);
    PathFollower ScoreTag3and6South = GeneratePath(Constants.RedScoreTag3and6South);
    PathFollower ScoreTag3and6WestEast = GeneratePath(Constants.RedScoreTag3and6WestEast);
    PathFollower ScoreTag3and6North = GeneratePath(Constants.RedScoreTag3and6North);
    
    PathFollower BlueAutoTest = GeneratePath(Constants.redAuto);
    PathFollower BlueAutoBalancePath = GeneratePath(Constants.redAuto);
    PathFollower RedAutoBalancePath = GeneratePath(Constants.redBalance);
   
    //RedAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, 0.920, 0).withTimeout(.1), RedAutoPath);
   
    //TestRedAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, -0.07, 0).withTimeout(.1), RedAutoTestPath);   
    //TestRedAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -2.75, -1.041, 0).withTimeout(.1), Cone4PickUp);   
    //TestRedAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, -0.07, 0).withTimeout(.1), ScoringWpToConeWpTag3and6, Cone4PickUp);   
    TestRedAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      //GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, -17, Constants.minExtensionValue, 1, false), 
      GeneratePath(Constants.RedScoringWpToConeWpTag2and7)),
      GeneratePath(Constants.RedConeBalanceToBalanceWpTag2and7),
      GenerateAutoBalance()
    );  
    
    JustScoreAndDriveBack = new SequentialCommandGroup(
      //Add feeder open correct position eventually
      //new ResetPose(drivetrain, 0, 0, 0),
      //new RunFeeder(feeder, .2).withTimeout(0.5),
      //new GoToAngleAndExtension(slide, 31, Constants.maxExtensionValue, 1, false),
      //new RunFeeder(feeder, -.3).withTimeout(2),
      //new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),
      GeneratePath(Constants.JustScoreAndDriveBack)
    );

    JustScore = new SequentialCommandGroup(
      //Add feeder open correct position eventually
      new RunFeeder(feeder, .2).withTimeout(0.5),
      new GoToAngleAndExtension(slide, Constants.maxExtensionValue, Constants.maxExtensionValue, 1, false),
      new RunFeeder(feeder, -.3).withTimeout(2),
      new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false)
    );

    //Places one cone on high pole and drives pass line
    RedOneConeLineTag1and8 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags1and8.ScoreWestEast[0], -CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedScoringWpToConeWpTag1and8)),
      new GoToFeederPosition(feeder, -.5).withTimeout(2)
    );  

    //Places one cone on high pole and drives pass line
    RedOneConeLineTag2and7 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedScoringWpToConeWpTag2and7)),
      new GoToFeederPosition(feeder, -.5).withTimeout(2)
    );  
    
    //Places one cone on high pole and drives pass line
    RedOneConeLineTag3and6 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags3and6.ScoreWestEast[0], -CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedScoringWpToConeWpTag3and6)),
      new GoToFeederPosition(feeder, -.5).withTimeout(2)
    );  

    //Places one cone on high pole, drives pass line, and balances
    RedOneConeBalanceTag2and7 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, -17, Constants.minExtensionValue, 1, false), 
      GeneratePath(Constants.RedScoringWpToConeWpTag2and7)),
      GeneratePath(Constants.RedConeBalanceToBalanceWpTag2and7),
      GenerateAutoBalance()
    );   
    
    //Places one cone on first high pole, cycles for two cones and places on the rest of the high poles
    RedTwoConeCollectTag1and8 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags1and8.ScoreWestEast[0], -CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),

      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedScoringWpToConeWpTag1and8)),
      new GoToFeederPosition(feeder, -.2),
      new ParallelCommandGroup(new IntakeFeeder(feeder).withTimeout(5), GeneratePath(Constants.RedCone1PickUp)),
      new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedConeWpToScoringWpTag1and8),  
      GeneratePath(Constants.RedScoreTag1and8South),
      new GoToAngleAndExtension(slide, Constants.maxExtensionValue, Constants.maxExtensionValue, 1, false),
      new GoToFeederPosition(feeder, .2),
      
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false), 
      GeneratePath(Constants.RedScoringWpToConeWpTag1and8)),
      new ParallelCommandGroup(new IntakeFeeder(feeder).withTimeout(5), GeneratePath(Constants.RedCone2PickUp)),
      new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedConeWpToScoringWpTag1and8),  
      GeneratePath(Constants.RedScoreTag1and8North),
      new GoToAngleAndExtension(slide, Constants.maxExtensionValue, Constants.maxExtensionValue, 1, false),
      new GoToFeederPosition(feeder, .2),
      new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false)
    ); 

    //Places one cone on first high pole, cycles for two cones and places on the rest of the high poles
    RedTwoConeCollectTag3and6 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags3and6.ScoreWestEast[0], -CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1),  
      
      //Initial Score
      GenerateScoreHigh(),
      
      //Go to the first waypoint
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false, 2.2),  
      GeneratePath(Constants.RedScoringWpToConeWpTag3and6)),
      //GeneratePath(Constants.RedCone4), This is redundant, RedCone4PickUp below has the same coords as this

      //Run the feeder to collect the Cone
      new ParallelCommandGroup(new RunFeederContinously(feeder, .75), GeneratePath(Constants.RedCone4PickUp).withTimeout(5)),
      new RunFeederContinously(feeder, 0),  
      new ParallelCommandGroup( new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      //GeneratePath(Constants.RedConeWpToScoringWpTag3and6)),

      //Go to where we score the first cone
      GeneratePath(Constants.RedScoreTag3and6North)),
      new GoToAngleAndExtension(slide, Constants.maxAngleEncoderValue, Constants.maxExtensionValue, 1, false),
      new GoToFeederPosition(feeder, -.5).withTimeout(2),
      
      //Go to the second cone
      new GoToFeederPosition(feeder, .5).withTimeout(1),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false, 2.2), 
      GeneratePath(Constants.RedScoringWpToConeWpTag3and6), new GoToFeederPosition(feeder, -.2).withTimeout(.5)),
      GeneratePath(Constants.RedCone3),

      //Collect the second cone
      new ParallelCommandGroup(new RunFeederContinously(feeder, .6), GeneratePath(Constants.RedCone3PickUp).withTimeout(5)),
      new RunFeederContinously(feeder, 0),  
      new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),
      
      //Go to where we score the second cone
      GeneratePath(Constants.RedConeWpToScoringWpTag3and6),  
      GeneratePath(Constants.RedScoreTag3and6South),
      new GoToAngleAndExtension(slide, Constants.maxAngleEncoderValue, Constants.maxExtensionValue, 1, false),
      new WaitCommand(1),
      new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false)
      
    );  

    RedTwoConeCollectBalanceTag3and6 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags3and6.ScoreWestEast[0], -CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
    GenerateScoreHigh(),
    new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
    GeneratePath(Constants.RedScoringWpToConeWpTag3and6)),
    GeneratePath(Constants.RedCone4PickUp), 
    GeneratePath(Constants.RedConeWpToScoringWpTag3and6),  
    GeneratePath(Constants.RedScoreTag3and6North),  
    GeneratePath(Constants.RedScoringWpToConeWpTag3and6), 
    GeneratePath(Constants.RedCone3PickUp), 
    GeneratePath(Constants.RedConeBalanceToBalanceWpTag3and6)
  );   

    RedTwoConeBalanceTag3and6 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags3and6.ScoreWestEast[0], -CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
    GenerateScoreHigh(),
    new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false),  
    GeneratePath(Constants.RedScoringWpToConeWpTag3and6)),
    new GoToFeederPosition(feeder, -.2),
    new ParallelCommandGroup(new IntakeFeeder(feeder).withTimeout(5), GeneratePath(Constants.RedCone4PickUp)),
    new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
    GeneratePath(Constants.RedConeWpToScoringWpTag3and6),  
    GeneratePath(Constants.RedScoreTag3and6North),
    new GoToAngleAndExtension(slide, Constants.maxExtensionValue, Constants.maxExtensionValue, 1, false),
    new GoToFeederPosition(feeder, .2),
    new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false), 
    GeneratePath(Constants.RedScoringBalanceToBalanceWpTag3and6) 
  );  
 
    RedTwoConeBalanceTag1and8 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags1and8.ScoreWestEast[0], -CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedScoringWpToConeWpTag1and8)),
      GeneratePath(Constants.RedCone1PickUp), 
      GeneratePath(Constants.RedConeWpToScoringWpTag1and8), 
      GeneratePath(Constants.RedScoreTag1and8South),
      GeneratePath(Constants.RedScoringBalanceToBalanceWpTag1and8)
    ); 

    RedTwoConeCollectBalanceTag1and8 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags1and8.ScoreWestEast[0], -CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedScoringWpToConeWpTag1and8)),
      GeneratePath(Constants.RedCone1PickUp), 
      GeneratePath(Constants.RedConeWpToScoringWpTag1and8), 
      GeneratePath(Constants.RedScoreTag1and8South),
      GeneratePath(Constants.RedScoringWpToConeWpTag3and6),
      GeneratePath(Constants.RedCone2PickUp),
      GeneratePath(Constants.RedConeBalanceToBalanceWpTag1and8)
    );  

    RedTwoConeBalanceTag2and7 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedScoringWpToConeWpTag2and7)),
      GeneratePath(Constants.RedCone2PickUp), 
      GeneratePath(Constants.RedConeWpToScoringWpTag2and7), 
      GeneratePath(Constants.RedScoreTag2and7South), 
      GeneratePath(Constants.RedScoringBalanceToBalanceWpTag2and7)
    );   
    
    RedTwoConeCollectBalanceTag2and7 = new SequentialCommandGroup(new ResetPose(drivetrain, -CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -Constants.side))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.RedScoringWpToConeWpTag2and7)),
      GeneratePath(Constants.RedCone2PickUp), 
      GeneratePath(Constants.RedConeWpToScoringWpTag2and7), 
      GeneratePath(Constants.RedScoreTag2and7South),
      GeneratePath(Constants.RedScoringWpToConeWpTag2and7),
      GeneratePath(Constants.RedCone3PickUp),
      GeneratePath(Constants.RedConeBalanceToBalanceWpTag2and7)
    );     

//---------------------------------
//Blue
//---------------------------------

    //Places one cone on high pole, drives pass line, and balances
    BlueOneConeLineTag1and8 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags1and8.ScoreWestEast[0], -CoordsTags1and8.ScoreWestEast[1], 180).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.BlueScoringWpToConeWpTag1and8)),
      new GoToFeederPosition(feeder, -.5)
    );  

    //Places one cone on high pole, drives pass line, and balances
    BlueOneConeLineTag2and7 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.BlueScoringWpToConeWpTag2and7)),
      new GoToFeederPosition(feeder, -.5)
    );  
    
    //Places one cone on high pole, drives pass line, and balances
    BlueOneConeLineTag3and6 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags3and6.ScoreWestEast[0], -CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.BlueScoringWpToConeWpTag3and6)),
      new GoToFeederPosition(feeder, -.5)
    );  

    //Places one cone on high pole, drives pass line, and balances
    BlueOneConeBalanceTag2and7 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),          
      GeneratePath(Constants.BlueScoringWpToConeWpTag2and7)),
      GeneratePath(Constants.BlueScoringBalanceToBalanceWpTag2and7)
      //AutoBalance
    );  

    BlueTwoConeCollectTag1and8 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags1and8.ScoreWestEast[0], -CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      GeneratePath(Constants.BlueScoringWpToConeWpTag1and8), 
      GeneratePath(Constants.BlueCone1PickUp), 
      GeneratePath(Constants.BlueConeWpToScoringWpTag1and8), 
      GeneratePath(Constants.BlueScoreTag1and8South), 
      GeneratePath(Constants.BlueScoringWpToConeWpTag1and8),
      GeneratePath(Constants.BlueCone2PickUp)
    ); 

    BlueTwoConeCollectTag3and6 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags3and6.ScoreWestEast[0], -CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(), 
      GeneratePath(Constants.BlueScoringWpToConeWpTag3and6), 
      GeneratePath(Constants.BlueCone4PickUp), 
      GeneratePath(Constants.BlueConeWpToScoringWpTag3and6), 
      GeneratePath(Constants.BlueScoreTag3and6North),
      GeneratePath(Constants.BlueScoringWpToConeWpTag3and6),
      GeneratePath(Constants.BlueCone3PickUp)
    );    

    BlueTwoConeChargeTag1and8 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags1and8.ScoreWestEast[0], -CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      GeneratePath(Constants.BlueScoringWpToConeWpTag1and8), 
      GeneratePath(Constants.BlueCone1PickUp), 
      GeneratePath(Constants.BlueConeWpToScoringWpTag1and8), 
      GeneratePath(Constants.BlueScoreTag1and8South),
      GeneratePath(Constants.BlueScoringBalanceToBalanceWpTag1and8)
    ); 

    BlueTwoConeCollectBalanceTag1and8 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags1and8.ScoreWestEast[0], -CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      GeneratePath(Constants.BlueScoringWpToConeWpTag1and8), 
      GeneratePath(Constants.BlueCone1PickUp), 
      GeneratePath(Constants.RedConeWpToScoringWpTag1and8), 
      GeneratePath(Constants.BlueScoreTag1and8South),
      GeneratePath(Constants.BlueScoringWpToConeWpTag3and6),
      GeneratePath(Constants.BlueCone2PickUp),
      GeneratePath(Constants.BlueConeBalanceToBalanceWpTag1and8)
    );   

    BlueTwoConeChargeTag2and7 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      GeneratePath(Constants.BlueScoringWpToConeWpTag2and7), 
      GeneratePath(Constants.BlueCone2PickUp), 
      GeneratePath(Constants.BlueConeWpToScoringWpTag2and7), 
      GeneratePath(Constants.BlueScoreTag2and7South), 
      GeneratePath(Constants.BlueScoringBalanceToBalanceWpTag2and7)
    );

    BlueTwoConeCollectBalanceTag2and7 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags2and7.ScoreWestEast[0], -CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      GeneratePath(Constants.BlueScoringWpToConeWpTag2and7), 
      GeneratePath(Constants.BlueCone2PickUp), 
      GeneratePath(Constants.BlueConeWpToScoringWpTag2and7), 
      GeneratePath(Constants.BlueScoreTag2and7South),
      GeneratePath(Constants.BlueScoringWpToConeWpTag2and7),
      GeneratePath(Constants.BlueCone3PickUp),
      GeneratePath(Constants.BlueConeBalanceToBalanceWpTag2and7)
    ); 

    BlueTwoConeCollectBalanceTag3and6 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags3and6.ScoreWestEast[0], -CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      GeneratePath(Constants.BlueScoringWpToConeWpTag3and6),  
      GeneratePath(Constants.BlueCone4PickUp), 
      GeneratePath(Constants.BlueConeWpToScoringWpTag3and6),  
      GeneratePath(Constants.BlueScoreTag3and6North),  
      GeneratePath(Constants.BlueScoringWpToConeWpTag3and6), 
      GeneratePath(Constants.BlueCone3PickUp), 
      GeneratePath(Constants.BlueConeBalanceToBalanceWpTag3and6)
    );   

    BlueTwoConeBalanceTag3and6 = new SequentialCommandGroup(new ResetPose(drivetrain, CoordsTags3and6.ScoreWestEast[0], -CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * Constants.BLUE_SIDE))/2)).withTimeout(.1), 
      GenerateScoreHigh(),
      new ParallelCommandGroup(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.BlueScoringWpToConeWpTag3and6)),
      new GoToFeederPosition(feeder, -.2),
      new ParallelCommandGroup(new IntakeFeeder(feeder).withTimeout(5), GeneratePath(Constants.BlueCone4PickUp)),
      new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false),  
      GeneratePath(Constants.BlueConeWpToScoringWpTag3and6),  
      GeneratePath(Constants.BlueScoreTag3and6North),
      new GoToAngleAndExtension(slide, 31, Constants.maxExtensionValue, 1, false),
      new RunFeeder(feeder, -.2).withTimeout(1),
      new GoToAngleAndExtension(slide, 0, Constants.minExtensionValue, 1, false), 
      GeneratePath(Constants.BlueScoringBalanceToBalanceWpTag3and6) 
  );

    /*GoToBottom = new SequentialCommandGroup(new GoToAngleAndExtension(slide, 0, 0, 0, false));

    BlueAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, 0.920, 0).withTimeout(.1), BlueAutoPath);
    TestBlueAuto = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, 0.8, 0).withTimeout(.1), BlueAutoTest);
    RedBalance = new SequentialCommandGroup(new ResetPose(drivetrain, -6.495, 0.8, 0).withTimeout(.1), RedAutoBalancePath, new AutoBalance(drivetrain, xbox1, 5));
    */

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
    driver2A.onTrue(new GoToAngleAndExtension(slide, Constants.minAngleEncoderValue, 2.2, 1, false, 2.2));
    driver2X.whileTrue(new KillArm(slide));

    // Negative speed for cone, positive speed for cube
    driver2LB.onTrue(new GoToFeederPosition(feeder, -.5)); //Open for cube
    driver2RB.onTrue(new GoToFeederPosition(feeder, .5)); //Close for cone

    //driver1RT.whileTrue(new RunFeeder(feeder, 1));
    //driver1LT.whileTrue(new RunFeeder(feeder, -.6));

    driver1RT.whileTrue(new IntakeFeeder(feeder));
    driver1LT.whileTrue(new RunFeeder(feeder, -1));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (Constants.autoSelect.getSelected() == "Red Side") {

      return RedAuto;

    } else if (Constants.autoSelect.getSelected() == "Blue Side") {

      return BlueAuto;

    } else if (Constants.autoSelect.getSelected() == "TEST Red Side") {

      return TestRedAuto;

    } else if (Constants.autoSelect.getSelected() == "TEST Blue Side") {

      return TestBlueAuto;

    } else if (Constants.autoSelect.getSelected() == "Red Side Balance") {

      return RedBalance;

    } else if (Constants.autoSelect.getSelected() == "RedOneConeLineTag1and8") {

      return RedOneConeLineTag1and8;

    } else if (Constants.autoSelect.getSelected() == "RedOneConeLineTag2and7") {

      return RedOneConeLineTag2and7;
    
    } else if (Constants.autoSelect.getSelected() == "RedOneConeLineTag3and6") {

      return RedOneConeLineTag3and6;
    
    } else if (Constants.autoSelect.getSelected() == "RedOneConeBalanceTag2and7") {

      return RedOneConeBalanceTag2and7;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeChargeTag1and8") {

      return RedTwoConeBalanceTag1and8;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeCollectBalanceTag1and8") {

      return RedTwoConeCollectBalanceTag1and8;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeCollectTag1and8") {

      return RedTwoConeCollectTag1and8;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeChargeTag2and7") {

      return RedTwoConeBalanceTag2and7;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeCollectBalanceTag2and7") {

      return RedTwoConeCollectBalanceTag2and7;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeCollectTag2and7") {

      return RedTwoConeCollectTag2and7;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeChargeTag3and6") {

      return RedTwoConeBalanceTag3and6;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeCollectBalanceTag3and6") {

      return RedTwoConeCollectBalanceTag3and6;

    } else if (Constants.autoSelect.getSelected() == "RedTwoConeCollectTag3and6") {

      return RedTwoConeCollectTag3and6;

    } else if (Constants.autoSelect.getSelected() == "BlueOneConeLineTag1and8") {

      return BlueOneConeLineTag1and8;

    } else if (Constants.autoSelect.getSelected() == "BlueOneConeLineTag2and7") {

      return BlueOneConeLineTag2and7;

    } else if (Constants.autoSelect.getSelected() == "BlueOneConeLineTag3and6") {

      return BlueOneConeLineTag3and6;

    } else if (Constants.autoSelect.getSelected() == "BlueOneConeBalanceTag2and7") {

      return BlueOneConeBalanceTag2and7;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeChargeTag1and8") {

      return BlueTwoConeChargeTag1and8;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeCollectBalanceTag1and8") {

      return BlueTwoConeCollectBalanceTag1and8;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeCollectTag1and8") {

      return BlueTwoConeCollectTag1and8;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeChargeTag2and7") {

      return BlueTwoConeChargeTag2and7;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeCollectBalanceTag2and7") {

      return BlueTwoConeCollectBalanceTag2and7;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeCollectTag2and7") {

      return BlueTwoConeCollectTag2and7;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeChargeTag3and6") {

      return BlueTwoConeBalanceTag3and6;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeCollectBalanceTag3and6") {

      return BlueTwoConeCollectBalanceTag3and6;

    } else if (Constants.autoSelect.getSelected() == "BlueTwoConeCollectTag3and6") {

      return BlueTwoConeCollectTag3and6;

    } else if (Constants.autoSelect.getSelected() == "JustScore") {

      return JustScore;

    } else if (Constants.autoSelect.getSelected() == "AutoBalance") {

      return AutoBalance;

    }else if (Constants.autoSelect.getSelected() == "Nothing") {

      return null;

    }  else {

      return null;

    }
  }
}
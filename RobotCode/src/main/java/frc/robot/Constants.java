// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Limelight;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
 
public final class Constants {

    public static double side = 1;

    public DriverStation drive;

    public final class BlueAutoCoordsTag6 {
    public double[] blueStartingPosition_1 = {6.495 * side, 0.920, 0};
    public double[] moveToNorthConeWaypoint = {2.750, 1.041, 0};
    public double[] rotate180_1 = {2.750, 1.041, 180};
    public double[] PickUpConePosition_4 = {1.250, 1.041, 180};
    public double[] rotate180_2 = {1.250, 1.041, 0};
    public double[] moveToConeWayPoint = {2.750, 1.041, 0};
    public double[] moveToScoreWayPoint = {6.000, 1.041, 0};
    public double[] moveToScoringPosition_3 = {6.495, 1.428, 0};
    public double[] moveToScoreWayPoint_2 = {6.000, 1.041, 0};
    public double[] moveToNorthConeWayPoint = {2.750, 1.041, 0};
    public double[] rotate180_3 = {2.750, 1.041, 180};
    public double[] moveToFinalConeWayPoint = {2.000, -0.191, 180};
    public double[] moveToPickupCone_3 = {6.000, -0.191, 180};
    }
  
    public final static class BlueAutoCoordsTag8{
    public static final double[] blueStartingPosition_3 = {0, -6.495, -2.500, 0, 0.2};
    public static final double[] moveToSouthConeWayPoint = {1, -2.750, -2.600, 0, 0.2};
    public static final double[] rotate180_1 = {3, -2.750, -2.600, 180, 0.2};
    public static final double[] PickUpConePosition_1 = {4, -1.250, -2.600, 180, 0.2};
    public static final double[] rotate180_2 = {5, -1.250, -2.600, 0, 0.2};
    public static final double[] moveToConeWayPoint = {6, -2.750, -2.600, 0 , 0.2};
    public static final double[] moveToScoreWayPoint = {7, -6.000, -2.600, 0, 0.2};
    public static final double[] moveToSCoringPosition_1 = {8, -6.495, -2.500, 0, 0.2};
    //public static final double[]moveToScoreWayPoint_2 = 
    }

    public final static class BlueAutoCoordsTag7{
    public static final double[] blueStartingPosition_2 = {0, -6.495, -0.8, 0, 0.2};
    public static final double[] moveToScoringWayPoint = {1, -6.000, -0.8, 0, 0.2};
    public static final double[] moveToPlatformWayPoint_1 = {2, -4.400, -0.8, 0, 0.2};
    public static final double[] moveToMiddleConeWayPoint = {3, -2.750, -0.8, 0, 0.2};
    public static final double[] moveToPlatformWayPoint_2 = {4, -4.400, -0.8, 0, 0.2};
    }

    /*  public final static class RedAutoCoordsTag1{
    public static final double[] redStartingPosition_1 = {0, 6.495, -2.500, 0, 0.2};
    public static final double[] moveToSouthConeWayPoint_1 = {1, 2.750, -1.041, 0, 0.2};
    public static final double[] rotate180 = {2, 2.750, -1.041, 0, 0.2};
    public static final double[] cone4PickUpStart = {3, 1.750, 1.000, 180, 0.2};
    public static final double[] cone4PickUpEnd = {4, 1.250, 1.041, 180, 0.2};
    public static final double[] cone3PickUpStart = {5, 1.750, -0.200, 180, 0.2};
    public static final double[] cone3PickUpEnd = {6, 1.250, -0.200, 180, 0.2};
    public static final double[] cone2PickUpStart = {7, 1.750, -1.400, 180, 0.2};
    public static final double[] cone2PickUpEnd = {8, 1.250, -1.400, 180, 0.2};
    public static final double[] cone1PickUpStart = {9, 1.750, -2.630, 180, 0.2};
    public static final double[] cone1PickUpEnd = {10, 1.250, -2.630, 180, 0.2};
    public static final double[] rotate180_1 = {11, 1.250, -2.630, 0, 0.2};
    public static final double[] rotate180_2 = {12, 1.250, -1.400, 0, 0.2};
    public static final double[] rotate180_3 = {13, 1.250, -0.200, 180, 0.2};
    public static final double[] rotate180_4 = {14, 1.250, 1.041, 180, 0.2};
    public static final double[] moveToSouthConeWayPoint_2 = {15, 2.750, -1.041, 0, 0.2};
    public static final double[] moveToScoreWayPoint = {16, 6.000, -1.041, 0, 0.2};
    public static final double[] moveToScoringPosition_3 = {17, 6.495, -3.000,0, 0.2};
    public static final double[] moveToScoreWayPoint_2 = {18, 6.000, -1.041, 0, 0.2};
    public static final double[] moveToSouthConeWayPoint_3 = {19, 2.750, -1.041, 0, 0.2};
    public static final double[] rotate180_5 = {20, 2.750, 1.041, 180, 0.2};
    public static final double[] moveToFinalConeWayPoint = {21, 2.000, -0.191, 180, 0.2};
    public static final double[] moveToPickupCone_1 = {22, 1.750, -2.630, 180, 0.2};
    public static final double[] moveToPickupCone_2 = {23, 1.750, -1.400, 180, 0.2};
    public static final double[] moveToPickupCone_3 = {24, 1.750, -0.200, 180, 0.2};
    public static final double[] moveToPickupCone_4 = {25, 1.750, 1.000, 180, 0.2};
    }

    public final static class RedAutoCoordsTag2{
        public static final double[] redStartingPosition_2 = {0, 6.495, -0.8, 0, 0.2};
        public static final double[] moveToScoringWayPoint_1 = {1, 6.000, -0.8, 0, 0.2};
   // public static final double[] moveToPlatformWayPoint_1 = {2, 4.400, -0.8, 0, 0.2};
    public static final double[] moveToPlatformWayPoint_1 = {2, 3.700, -0.8, 0, 0.2};
    public static final double[] moveToMiddleConeWayPoint_1 = {3, 1.900, -0.8, 0, 0.2};
    public static final double[] rotate180 = {2, 1.900, -0.8, 180, 0.2};
    public static final double[] cone4PickUpStart = {3, 1.750, 1.000, 180, 0.2};
    public static final double[] cone4PickUpEnd = {4, 1.250, 1.041, 180, 0.2};
    public static final double[] cone3PickUpStart = {5, 1.750, -0.200, 180, 0.2};
    public static final double[] cone3PickUpEnd = {6, 1.250, -0.200, 180, 0.2};
    public static final double[] cone2PickUpStart = {7, 1.750, -1.400, 180, 0.2};
    public static final double[] cone2PickUpEnd = {8, 1.250, -1.400, 180, 0.2};
    public static final double[] cone1PickUpStart = {9, 1.750, -2.630, 180, 0.2};
    public static final double[] cone1PickUpEnd = {10, 1.250, -2.630, 180, 0.2};
    public static final double[] rotate180_1 = {11, 1.250, -2.630, 0, 0.2};
    public static final double[] rotate180_2 = {12, 1.250, -1.400, 0, 0.2};
    public static final double[] rotate180_3 = {13, 1.250, -0.200, 180, 0.2};
    public static final double[] rotate180_4 = {14, 1.250, 1.041, 180, 0.2};
    public static final double[] moveToPlatformWayPoint_2 = {4, 3.700, -0.8, 0, 0.2};
    public static final double[] moveToScoringWayPoint_2 = {1, 6.000, -0.8, 0, 0.2};
    public static final double[] moveToScoringposition = {0, 6.495, -0.8, 0, 0.2};
    public static final double[] moveToScoringWayPoint_3 = {18, 6.000, -0.8, 0, 0.2};
    public static final double[] moveToPlatformWayPoint_3 = {2, 3.700, -0.8, 0, 0.2};
    public static final double[] moveToMiddleConeWayPoint_2 = {3, 1.900, -0.8, 0, 0.2};
    public static final double[] rotate180_5 = {20, 2.750, -8, 180, 0.2};
    public static final double[] moveToMiddleConeWayPoint_3 = {21, 2.000, -0.8, 180, 0.2};
    public static final double[] moveToPickupCone_1 = {22, 1.750, -2.630, 180, 0.2};
    public static final double[] moveToPickupCone_2 = {23, 1.750, -1.400, 180, 0.2};
    public static final double[] moveToPickupCone_3 = {24, 1.750, -0.200, 180, 0.2};
    public static final double[] moveToPickupCone_4 = {25, 1.750, 1.000, 180, 0.2};
    }

    public final static class RedAutoCoordsTag3 {
      public static final double[] rotate180 = {2.750, 1.041, 180};
    public static final double[] cone4PickUpStart = {1.750, 1.000, 180};
    public static final double[] cone4PickUpEnd = {1.250, 1.041, 180};
    public static final double[] cone3PickUpStart = {1.750, -0.200,180};
    public static final double[] cone3PickUpEnd = {1.250, -0.200, 180};
    public static final double[] cone2PickUpStart = {1.750, -1.400,180};
    public static final double[] cone2PickUpEnd = {1.250, -1.400, 180};
    public static final double[] cone1PickUpStart = {1.750, -2.630,180};
    public static final double[] cone1PickUpEnd = {1.250, -2.630, 180};
    public static final double[] rotate180_1 =  {1.250, -2.630, 0, 0.2};
    public static final double[] rotate180_2 = {1.250, -1.400, 0, 0.2};
    public static final double[] rotate180_3 = {1.250, -0.200, 180, 0.2};
    public static final double[] rotate180_4 = {1.250, 1.041, 180, 0.2};
    public static final double[] moveToConeWayPoint = {2.750, 1.041, 0};
    public static final double[] moveToScoreWayPoint = {6.000, 1.041, 0};
    public static final double[] moveToScoringPosition_3 = {6.495, 1.428,0};
    public static final double[] moveToScoreWayPoint_2 = {6.000, 1.041, 0};
    public static final double[] moveToNorthConeWayPoint_2 = {2.750, 1.041, 0};
    public static final double[] rotate180_3 = {2.750, 1.041, 180, 0.2};
    public static final double[] moveToFinalConeWayPoint = {2.000, -0.191, 180};
    public static final double[] moveToPickupCone_1 = {1.750, -2.630, 180};
    public static final double[] moveToPickupCone_2 = {1.750, -1.400,180};
    public static final double[] moveToPickupCone_3 = {1.750, -0.200,180};
    public static final double[] moveToPickupCone_4 = {1.750, 1.041, 180};
    }
*/
    public final static class CoordsCones{
        public static final double[] cone4PickUpStart = {1.750, 1.041, 180};
        public static final double[] cone4PickUpEnd = {.75, 1.041, 180};
        public static final double[] cone3PickUpStart = {1.750, -0.200,180};
        public static final double[] cone3PickUpEnd = {1.250, -0.200, 180};
        public static final double[] cone2PickUpStart = {1.750, -1.400,180};
        public static final double[] cone2PickUpEnd = {1.250, -1.400, 180};
        public static final double[] cone1PickUpStart = {1.750, -2.630,180};
        public static final double[] cone1PickUpEnd = {1.250, -2.630, 180};
    }
    
    public final static class CoordsTags1and8{
        public static final double[] ScoreNorth = {6.495, -1.900, 0};
        public static final double[] ScoreWestEast = {6.495, -2.450, 0};
        public static final double[] ScoreSouth = {6.495, -3, 0};
        public static final double[] ScoreWayPoint = {6.000, -2.630, 0};
        public static final double[] MidWayPoint = {4.400, -2.630, 0};
        public static final double[] ConeWayPoint = {2.75, -2.630, 0};
        public static final double[] PlatformWayPoint = {4.400, -1.4, 0};
        public static final double[] ScorePlatformWayPoint = {6.000, -1.4, 0}; 
        public static final double[] ConePlatformWayPoint = {2.75, -1.4, 0}; 
    }
    
    public final static class CoordsTags2and7{
        public static final double[] ScoreNorth = {6.495, -0.250, 0};
        public static final double[] ScoreWestEast = {6.495, -0.8, 0};
        public static final double[] ScoreSouth = {6.495, -1.350, 0};
        public static final double[] ScoreWayPoint = {6.000, -0.8, 0};
        public static final double[] MidWayPoint = {4.400, -0.8, 0};      
        public static final double[] ConeWayPoint = {2.75, -0.8, 0};
        public static final double[] PlatformWayPoint = {4.400, -0.200, 0}; // Raw odemetry stop point 3.7
        public static final double[] ScorePlatformWayPoint = {6.000, -0.8, 0}; 
        public static final double[] ConePlatformWayPoint = {2.875, -0.8, 0}; //2.75 before 3/16/23
    }

    public final static class CoordsTags3and6 {
        public static final double[] ScoreNorth = {6.495, 1.450, 0};
        public static final double[] ScoreWestEast = {6.495, 0.900, 0};
        public static final double[] ScoreSouth = {6.495, 0.350, 0};
        public static final double[] ScoreWayPoint = {6.000, 1.041, 0};
        public static final double[] MidWayPoint = {4.400, 1.041, 0};
        public static final double[] ConeWayPoint = {2.75, 1.041, 0}; 
        public static final double[] PlatformWayPoint = {4.400, -0.200, 0}; 
        public static final double[] ScorePlatformWayPoint = {6.000, -0.200, 0}; 
        public static final double[] ConePlatformWayPoint = {2.75, -0.200, 0}; 
    }

    //Declare coordinates in the form {u, x, y, angle, tolerance}
    public static final double[][] autoCoordinates = {{0, 4,2 ,0, 0.2}, {1, 4.5,2 ,-90, 0.2}, {2, 5,2 ,180, 0.05}};
    
    //public static final double[][] testCoords = {{0, -6.445, 0.92, 0, .2}, {1, -2.214, 1.041, 0, .1}};
    public static final double[][] testCoords = {{0, 4.445, 1.92, 0, .025}, {1, 4.445, 1.92, 0, .025}};

    //Blue Auto
    public static final double[][] blueAuto = {{0, 6.495, 0.92, 0, 0.2}, {1, 2.750, 1.041, 0, 0.2}};
    //public static final double[][] blueAutoTEST = {BlueAutoCoordsTag7.blueStartingPosition_2, BlueAutoCoordsTag7.moveToScoringWayPoint, BlueAutoCoordsTag7.moveToPlatformWayPoint_1, BlueAutoCoordsTag7.moveToMiddleConeWayPoint, BlueAutoCoordsTag7.moveToPlatformWayPoint_2};
 
    //Auto Trajectories
    public static final double[][] redAuto = {{0, 6.495, 0.92, 0, 0.2}, {1, 6.495, 0.92, 0, 0.2}};
    public static final double[][] redAutoTEST = {
        {0, CoordsTags2and7.ScoreWayPoint[0] * side, CoordsTags2and7.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {1, CoordsTags2and7.MidWayPoint[0] * side, CoordsTags2and7.MidWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {2, CoordsTags2and7.ConeWayPoint[0] * side, CoordsTags2and7.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {3, CoordsTags2and7.PlatformWayPoint[0] * side,  CoordsTags2and7.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.2}
    };

    public static final double[][] redBalance = {
        {0, CoordsTags2and7.ScoreWayPoint[0] * side, CoordsTags2and7.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, 
        {1, CoordsTags2and7.MidWayPoint[0] * side, CoordsTags2and7.MidWayPoint[1], ((180 + (180 * -side))/2), 0.2 }
    };

    public static double[][] scoringWpToConeWpTag3and6 = {
        {0, CoordsTags3and6.ScoreWayPoint[0] * side, CoordsTags3and6.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags3and6.ConeWayPoint[0] * side, CoordsTags3and6.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2}
    };

    public static double[][] coneWpToScoringWpTag3and6 = {
        {0, CoordsTags3and6.ConeWayPoint[0] * side, CoordsTags3and6.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {1, CoordsTags3and6.ScoreWayPoint[0] * side, CoordsTags3and6.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };

    public static double[][] scoringBalanceToBalanceWpTag3and6 = {
        {0, CoordsTags3and6.ScorePlatformWayPoint[0] * side, CoordsTags3and6.ScorePlatformWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags3and6.PlatformWayPoint[0] * side, CoordsTags3and6.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}
    };

    public static double[][] coneBalanceToBalanceWpTag3and6 = {
        {0, CoordsTags3and6.ConePlatformWayPoint[0] * side, CoordsTags3and6.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {1, CoordsTags3and6.PlatformWayPoint[0] * side, CoordsTags3and6.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };

    public static double[][] scoringBalanceToBalanceWpTag2and7 = {
        {0, CoordsTags2and7.ScorePlatformWayPoint[0] * side, CoordsTags2and7.ScorePlatformWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags2and7.PlatformWayPoint[0] * side, CoordsTags2and7.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}
    };

    public static double[][] coneBalanceToBalanceWpTag2and7 = {
        {0, CoordsTags2and7.ConePlatformWayPoint[0] * side, CoordsTags2and7.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {1, CoordsTags2and7.PlatformWayPoint[0] * side, CoordsTags2and7.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };


    public static double[][] scoringBalanceToBalanceWpTag1and8 = {
        {0, CoordsTags3and6.ScorePlatformWayPoint[0] * side, CoordsTags3and6.ScorePlatformWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags3and6.PlatformWayPoint[0] * side, CoordsTags3and6.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}
    };

    public static double[][] coneBalanceToBalanceWpTag1and8 = {
        {0, CoordsTags1and8.ConePlatformWayPoint[0] * side, CoordsTags1and8.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {1, CoordsTags1and8.PlatformWayPoint[0] * side, CoordsTags1and8.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };



    public static double[][] scoringWpToConeWpTag2and7 = {
        {0, CoordsTags2and7.ScoreWayPoint[0] * side, CoordsTags2and7.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags2and7.ConeWayPoint[0] * side, CoordsTags2and7.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2}
    };

    public static double[][] coneWpToScoringWpTag2and7 = {
        {0, CoordsTags2and7.ConeWayPoint[0] * side, CoordsTags2and7.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {1, CoordsTags2and7.ScoreWayPoint[0] * side, CoordsTags2and7.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };

    public static double[][] scoringWpToConeWpTag1and8 = {
        {0, CoordsTags1and8.ScoreWayPoint[0] * side, CoordsTags1and8.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags1and8.ConeWayPoint[0] * side, CoordsTags1and8.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2}
    };

    public static double[][] coneWpToScoringWpTag1and8 = {
        {0, CoordsTags1and8.ConeWayPoint[0] * side, CoordsTags1and8.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {1, CoordsTags1and8.ScoreWayPoint[0] * side, CoordsTags1and8.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };


    public static double[][] cone1PickUp = {
        {0, CoordsCones.cone1PickUpStart[0] * side, CoordsCones.cone1PickUpStart[1], ((180 + (180 * side))/2), .05},  //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsCones.cone1PickUpEnd[0] * side, CoordsCones.cone1PickUpEnd[1], ((180 + (180 * side))/2), .05}
    };
    
    public static double[][] cone2PickUp = {
        {0, CoordsCones.cone2PickUpStart[0] * side, CoordsCones.cone2PickUpStart[1], ((180 + (180 * side))/2), .05},
        {1, CoordsCones.cone2PickUpEnd[0] * side, CoordsCones.cone2PickUpEnd[1], ((180 + (180 * side))/2), .05}
    };

    public static double[][] cone3PickUp = {
        {0, CoordsCones.cone3PickUpStart[0] * side, CoordsCones.cone3PickUpStart[1], ((180 + (180 * side))/2), .05},
        {1, CoordsCones.cone3PickUpEnd[0] * side, CoordsCones.cone3PickUpEnd[1], ((180 + (180 * side))/2), .05}
    };

    public static double[][] cone4PickUp = {
        {0, CoordsCones.cone4PickUpStart[0] * side, CoordsCones.cone4PickUpStart[1], ((180 + (180 * side))/2), .1},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsCones.cone4PickUpEnd[0] * side, CoordsCones.cone4PickUpEnd[1], ((180 + (180 * side))/2), .05}
    };
    
    public static double[][] scoreTag1and8South = {
        {0, CoordsTags1and8.ScoreSouth[0] * side, CoordsTags1and8.ScoreSouth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags1and8.ScoreSouth[0] * side, CoordsTags1and8.ScoreSouth[1], ((180 + (180 * -side))/2), .05}
    };
    public static double[][] scoreTag1and8WestEast = {
        {0, CoordsTags1and8.ScoreWestEast[0] * side, CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags1and8.ScoreWestEast[0] * side, CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * -side))/2), .05}
    };
    public static double[][] scoreTag1and8North = {
        {0, CoordsTags1and8.ScoreNorth[0] * side, CoordsTags1and8.ScoreNorth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags1and8.ScoreNorth[0] * side, CoordsTags1and8.ScoreNorth[1], ((180 + (180 * -side))/2), .05}
    };
    
    
    public static double[][] scoreTag2and7South = {
        {0, CoordsTags2and7.ScoreSouth[0] * side, CoordsTags2and7.ScoreSouth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags2and7.ScoreSouth[0] * side, CoordsTags2and7.ScoreSouth[1], ((180 + (180 * -side))/2), .05}
    };
    public static double[][] scoreTag2and7WestEast = {
        {0, CoordsTags2and7.ScoreWestEast[0] * side, CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags2and7.ScoreWestEast[0] * side, CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -side))/2), .05}
    };
    public static double[][] scoreTag2and7North = {
        {0, CoordsTags2and7.ScoreNorth[0] * side, CoordsTags2and7.ScoreNorth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags2and7.ScoreNorth[0] * side, CoordsTags2and7.ScoreNorth[1], ((180 + (180 * -side))/2), .05}
    };
    

    public static double[][] scoreTag3and6South = {
        {0, CoordsTags3and6.ScoreSouth[0] * side, CoordsTags3and6.ScoreSouth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags3and6.ScoreSouth[0] * side, CoordsTags3and6.ScoreSouth[1], ((180 + (180 * -side))/2), .05}
    };
    public static double[][] scoreTag3and6WestEast = {
        {0, CoordsTags3and6.ScoreWestEast[0] * side, CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags3and6.ScoreWestEast[0] * side, CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * -side))/2), .05}
    };
    public static double[][] scoreTag3and6North = {
        {0, CoordsTags3and6.ScoreNorth[0] * side, CoordsTags3and6.ScoreNorth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is 1 or 180 if side is 1
        {1, CoordsTags3and6.ScoreNorth[0] * side, CoordsTags3and6.ScoreNorth[1], ((180 + (180 * -side))/2), .05}
    };


    //Create Mode Select
    public static SendableChooser<String> modeSelect;
    public static SendableChooser<String> autoSelect;

    //Offsets and Zeroing
    public static String angleZeroKey = "AngleZero";
    public static double angleZeroValue = 0;

    //Ports
    public static final int frontLeftDrvMotorPort = 4;
    public static final int frontRightDrvMotorPort = 0;
    public static final int rearLeftDrvMotorPort = 6;
    public static final int rearRightDrvMotorPort = 8;

    public static final int frontLeftRotMotorPort = 3;
    public static final int frontRightRotMotorPort = 1;
    public static final int rearLeftRotMotorPort = 5;
    public static final int rearRightRotMotorPort = 7;

    public static final int frontLeftRotEncoderPort = 11;
    public static final int frontRightRotEncoderPort = 9;
    public static final int rearLeftRotEncoderPort = 12;
    public static final int rearRightRotEncoderPort = 10;

    public static final int leftExtMotorPort = 2;
    public static final int rightExtMotorPort = 13;
    public static final int angMotorPort = 15;
    public static final int angleEncoderChannel = 15;

   // public static final int clawMotorPort = 25;
  //  public static final int rotateClawMotorPort = 21;
    public static final int leftFeederMotorPort = 25;
    public static final int rightFeederMotorPort = 21;
    public static final int rotationFeederMotorPort = 26;
    public static final int rotationEncoderChannel = 5;
    public enum GrabPosition {

        Init,
        Open,
        Cone,
        Cube

    }

    //Encoder Values
    public static final double frontLeftEncoderOffset = 284.58984375;
    public static final double frontRightEncoderOffset = 254.267578125;
    public static final double rearLeftEncoderOffset = 127.353515625;
    public static final double rearRightEncoderOffset = 72.509765625;

    public static final int angleEncoderDIO = 0;
    public static final int feederRotationEncoderDIO = 1;
    public static final int extensionEncoderDIO = 9;
 
    public static final double angleOffset = -11.5;
    public static final double maxAngleEncoderValue = 36.2;
    public static final double minAngleEncoderValue = -14.0;
    //public static final double maxAngleEncoderValue = 9999;
    //public static final double minAngleEncoderValue = -9999;

    //public static final double maxRotationEncoderValue = 0.886;
    //public static final double minRotationEncoderValue = 0.348;

    public static final double maxCounterClockwiseRotationEncoderValue = 0.742;
    public static final double middleRotationEncoderValue = .732; //when claw is horizontal
    public static final double maxClockwiseRotationEncoderValue = 0.225;
   
    public static final double maxGrabEncoderValue = .7;
    public static final double minGrabEncoderValue = .40;
 
    public static final double maxExtensionValue = 52;
    public static final double minExtensionValue = 0;

    public static final double limelightSingleTargetPoseLengthCutoff = 1.9;

    public static final double trackwidth = 22.5;
    public static final double wheelbase = 22.5;

    //Distance from center of robot to any module
    public static final double drivetrainRadius = Math.sqrt(Math.pow(trackwidth, 2) + Math.pow(wheelbase, 2)); 

    public static final int xboxControllerPort = 0;

    //Phoenix's Falcon FX Motor Sensor Units Per Rotation
    public final static int kSensorUnitsPerRotation = 4096;

    //Number of rotations to drive when performing Distance Closed Loop
    public final static double kRotationsToTravel = 6;

    //Drive PIDs
    public static final double rotPID_P = 1;
    public static final double rotPID_I = 0;
    public static final double rotPID_D = 0.00;
    public static final double rotPIDMinValue = 0.07;

    //Auto PIDs
    public static final double xControllerP = 8;
    public static final double xControllerI = 2;
    public static final double xControllerD = 0;

    public static final double yControllerP = 8;
    public static final double yControllerI = 2;
    public static final double yControllerD = 0;

    public static final double zControllerProportion = 10;

    public static final double aBalanceXConstant = .2;
    public static final double aBalanceTurnConstant = .2;


    //Instansiated in this order:
    //FrontLeft, FrontRight, RearLeft, RearRight
    public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(-trackwidth / 2, wheelbase / 2),
            new Translation2d(trackwidth / 2, wheelbase / 2),
            new Translation2d(-trackwidth / 2, -wheelbase / 2),
            new Translation2d(trackwidth / 2, -wheelbase / 2));


    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);

    

    public static double smoothUsingError(double Encoder, double Range, double SetPoint, double MinSpeed, double MaxSpeed) {

        double error = (Math.abs(Encoder - SetPoint) / Range);

        if (error < MinSpeed) {

            error = MinSpeed;

        }
        
        if (error > MaxSpeed) {

            error = MaxSpeed;

        }

        return error;

    }
 
}
 
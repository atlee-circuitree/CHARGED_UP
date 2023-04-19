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
  
    //3 & 6 Score Wp = {6, 1.041, 0}
    //3 & 6 Cone Wp = {2.5, 1.041, 0}

    //2 & 7 Score Platform Wp = {6, -0.8, 0}
    //2 & 7 Platform Wp = {4.1, -0.8, 0}
    //2 & 7 Cone Wp = {1.8, -0.8, 0}

    //1 & 8 Score Wp = {6, -2.63, 0}
    //1 & 8 Cone Wp = {2.5, -2.63, 0}

    //Cone Coords Cone 1 Pickup Start = {2, -2.63, 180}
    //Cone Coords Cone 1 Pickup End = {0.8, -2.63, 180}

    //Cone Coords Cone 4 Pickup Start = {2, 1.065, 180}
    //Cone Coords Cone 4 Pickup End = {0.8, 1.065, 180}
 
    private static final double BLUE_SIDE = 1;

    private static final double BLUE_SIDE_Y = 1;

    public static double[][] RedScoringWpToConeWpTag3and6 = {
        //Start at the grid in front of Tag 3
        {0,     6, 1.041, 0,        0.3, 0.2},
        //Head to the top-most cone
        {1,     2.5, 1.041, 0,      0.3, 0.2}
    };

    public static double[][] RedScoringBalanceToBalanceWpTag2and7 = {
        //Start at the grid in front of Tag 2
        {0,     6, -0.8, 0,         0.3, 0.2},
        //Drive up onto the charge station
        {1,     4.1, -0.8, 0,       0.3, 0.05}
    };

    public static double[][] RedConeBalanceToBalanceWpTag2and7 = {
        //Start in front of the charge station near the cones
        {0,     1.8, -0.8, 0,       0.3, 0.2},
        //Drive up onto the charge station
        {1,     4.1, -0.8, 0,       0.3, 0.05}
    };

    public static double[][] RedScoringWpToConeWpTag2and7 = {
        //Start at the grid in front of Tag 2
        {0,     6, -0.8, 0,         0.3, 0.2},
        //Drive up over the charge station to the cone
        {1,     1.8, -0.8, 0,       0.3, 0.2}
    };

    public static double[][] RedScoringWpToConeWpTag1and8 = {
        //Start at the grid in front of Tag 1
        {0,     6, -2.63, 0,        0.3, 0.2},
        //Head to the bottom-most cone
        {1,     2.5, -2.63, 0,      0.3, 0.2} 
    };

    public static double[][] RedCone1PickUp = {
        //Start right behind the bottom-most cone facing the blue side
        {0,     2, -2.63, 180,      0.3, 0.2},
        //Drive through the cone and pick it up
        {1,     0.8, -2.63, 180,    0.3, 0.1}
    };

    public static double[][] RedCone4PickUp = {
        //Start right behind the top-most cone facing the blue side
        {0,     2, 1.065, 180,      0.3, 0.2},
        //Drive through the cone and pick it up
        {1,     0.8, 1.065, 180,    0.3, 0.1}
    };

    public static double side = 1;
 
    // OLD Cords
    public final class BlueAutoCoordsTag6 {
        public double[] blueStartingPosition_1 = {6.495, 0.920, 0};
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
    
        public final static class RedAutoCoordsTag1{
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
        //public static final double[] rotate180_3 = {1.250, -0.200, 180, 0.2};
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
    
        public final static class CoordsCones{
            public static final double[] cone4PickUpStart = {2.0, 1.065, 180};
            public static final double[] cone4PickUpEnd = {0.80, 1.065, 180};
            public static final double[] cone3PickUpStart = {2.0, -0.2,180};
            public static final double[] cone3PickUpEnd = {0.80, -0.2, 180};
            public static final double[] cone2PickUpStart = {2.0, -1.400,180};
            public static final double[] cone2PickUpEnd = {0.80, -1.400, 180};
            public static final double[] cone1PickUpStart = {2.0, -2.630,180};
            public static final double[] cone1PickUpEnd = {0.80, -2.630, 180};
        }
        
        public final static class CoordsTags1and8{
            public static final double[] ScoreNorth = {6.495, -1.900, 0};
            public static final double[] ScoreWestEast = {6.495, -2.450, 0};
            public static final double[] ScoreSouth = {6.495, -3, 0};
            public static final double[] ScoreWayPoint = {6.000, -2.630, 0};
            public static final double[] MidWayPoint = {4.400, -2.630, 0};
            public static final double[] ConeWayPoint = {2.5, -2.630, 0};
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
            public static final double[] ConeWayPoint = {1.8, -0.8, 0};
            public static final double[] PlatformWayPoint = {4.10, -0.800, 0}; //Before x = 4.7 3/18/23 1:00 PM
            public static final double[] ScorePlatformWayPoint = {6.000, -0.8, 0}; 
            public static final double[] ConePlatformWayPoint = {1.8, -0.8, 0}; //Before x = 2.75 3/16/23
        }
    
        public final static class CoordsTags3and6 {
            public static final double[] ScoreNorth = {6.495, 1.40, 0};
            public static final double[] ScoreWestEast = {6.495, 0.900, 0};
            public static final double[] ScoreSouth = {6.495, 0.345, 0};
            public static final double[] ScoreWayPoint = {6.000, 1.041, 0};
            public static final double[] MidWayPoint = {4.400, 1.041, 0};
            public static final double[] ConeWayPoint = {2.5, 1.041, 0}; 
            public static final double[] PlatformWayPoint = {4.40, -0.200, 0};
            public static final double[] ScorePlatformWayPoint = {6.000, -0.200, 0}; 
            public static final double[] ConePlatformWayPoint = {2.75, -0.200, 0}; 
        }

        public static double[][] RedScoreTag1and8South = {
            {0, CoordsTags1and8.ScoreSouth[0] * side, CoordsTags1and8.ScoreSouth[1], ((180 + (180 * -side))/2), .1},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsTags1and8.ScoreSouth[0] * side, CoordsTags1and8.ScoreSouth[1], ((180 + (180 * -side))/2), .05}
        };
        public static double[][] RedScoreTag1and8WestEast = {
            {0, CoordsTags1and8.ScoreWestEast[0] * side, CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * -side))/2), .1},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsTags1and8.ScoreWestEast[0] * side, CoordsTags1and8.ScoreWestEast[1], ((180 + (180 * -side))/2), .05}
        };
        public static double[][] RedScoreTag1and8North = {
            {0, CoordsTags1and8.ScoreNorth[0] * side, CoordsTags1and8.ScoreNorth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsTags1and8.ScoreNorth[0] * side, CoordsTags1and8.ScoreNorth[1], ((180 + (180 * -side))/2), .05}
        };
        
        
        public static double[][] RedScoreTag2and7South = {
            {0, CoordsTags2and7.ScoreSouth[0] * side, CoordsTags2and7.ScoreSouth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsTags2and7.ScoreSouth[0] * side, CoordsTags2and7.ScoreSouth[1], ((180 + (180 * -side))/2), .05}
        };
        public static double[][] RedScoreTag2and7WestEast = {
            {0, CoordsTags2and7.ScoreWestEast[0] * side, CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsTags2and7.ScoreWestEast[0] * side, CoordsTags2and7.ScoreWestEast[1], ((180 + (180 * -side))/2), .05}
        };
        public static double[][] RedScoreTag2and7North = {
            {0, CoordsTags2and7.ScoreNorth[0] * side, CoordsTags2and7.ScoreNorth[1], ((180 + (180 * -side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsTags2and7.ScoreNorth[0] * side, CoordsTags2and7.ScoreNorth[1], ((180 + (180 * -side))/2), .05}
        };
        public static double[][] BlueScoringWpToConeWpTag3and6 = {
            //Start at the grid in front of Tag 6
            {0,     -6, 1.041, 180,        0.3, 0.2},
            //Head to the top-most cone
            {1,     -2.5, 1.041, 180,        0.3, 0.2}
        };
        
        //Cone Coords Cone 1 Pickup Start = {2, -2.63, 180}
        //Cone Coords Cone 1 Pickup End = {0.8, -2.63, 180}
    
        public static double[][] RedScoringWpToConeWpTag3and6ButBetter = {
            {0, CoordsTags3and6.ScoreWayPoint[0] * side, CoordsTags3and6.ScoreWayPoint[1] + 0.15, 0, 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
            {1, CoordsTags3and6.ConeWayPoint[0] * side, CoordsTags3and6.ConeWayPoint[1] + 0.15, 0, 0.2}
        };
        //Cone Coords Cone 4 Pickup Start = {2, 1.065, 180}
        //Cone Coords Cone 4 Pickup End = {0.8, 1.065, 180}
        
    
        public static double[][] RedConeWpToScoringWpTag3and6 = {
            {0, CoordsTags3and6.ConeWayPoint[0] * side, CoordsTags3and6.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
            {1, CoordsTags3and6.ScoreWayPoint[0] * side, CoordsTags3and6.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        };
    
        public static double[][] RedScoringBalanceToBalanceWpTag3and6 = {
            {0, CoordsTags3and6.ScorePlatformWayPoint[0] * side, CoordsTags3and6.ScorePlatformWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
            {1, CoordsTags3and6.PlatformWayPoint[0] * side, CoordsTags3and6.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}
        };
    
        public static double[][] RedScoringBalanceToBalanceWpTag1and8 = {
            {0, CoordsTags1and8.ScorePlatformWayPoint[0] * side, CoordsTags1and8.ScorePlatformWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
            {1, CoordsTags1and8.PlatformWayPoint[0] * side, CoordsTags1and8.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}
        };
    
        public static double[][] RedConeBalanceToBalanceWpTag1and8 = {
            {0, CoordsTags1and8.ConePlatformWayPoint[0] * side, CoordsTags1and8.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
            {1, CoordsTags1and8.PlatformWayPoint[0] * side, CoordsTags1and8.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        };
    
        public static double[][] RedConeWpToScoringWpTag2and7 = {
            {0, CoordsTags2and7.ConeWayPoint[0] * side, CoordsTags2and7.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
            {1, CoordsTags2and7.ScoreWayPoint[0] * side, CoordsTags2and7.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
            //Start at the grid in front of Tag 2
            {0,     6, -0.8, 0,         0.3, 0.2},
            //Drive up over the charge station to the cone
            {1,     1.8, -0.8, 0,       0.3, 0.2}
        };
    
        public static double[][] RedScoringWpToConeWpTag1and8ButBetter = {
            {0, CoordsTags1and8.ScoreWayPoint[0] * side, CoordsTags1and8.ScoreWayPoint[1] - 0.15, 0, 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
            {1, CoordsTags1and8.ConeWayPoint[0] * side, CoordsTags1and8.ConeWayPoint[1] - 0.15, 0, 0.2}
 
        };
    
        public static double[][] RedConeWpToScoringWpTag1and8 = {
            {0, CoordsTags1and8.ConeWayPoint[0] * side, CoordsTags1and8.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
            {1, CoordsTags1and8.ScoreWayPoint[0] * side, CoordsTags1and8.ScoreWayPoint[1], ((180 + (180 * -side))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        };
    
        public static double[][] RedCone1 = {
            {0, CoordsCones.cone1PickUpStart[0] * side, CoordsCones.cone1PickUpStart[1], ((180 + (180 * side))/2), .2},  //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsCones.cone1PickUpStart[0] * side, CoordsCones.cone1PickUpStart[1], ((180 + (180 * side))/2), .1}
        };
        
        public static double[][] RedCone2 = {
            {0, CoordsCones.cone2PickUpStart[0] * side, CoordsCones.cone2PickUpStart[1], ((180 + (180 * side))/2), .2},
            {1, CoordsCones.cone2PickUpStart[0] * side, CoordsCones.cone2PickUpStart[1], ((180 + (180 * side))/2), .1}
        };
    
        public static double[][] RedCone3 = {
            {0, CoordsCones.cone3PickUpStart[0] * side, CoordsCones.cone3PickUpStart[1], ((180 + (180 * side))/2), .2},
            {1, CoordsCones.cone3PickUpStart[0] * side, CoordsCones.cone3PickUpStart[1], ((180 + (180 * side))/2), .1}
        };
    
        public static double[][] RedCone4 = {
            {0, CoordsCones.cone4PickUpStart[0] * side, CoordsCones.cone4PickUpStart[1], ((180 + (180 * side))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsCones.cone4PickUpEnd[0] * side, CoordsCones.cone4PickUpEnd[1], ((180 + (180 * side))/2), .1}
        };
        
        public static double[][] RedCone2PickUp = {
            {0, CoordsCones.cone2PickUpStart[0] * side, CoordsCones.cone2PickUpStart[1], ((180 + (180 * side))/2), .2},
            {1, CoordsCones.cone2PickUpEnd[0] * side, CoordsCones.cone2PickUpEnd[1], ((180 + (180 * side))/2), .1}
        };
    
        public static double[][] RedCone3PickUp = {
            {0, CoordsCones.cone3PickUpStart[0] * side, CoordsCones.cone3PickUpStart[1], 0, .2},
            {1, CoordsCones.cone3PickUpEnd[0] * side, CoordsCones.cone3PickUpEnd[1], 180, .1}
 
        };
    
        public static double[][] RedScoreTag3and6South = {
            {0, CoordsTags3and6.ScoreSouth[0] * side - 0.5, CoordsTags3and6.ScoreSouth[1], ((180 + (180 * -side))/2), .1},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsTags3and6.ScoreSouth[0] * side, CoordsTags3and6.ScoreSouth[1], ((180 + (180 * -side))/2), .1}
        };
        public static double[][] RedScoreTag3and6WestEast = {
            {0, CoordsTags3and6.ScoreWestEast[0] * side, CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * -side))/2), .1},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
            {1, CoordsTags3and6.ScoreWestEast[0] * side, CoordsTags3and6.ScoreWestEast[1], ((180 + (180 * -side))/2), .1}
        };
        public static double[][] RedScoreTag3and6North = {
            {0, CoordsTags3and6.ScoreNorth[0] * side - 0.5, CoordsTags3and6.ScoreNorth[1], ((180 + (180 * -side))/2), .1},   //((180 + (180 * side))/2) calculates and angle of zero if side is 1 or 180 if side is 1
            {1, CoordsTags3and6.ScoreNorth[0] * side, CoordsTags3and6.ScoreNorth[1], ((180 + (180 * -side))/2), .1}
        };
  
    //-----------------------------------------------------------------
    //Blue Coords
    //-----------------------------------------------------------------
 
    public static double[][] BlueScoringWpToConeWpTag3and6ButBetter = {
        {0, CoordsTags3and6.ScoreWayPoint[0] * BLUE_SIDE, CoordsTags3and6.ScoreWayPoint[1] * BLUE_SIDE_Y - 0.15, 0, 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags3and6.ConeWayPoint[0] * BLUE_SIDE, CoordsTags3and6.ConeWayPoint[1] * BLUE_SIDE_Y - 0.15, 0, 0.2}
    };
    
    public static double[][] BlueConeWpToScoringWpTag3and6 = {
        {0, CoordsTags3and6.ConeWayPoint[0] * BLUE_SIDE, CoordsTags3and6.ConeWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2},
        {1, CoordsTags3and6.ScoreWayPoint[0] * BLUE_SIDE, CoordsTags3and6.ScoreWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2}, //((180 + (180 * BLUE_SIDE))/2) calculates and angle of zero if BLUE_SIDE is 1 or 180 if side is -1
    };
    
    public static double[][] BlueScoringBalanceToBalanceWpTag3and6 = {
        {0, CoordsTags3and6.ScorePlatformWayPoint[0] * BLUE_SIDE, CoordsTags3and6.ScorePlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags3and6.PlatformWayPoint[0] * BLUE_SIDE, CoordsTags3and6.PlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.05}
    };
    
    public static double[][] BlueConeBalanceToBalanceWpTag3and6 = {
        {0, CoordsTags3and6.ConePlatformWayPoint[0] * BLUE_SIDE, CoordsTags3and6.ConeWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2},
        {1, CoordsTags3and6.PlatformWayPoint[0] * BLUE_SIDE, CoordsTags3and6.PlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.05}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };
    
    public static double[][] BlueScoringBalanceToBalanceWpTag2and7 = {
        {0, CoordsTags2and7.ScorePlatformWayPoint[0] * BLUE_SIDE, CoordsTags2and7.ScorePlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags2and7.PlatformWayPoint[0] * BLUE_SIDE, CoordsTags2and7.PlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.05}
    };
    
    public static double[][] BlueConeBalanceToBalanceWpTag2and7 = {
        {0, CoordsTags2and7.ConePlatformWayPoint[0] * BLUE_SIDE, CoordsTags2and7.ConeWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2},
        {1, CoordsTags2and7.PlatformWayPoint[0] * BLUE_SIDE, CoordsTags2and7.PlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.05}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };
    
    
    public static double[][] BlueScoringBalanceToBalanceWpTag1and8 = {
        {0, CoordsTags1and8.ScorePlatformWayPoint[0] * BLUE_SIDE, CoordsTags1and8.ScorePlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags1and8.PlatformWayPoint[0] * BLUE_SIDE, CoordsTags1and8.PlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.05}
    };
    
    public static double[][] BlueConeBalanceToBalanceWpTag1and8 = {
        {0, CoordsTags1and8.ConePlatformWayPoint[0] * BLUE_SIDE, CoordsTags1and8.ConeWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2},
        {1, CoordsTags1and8.PlatformWayPoint[0] * BLUE_SIDE, CoordsTags1and8.PlatformWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.05}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };
    
    
    
    public static double[][] BlueScoringWpToConeWpTag2and7 = {
        {0, CoordsTags2and7.ScoreWayPoint[0] * BLUE_SIDE, CoordsTags2and7.ScoreWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags2and7.ConeWayPoint[0] * BLUE_SIDE, CoordsTags2and7.ConeWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2}
    };
    
    public static double[][] BlueConeWpToScoringWpTag2and7 = {
        {0, CoordsTags2and7.ConeWayPoint[0] * BLUE_SIDE, CoordsTags2and7.ConeWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2},
        {1, CoordsTags2and7.ScoreWayPoint[0] * BLUE_SIDE, CoordsTags2and7.ScoreWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };
    
    public static double[][] BlueScoringWpToConeWpTag1and8 = {
        {0, CoordsTags1and8.ScoreWayPoint[0] * BLUE_SIDE, CoordsTags1and8.ScoreWayPoint[1] * BLUE_SIDE_Y, 180, 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags1and8.ConeWayPoint[0] * BLUE_SIDE, CoordsTags1and8.ConeWayPoint[1] * BLUE_SIDE_Y, 180, 0.2}
    };
    
    public static double[][] BlueScoringWpToConeWpTag1and8ButBetter = {
        {0, CoordsTags1and8.ScoreWayPoint[0] * BLUE_SIDE, CoordsTags1and8.ScoreWayPoint[1] * BLUE_SIDE_Y + 0.15, 0, 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
        {1, CoordsTags1and8.ConeWayPoint[0] * BLUE_SIDE, CoordsTags1and8.ConeWayPoint[1] * BLUE_SIDE_Y + 0.15, 0, 0.2}
    };
    
    public static double[][] BlueConeWpToScoringWpTag1and8 = {
        {0, CoordsTags1and8.ConeWayPoint[0] * BLUE_SIDE, CoordsTags1and8.ConeWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2},
        {1, CoordsTags1and8.ScoreWayPoint[0] * BLUE_SIDE, CoordsTags1and8.ScoreWayPoint[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), 0.2}, //((180 + (180 * -side))/2) calculates and angle of zero if side is 1 or 180 if side is -1
    };
    
    
    public static double[][] BlueCone1PickUp = {
        {0, CoordsCones.cone1PickUpStart[0] * BLUE_SIDE, CoordsCones.cone1PickUpStart[1] * BLUE_SIDE_Y, 0, .05},  //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsCones.cone1PickUpEnd[0] * BLUE_SIDE, CoordsCones.cone1PickUpEnd[1] * BLUE_SIDE_Y, 180, .05}
    };
    
    public static double[][] BlueCone2PickUp = {
        {0, CoordsCones.cone2PickUpStart[0] * BLUE_SIDE, CoordsCones.cone2PickUpStart[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05},
        {1, CoordsCones.cone2PickUpEnd[0] * BLUE_SIDE, CoordsCones.cone2PickUpEnd[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    
    public static double[][] BlueCone3PickUp = {
        {0, CoordsCones.cone3PickUpStart[0] * BLUE_SIDE, CoordsCones.cone3PickUpStart[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05},
        {1, CoordsCones.cone3PickUpEnd[0] * BLUE_SIDE, CoordsCones.cone3PickUpEnd[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    
    public static double[][] BlueCone4PickUp = {
        {0, CoordsCones.cone4PickUpStart[0] * BLUE_SIDE, CoordsCones.cone4PickUpStart[1] * BLUE_SIDE_Y, 0, .1},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsCones.cone4PickUpEnd[0] * BLUE_SIDE, CoordsCones.cone4PickUpEnd[1] * BLUE_SIDE_Y, 180, .05}
    };
    
    public static double[][] BlueScoreTag1and8South = {
        {0, CoordsTags1and8.ScoreSouth[0] * BLUE_SIDE, CoordsTags1and8.ScoreSouth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags1and8.ScoreSouth[0] * BLUE_SIDE, CoordsTags1and8.ScoreSouth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    public static double[][] BlueScoreTag1and8WestEast = {
        {0, CoordsTags1and8.ScoreWestEast[0] * BLUE_SIDE, CoordsTags1and8.ScoreWestEast[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags1and8.ScoreWestEast[0] * BLUE_SIDE, CoordsTags1and8.ScoreWestEast[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    public static double[][] BlueScoreTag1and8North = {
        {0, CoordsTags1and8.ScoreNorth[0] * BLUE_SIDE, CoordsTags1and8.ScoreNorth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags1and8.ScoreNorth[0] * BLUE_SIDE, CoordsTags1and8.ScoreNorth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    
    
    public static double[][] BlueScoreTag2and7South = {
        {0, CoordsTags2and7.ScoreSouth[0] * BLUE_SIDE, CoordsTags2and7.ScoreSouth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags2and7.ScoreSouth[0] * BLUE_SIDE, CoordsTags2and7.ScoreSouth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    public static double[][] BlueScoreTag2and7WestEast = {
        {0, CoordsTags2and7.ScoreWestEast[0] * BLUE_SIDE, CoordsTags2and7.ScoreWestEast[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags2and7.ScoreWestEast[0] * BLUE_SIDE, CoordsTags2and7.ScoreWestEast[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    public static double[][] BlueScoreTag2and7North = {
        {0, CoordsTags2and7.ScoreNorth[0] * BLUE_SIDE, CoordsTags2and7.ScoreNorth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags2and7.ScoreNorth[0] * BLUE_SIDE, CoordsTags2and7.ScoreNorth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    
    
    public static double[][] BlueScoreTag3and6South = {
        {0, CoordsTags3and6.ScoreSouth[0] * BLUE_SIDE, CoordsTags3and6.ScoreSouth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags3and6.ScoreSouth[0] * BLUE_SIDE, CoordsTags3and6.ScoreSouth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    public static double[][] BlueScoreTag3and6WestEast = {
        {0, CoordsTags3and6.ScoreWestEast[0] * BLUE_SIDE, CoordsTags3and6.ScoreWestEast[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is -1 or 180 if side is 1
        {1, CoordsTags3and6.ScoreWestEast[0] * BLUE_SIDE, CoordsTags3and6.ScoreWestEast[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
    public static double[][] BlueScoreTag3and6North = {
        {0, CoordsTags3and6.ScoreNorth[0] * BLUE_SIDE, CoordsTags3and6.ScoreNorth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .2},   //((180 + (180 * side))/2) calculates and angle of zero if side is 1 or 180 if side is 1
        {1, CoordsTags3and6.ScoreNorth[0] * BLUE_SIDE, CoordsTags3and6.ScoreNorth[1] * BLUE_SIDE_Y, ((180 + (180 * BLUE_SIDE))/2), .05}
    };
 
    public static double[][] RedConeBalanceToBalanceWpTag3and6 = {
        {0, CoordsTags3and6.ConePlatformWayPoint[0] * side, CoordsTags3and6.ConeWayPoint[1], ((180 + (180 * -side))/2), 0.2},
        {1, CoordsTags3and6.PlatformWayPoint[0] * side, CoordsTags3and6.PlatformWayPoint[1], ((180 + (180 * -side))/2), 0.05}, //((180 + (180 * -side))/2) c
    };

    public static final double CRUSH_POSITION = .57;
    public static final double CONE_POSITION = .75;
    public static final double CUBE_POSITION = .93;

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
    public enum FeederPosition {

        Crush,
        Cone,
        Cube

    }

    //Encoder Values
    public static final double frontLeftEncoderOffset = 282.568;
    public static final double frontRightEncoderOffset = 251.806;
    public static final double rearLeftEncoderOffset = 128.759;
    public static final double rearRightEncoderOffset = 75.146;

    public static final int angleEncoderDIO = 0;
    public static final int feederRotationEncoderDIO = 1;
    public static final int extensionEncoderDIO = 9;
 
    public static final double angleOffset = -4.5; //Before -11.5
    public static final double maxAngleEncoderValue = 22.1; 
    public static final double minAngleEncoderValue = -29; 

    public static final double maxCounterClockwiseRotationEncoderValue = 0.742;
    public static final double middleRotationEncoderValue = .732; //when claw is horizontal
    public static final double maxClockwiseRotationEncoderValue = 0.225;
   
    public static final double maxGrabEncoderValue = .7;
    public static final double minGrabEncoderValue = .40;
 
    public static final double maxExtensionValue = 52;
    public static final double minExtensionValue = -1; //Before -2 3/21/23

    public static final double limelightSingleTargetPoseLengthCutoff = 1.2;
    public static final double limelightMultiTargetPoseLengthCutoff = 5.5;
    public static final double rearlimelightMultiTargetPoseLengthCutoff = 5.8;

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

    public static final double zControllerProportion = 20;

    public static final double aBalanceXConstant = .58;
    


    //Instansiated in this order
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

}
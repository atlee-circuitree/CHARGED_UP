
package frc.robot;


/**
 * This class holds all of the auto paths
 * 
 * Try not to use it for anything else
 */
 
public class Paths {

    ///////////////////////////GENERAL NOTES PLEASE READ/////////////////////////////////
    
    //Declare coordinates in the form {u, x, y, angle, speed, tolerance}

    //Example test path (drive +x 1m and rotate 180)   
    public static final double[][] autoCoordinates = {
        //Start in front of April Tag 8.5
        {0,     0, 0, 0,         0.3, 0.2},
        //Go back behind the potential omnimeter
        {1,     0.5, 0, 90,      0.3, 0.2},
        //Finish at the sustainiable energy trapezoid lauch location facing the blue side 
        {2,     1, 0, 180,       0.3, 0.05}
    };

    //Tab-ing out around the x/y/z is encouraged (see above path), as it keeps it clean and easy to modify in a pinch
    
    //Also, PLEASE COMMENT WHAT EVERY COORDINATE ACTUALLY REPRESENTS
    //If the coordinate is taking the robot under the "STEM sprinkler" or into the "Neutral zone", make sure to write that down!
    //Again, see the above path for an example
    
    //Paths are stored based on:
    // 1) The April tag they start in front of
    // 2) What they actually do


    //Odometry (last updated 2023, update as needed) has its orgin at the center of the field
    //X and Y are layed out like a regular coordinate plane, and 0 degrees is pointing towards the red side
    
    //Amazingly detailed field diagram below vvv
    /* 
     * |-------------------------------------------|
     * |                     |                     |
     * |B        (-X,+Y)     |     (+X,+Y)        R|
     * |L -------------------|------------------- E|
     * |U        (-X,-Y)     |     (+X,-Y)        D|
     * |E                    |                     |
     * |-------------------------------------------|
    */

    //I haven't tested the max amount of coordinates possible per path, so try that at your own risk
    //Minimum coordinates per path is 2

    //Good luck!

    //////////////////////////CHARGED UP SPECIFIC NOTES////////////////////////////////////////

    //In case you forgot - 
    
    //Grid = the scoring area with the poles and ledges
    //Charge station = the giant tilting platform
    //Community = the taped-off area between the grid and the charge station
    //Substation = where the human player drops the cones/cubes in

    //Top cone = the cone closest to the substation, has the highest Y-value of all the cones
    //Top middle cone = one cone down from the top cone, closest to the centerline of the field
    //Bottom middle cone = one cone up from the bottom cone
    //Bottom cone = The cone with the lowest Y-value

    //April tag locations from top (highest Y) to bottom (lowest Y)

    //Tags 4 (red) and 5 (blue) <-- On substation, also higher up than the rest of them
    //Tags 3 (red) and 6 (blue) ]
    //Tags 2 (red) and 7 (blue) ] <-- On grid
    //Tags 1 (red) and 8 (blue) ]
    
    //"Scores preload" is assumed to be a cube on the to ledge unless otherwise noted
    
    //////////////////////////END OF NOTES/////////////////////////////////////////////////////


    //Please change the y to the tag you're starting at
    public static final double[][] StraightBackTest = {
        {0,     6, -3, 0,        0.6, 0.2},
        {1,     3.5, -3, 180,    0.3, 0.2},
        
    };
 
    //All the paths that start in front of Tag 1
    public static final class Tag1{

        //Scores preload, drives out of community, grabs bottom cone
        public static final class GrabBottomCone{

            public static final double[][] GridToBottomCone = {
                //Start at the grid in front of Tag 1
                {0,     6, -3, 0,        0.6, 0.2},
                //Start to rotate
                {1,     3.5, -3, 0,      0.6, 0.2},
                //Head to the bottom-most cone
                {2,     2, -3, 160,      0.3, 0.2}
            };

            public static final double[][] BottomConePickUp = {
                //Start right behind the bottom-most cone facing the blue side
                {0,     1, -3, 180,    0.15, 0.01},
                //Drive through the cone and pick it up
                {1,     0.8, -3, 180,    0.15, 0.01}
            };

        }
        
        //Scores preload, drives out of community, grabs bottom middle cone
        public static final class GrabBottomMiddleCone{

            //Wow great paths man, very cool

        }
    }

    //All the paths that start in front of Tag 2
    public static final class Tag2{

        //Scores preload, balances on charge station
        public static final class JustBalance{
            
            public static final double[][] GridToChargeStation = {
                //Start at the grid in front of Tag 2
                {0,     6, -1.3, 0,         0.3, 0.2},
                //Drive up onto the charge station
                {1,     4.1, -1.3, 0,       0.3, 0.05}
            };

        }

        //Scores preload, drives over charge station out of community, balances on charge station
        public static final class BehindTheLineBalance{
            
            public static final double[][] GridToOverChargeStation = {
                //Start at the grid in front of Tag 2
                {0,     6, -1.3, 0,         0.3, 0.2},
                //Drive up over the charge station and cross the tape line
                {1,     1.8, -1.3, 0,       0.3, 0.2}
            };

            public static final double[][] BackUpOntoChargeStation = {
                //Start in front of the charge station near the cones
                {0,     1.8, -1.3, 0,       0.3, 0.2},
                //Drive up onto the charge station
                {1,     4.1, -1.3, 0,       0.3, 0.05}
            };

        }
    }

    //All the paths that start in front of Tag 3
    public static final class Tag3{

        //Score preload, drive out of community, grabs top cone
        public static final class GrabTopCone{

            public static final double[][] GridToTopCone = {
                //Start at the grid in front of Tag 3
                {0,     6, 0.92, 0,        0.6, 0.2},
                //Start to rotate
                {1,     5.5, 0.92, 0,      0.6, 0.2},
                //Head to the top-most cone
                {2,     2, 0.92, -160,     0.3, 0.2}
            };

            public static final double[][] TopConePickUp = {
                //Start right behind the top-most cone facing the blue side
                {0,     1, 0.92, 180,      0.15, 0.01},
                //Drive through the cone and pick it up
                {1,     0.8, 0.92, 180,    0.15, 0.01}
            };

            public static final double[][] TopConePickUpToGrid = {
                //Start to drive from TopConePickUp to grid
                {0,     2, 0.92, 180,      0.3, 0.2 },
                //Drives up to and rotates to right grid
                {1,     6, 0.92, 0,        0.15, 0.01},
            };

        }

        //Score preload, drive out of community, grabs top-middle cone
        public static final class GrabTopMiddleCone{

            public static final double[][] GridToTopMiddleCone = {
                //Start at grid in front of grid in front of Tag 3
                {0,     6, 0.88, 0,         0.3, 0.2},
                //Go past the charge station and out of the community
                {1,     2.94, 1, 0,         0.3, 0.2},
                //Go to the top middle cone and face the blue side
                {2,     1.8, -0.17, 180,    0.3, 0.2}
            };

            public static final double[][] PickUpTopMiddleCone = {
                //Start behind top middle cone facing blue side
                {0,     1.8, -0.17, 180,    0.3, 0.2},
                //Drive through top middle cone to pick it up
                {1,     0.80, -0.17, 180,   0.15, 0.01}
            };

        } 
    }
    

    //All the paths that start in front of Tag 6
    public static final class Tag6{

        //Score preload, drive out of community, grabs top cone
        public static final class GrabTopCone{

            public static final double[][] GridToTopCone = {
                //Start at the grid in front of Tag 1
                {0,     -6, 3, 180,        0.6, 0.2},
                //Start to rotate
                {1,     -3.5, 3, 180,      0.6, 0.2},
                //Head to the bottom-most cone
                {2,     -2, 3, -20,         0.3, 0.2}
            };

            public static final double[][] TopConePickUp = {
                //Start right behind the bottom-most cone facing the blue side
                {0,     -1, 3, 0,          0.15, 0.01},
                //Drive through the cone and pick it up
                {1,     -0.8, 3, 0,        0.15, 0.01}
            };

            public static final double[][] TopConePickUpToGrid = {
                //Start to drive from TopConePickUp to grid
                {0,     -2, 3, 0,          0.3, 0.2 },
                //Drives up to and rotates to right grid
                {1,     -6, 3, 180,        0.15, 0.01},
            };

            

        }
    }


    //All the paths that start in front of Tag 7
    public static final class Tag7{

        //Scores preload, balances on charge station
        public static final class JustBalance{
            
            public static final double[][] GridToChargeStation = {
                //Start at the grid in front of Tag 7
                {0,     -6, -1.3, 180,         0.3, 0.2},
                //Drive up onto the charge station
                {1,     -4.1, -1.3, 180,       0.3, 0.05}
            };

        }

        //Scores preload, drives over charge station out of community, balances on charge station
        public static final class BehindTheLineBalance{
            
            public static final double[][] GridToOverChargeStation = {
                //Start at the grid in front of Tag 7
                {0,     -6, -1.3, 180,         0.3, 0.2},
                //Drive up over the charge station and cross the tape line
                {1,     -1.8, -1.3, 180,       0.3, 0.2}
            };

            public static final double[][] BackUpOntoChargeStation = {
                //Start in front of the charge station near the cones
                {0,     -1.8, -1.3, 180,       0.3, 0.2},
                //Drive up onto the charge station
                {1,     -4.1, -1.3, 180,       0.3, 0.05}
            };

        }
    }


    //All the paths that start in front of Tag 8
    public static final class Tag8{

        //Scores preload, drives out of community, grabs bottom cone
        public static final class GrabBottomCone{

            public static final double[][] GridToBottomCone = {
                //Start at the grid in front of Tag 3
                {0,     -6, -0.92, 180,        0.6, 0.2},
                //Start to rotate
                {1,     -5.5, -0.92, 180,      0.6, 0.2},
                //Head to the top-most cone
                {2,     -2, -0.92, 20,         0.3, 0.2}
            };

            public static final double[][] BottomConePickUp = {
                //Start right behind the top-most cone facing the blue side
                {0,     -1, -0.92, 0,          0.15, 0.01},
                //Drive through the cone and pick it up
                {1,     -0.8, -0.92, 0,        0.15, 0.01}
            };

            public static final double[][] BottomConePickUpToGrid = {
                //Start to drive from TopConePickUp to grid
                {0,     -2, -0.92, 0,          0.3, 0.2 },
                //Drives up to and rotates to right grid
                {1,     -6, -0.92, 180,        0.15, 0.01},
            };

        }
    }


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
    
    public static double[][] BlueScoringWpToConeWpTag3and6 = {
        //Start at the grid in front of Tag 6
        {0,     -6, 1.041, 180,        0.3, 0.2},
        //Head to the top-most cone
        {1,     -2.5, 1.041, 180,        0.3, 0.2}
    };

    public static double[][] BlueScoringWpToConeWpTag1and8 = {
        //Start at the grid in front of Tag 8
        {0,     -6, -2.63, 180,        0.3, 0.2},
        //Head to the bottom-most cone
        {1,     -2.5, -2.63, 180,      0.3, 0.2} 
    };


}
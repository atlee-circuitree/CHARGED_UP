// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.text.DecimalFormat;
import java.awt.Desktop;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* 
This will eventually become a custom object that can store a chunk of coeffecients from PathGenerator
You should also be able to take the derivative of that function and do some other useful stuff with it    
*/

public class PathEQ {

    double[][] coords;
    private double[][] xCoefs;
    private double[][] yCoefs;

    double beginningSlope = 0;
    double endingSlope = 0;

    int subscript = 0;
    int pow = 0;
    int coef = 1;
    int matrixOffset = 0;

    
    //Used to round the calculated values
    DecimalFormat rounder = new DecimalFormat("###.####");

    String[] theAlphabet = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "X", "Y", "Z"};
  

    /** 
    * @apiNote Coordinates should be entered in the form {u,x,y}, with u being the order in which the robot should pass through said coordinates
    */
    public PathEQ(double[][] inputCoordinates, boolean runDesmos){

        coords = inputCoordinates;

        //A really big matrix. Contains all of the function constraints and modified u values
        SimpleMatrix megaMatrix = new SimpleMatrix((coords.length-1)*4, (coords.length-1)*4);
    
        //Holds the x/y values and input slopes. It is used with megaMatrix to solve for the x/y coeffecients
        SimpleMatrix solverMatrix = new SimpleMatrix((coords.length-1)*4, 1);
    
        //Holds all of the calculated coeffecients of each function
        SimpleMatrix coefMatrix = new SimpleMatrix((coords.length-1)*4, 2);



        //// FILL MEGAMATRIX START (u matrix) ////////////////////////////////////////////////////////////////////////////////////
    
   
        //Make sure that the starting point of the path has the correct slope (fill line 1)
        matrixOffset = 1;
        for(int i = 0; i < 3; i++){
            megaMatrix.set(0, i + matrixOffset, coef * Math.pow(coords[subscript][0], pow));
            pow++;
            coef++;
        }
        pow = 0;
        coef = 0;
        matrixOffset = 0;
        


        //Fill middle functions
        //Makes sure that all functions not touching the overall start/endpoints of the path
        //have matching 1st/2nd derivatives at the points they touch,
        //as well as making sure that they pass through those said points

        for(int bigLoop = 0; bigLoop < coords.length-2; bigLoop++){

            //Used to determine which row to put numbers on
            int n = bigLoop * 4;

            //Make sure that the current function touches its first endpoint
            for(int i = 0; i < 4; i++){
                megaMatrix.set(n+1, i + matrixOffset, Math.pow(coords[subscript][0], pow));
                pow++;
            }
            pow = 0;

            //Make sure that the current function touches its second endpoint
            subscript++;
            for(int i = 0; i < 4; i++){
                megaMatrix.set(n+2, i + matrixOffset, Math.pow(coords[subscript][0], pow));
                pow++;
            }
            pow = -1;

            //Make sure that both funcitons touching the 2nd endpoint have equal 1st derivatives
            //There are 2 "parts" to this line, with the 2nd being the inverse of the 1st
            int invert = 1;
            int inlineOffset = 0;
            for(int i = 0; i < 2; i++){
                for(int j = 0; j < 4; j++){
                    megaMatrix.set(n+3, j + inlineOffset + matrixOffset, coef * Math.pow(coords[subscript][0], pow) * invert);
                    pow++;
                    coef++;
                }
                pow = -1;
                coef = 0;
                invert = -1;
                inlineOffset = 4;
            }
            invert = 1;
            inlineOffset = 0;
            pow = 0;
            coef = 0;

            //Make sure that both funcitons touching the 2nd endpoint have equal 2nd derivatives
            //Again, there are 2 "parts" to this line, with the 2nd being the inverse of the 1st
            for(int i = 0; i < 2; i++){
                megaMatrix.set(n+4, inlineOffset + matrixOffset, 0);
                megaMatrix.set(n+4, inlineOffset + 1 + matrixOffset, 0);
                megaMatrix.set(n+4, inlineOffset + 2 + matrixOffset, 2 * invert);
                megaMatrix.set(n+4, inlineOffset + 3 + matrixOffset, 6 * coords[subscript][0] * invert);

                inlineOffset = 4;
                invert = -1;
            }

            //Then repeat for all middle functions, but shift all of these matrix fillers right 4 rows
            matrixOffset = matrixOffset + 4;

        }

        
        
        //There are still have 3 lines to go, each correspoding to the last function

        //Make sure that the last function touches its first endpoint
        for(int i = 0; i < 4; i++){
            megaMatrix.set(megaMatrix.numRows()-3, i + matrixOffset, Math.pow(coords[subscript][0], pow));
            pow++;
        }
        pow = 0;

        //Make sure that the last function touches the endpoint for the whole path
        subscript++;
        for(int i = 0; i < 4; i++){
            megaMatrix.set(megaMatrix.numRows()-2, i + matrixOffset, Math.pow(coords[subscript][0], pow));
            pow++;
        }
        pow = 0;
        coef = 1;

        //Make sure that the ending point of the path has the correct slope (fill last line)
        matrixOffset = matrixOffset + 1;
        for(int i = 0; i < 3; i++){
            megaMatrix.set(megaMatrix.numRows()-1, i + matrixOffset, coef * Math.pow(coords[subscript][0], pow));
            pow++;
            coef++;
        }


        //// FILL MEGAMATRIX END /////////////////////////////////////////////////////////////////////////////////////////



        
        
        //The following lines fill the solver Matrix, which will be used to solve for the coeffecients
        //We fill it and solve it twice, once for the x coefs and once for the y coefs


        for(int xy = 1; xy < 3; xy++){


            //Put the 1st specified slope as the top spot
            solverMatrix.set(0, 0, beginningSlope);
            
            //Manually fill the 2nd line to make the for loop more effecient
            solverMatrix.set(1, 0, coords[0][xy]);

            subscript = 1;

            //fill the middle function x/y values
            for(int i = 0; i < coords.length-2; i++){
                
                //used to determine which row to put values on
                int n = i * 4;

                solverMatrix.set(n + 2, 0, coords[subscript][xy]);
                solverMatrix.set(n + 3, 0, 0);
                solverMatrix.set(n + 4, 0, 0);
                solverMatrix.set(n + 5, 0, coords[subscript][xy]);

                subscript++;

            }

            //Manually fill the 2nd to last line
            solverMatrix.set(solverMatrix.numRows()-2, 0, coords[subscript][xy]);

            //Set the 2nd specified slope as the last line
            solverMatrix.set(solverMatrix.numRows()-1, 0, endingSlope);



            coefMatrix.insertIntoThis(0, xy-1, megaMatrix.invert().mult(solverMatrix));

        
        }


        //set the PathEQ coefs

        double[][] pathEQXCoefs = new double[coefMatrix.numRows()/4][5];
        double[][] pathEQYCoefs = new double[coefMatrix.numRows()/4][5];


        for(int i = 0; i < coefMatrix.numRows()/4; i++){
            
            //Set up ending u value
            pathEQXCoefs[i][0] = coords[i+1][0];
            pathEQYCoefs[i][0] = coords[i+1][0];

            //Set up X coefs
            for(int j = 0; j < 4; j++){
                pathEQXCoefs[i][j+1] = coefMatrix.get(j + (i*4), 0);
            }

            //Set up Y coefs
            for(int j = 0; j < 4; j++){
                pathEQYCoefs[i][j+1] = coefMatrix.get(j + (i*4), 1);
            }
        }


        xCoefs = pathEQXCoefs;
        yCoefs = pathEQYCoefs;


        if(runDesmos){

            //Setting up query string
            //Testing query string: ?u=1&x=1&x=2&y=1&y=2&br&u=4&x=2&x=3&y=4&y=2&

            String queryString = "";
            //Second parameter should be coefMatrix.numRows()/4
            for(int i = 0; i < coefMatrix.numRows()/4; i++){

                //Set up ending u value
                queryString = queryString + "u=" + String.valueOf(coords[i+1][0]) + "&";

                //Set up X coefs
                for(int j = 0; j < 4; j++){
                    queryString = queryString + "x=" + rounder.format(coefMatrix.get(j + (i*4), 0)) + "&";
                    
                }

                //Set up Y coefs
                for(int j = 0; j < 4; j++){
                    queryString = queryString + "y=" + rounder.format(coefMatrix.get(j + (i*4), 1)) + "&";
                }

                queryString = queryString + "br&";

            }

            //Shave off the extra br&  
            queryString = queryString.substring(0,queryString.length()-3);



            //Opening Desmos
            try {

                URI desmosURI = new URI("https://atlee-circuitree.github.io/CircuitWeaveV2/");                              
                
                URI newDesmosURI = new URI(desmosURI.getScheme(), desmosURI.getAuthority(),
                desmosURI.getPath(), queryString, desmosURI.getFragment());
                
                Desktop.getDesktop().browse(newDesmosURI);
            


            } 
            catch (URISyntaxException e) {
                SmartDashboard.putString("ERROR", "Something Went Wrong (catch1)");
            }
            catch(IOException e){
                SmartDashboard.putString("ERROR", "Something Went Wrong (catch2)");
            }
            catch(Exception e){
                SmartDashboard.putString("ERROR", "Something Went Wrong (catch3)");
            }

        }
    
        
        SmartDashboard.putNumber("solveAngle(0)", solveAngle(0));
        SmartDashboard.putNumber("solveAngle(1)", solveAngle(1));
        SmartDashboard.putNumber("solveAngle(2)", solveAngle(2));
        SmartDashboard.putNumber("solveAngle(3)", solveAngle(3));
        SmartDashboard.putNumber("solveAngle(4)", solveAngle(4));

    }










    public void dashboardYCoefs(){
        for(int i = 0; i < yCoefs.length; i++){
            SmartDashboard.putNumberArray(String.valueOf(i), yCoefs[i]);
        }
    }


    /**
     * @param uValue Input a U value
     * @return Returns the {X,Y} output for the specified U value
    */
    public double[] solvePoint(double uValue){

        //Figure out which chunk of coefs contains the U value that we are searching for

        double[] subXCoefs = new double[5];
        double[] subYCoefs = new double[5];

        //If the requested uValue is larger than the defined function, grab the last row of coefs
        if(uValue >= getFinalUValue()){
            subXCoefs = xCoefs[xCoefs.length-1];
            subYCoefs = yCoefs[yCoefs.length-1];
            uValue = getFinalUValue();
            //SmartDashboard.putNumber("BREAKPOINT", 1);
        }
        else{
            //Otherwise seacrh for which row of coefs contains the correct u value
            for(int i = 0; i < xCoefs.length; i++){
                if(xCoefs[i][0] >= uValue){
                    subXCoefs = xCoefs[i];
                    subYCoefs = yCoefs[i];
                    break;
                }
            }
        }

        double[] result = {0,0};
        int powCounter = 0;

        //Go through each X term and add them together
        //i starts at one for each loop bc the 0 index value in subX and subY Coefs is a u value, not a coef
        for(int i = 1; i < subXCoefs.length; i++){
            result[0] = result[0] + (subXCoefs[i] * Math.pow(uValue, powCounter));
            powCounter++;
        }
        powCounter = 0;

        //SmartDashboard.putNumber("X result", result[0]);
        
        //Go through each Y term and add them together
        for(int i = 1; i < subYCoefs.length; i++){
            result[1] = result[1] + (subYCoefs[i] * Math.pow(uValue, powCounter));
            powCounter++;
        }

        //SmartDashboard.putNumber("Y result", result[1]);
        //SmartDashboard.putNumberArray("subY coefs", subYCoefs);
        
        return result;

    }


    public double solveAngle(double uValue){

        double targetTheta;
        double endpoints[][] = new double[2][2];

        //If u too big, return final heading
        if(uValue >= Constants.autoCoordinates[Constants.autoCoordinates.length-1][0]){
            targetTheta = Constants.autoCoordinates[Constants.autoCoordinates.length-1][3];
        }
        //If u too small, return first heading
        else if(uValue <= Constants.autoCoordinates[0][0]){
            targetTheta = Constants.autoCoordinates[0][3];
        }
        //Otherwise, cycle through each coordinate to find which ones uValue falls between
        else{
            for(int i = 0; i < Constants.autoCoordinates.length; i++){
                if(uValue <= Constants.autoCoordinates[i][0]){
                    endpoints[0][0] = Constants.autoCoordinates[i-1][0];
                    endpoints[0][1] = Constants.autoCoordinates[i-1][3];
                    endpoints[1][0] = Constants.autoCoordinates[i][0];
                    endpoints[1][1] = Constants.autoCoordinates[i][3];
                    break;
                }
            }

            targetTheta = (slope(endpoints[0], endpoints[1]) * (uValue - endpoints[0][0])) + endpoints[0][1];

        }

        return targetTheta;

    }


    /**
     * @param point1 Pass in as {X1,Y1}
     * @param point2 Pass in as {X2,Y2}
     * @return The slope of the line passing through both points
     * @apiNote This method will return Double.POSITIVE_INFINITY or Double.NEGATIVE_INFINITY if the slope is undefined (vertical)
     */

    public double slope(double[] point1, double[] point2){
        return (point2[1] - point1[1]) / (point2[0] - point1[0]);
    }

    /**
     * @param point1 Pass in as {X1,Y1}
     * @param point2 Pass in as {X2,Y2}
     * @return {Run, Rise}
     */

    public double[] slopeRiseRun(double[] point1, double[] point2){
        double[] riseRunArray = {point2[0]-point1[0], point2[1]-point1[1]};
        return riseRunArray;
    }

    public double getFinalUValue(){
        return xCoefs[xCoefs.length-1][0];
    }

    public double[] getLastRowofXCoefs(){
        return xCoefs[xCoefs.length-1];
    }
    


}

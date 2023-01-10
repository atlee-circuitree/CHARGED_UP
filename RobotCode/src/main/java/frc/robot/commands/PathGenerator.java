// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



//This is the main paper I referenced when creating this file:
//https://people.cs.clemson.edu/~dhouse/courses/405/notes/splines.pdf



package frc.robot.commands;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.net.URI;
import java.net.URISyntaxException;
import java.text.DecimalFormat;
import java.awt.Desktop;
import java.io.IOException;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PathEQ;

public class PathGenerator extends CommandBase {
  
  //CHANGE LATER, THIS IS JUST FOR TESTING
  public PathEQ pathEQ = null;

  //coords are written in the form (u,x,y)
  //u being the order in which the robot drives through the points
  double[][] coords = {{0,0,0}, {1,-0.25,0.5}, {2,0.25,1}, {3,0,1.5}};
  double beginningSlope = 0;
  double endingSlope = 0;

  int subscript = 0;
  int pow = 0;
  int coef = 1;
  int matrixOffset = 0;

  //A really big matrix. Contains all of the function constraints and modified u values
  SimpleMatrix megaMatrix = new SimpleMatrix((coords.length-1)*4, (coords.length-1)*4);
  
  //Holds the x/y values and input slopes. It is used with megaMatrix to solve for the x/y coeffecients
  SimpleMatrix solverMatrix = new SimpleMatrix((coords.length-1)*4, 1);
  
  //Holds all of the calculated coeffecients of each function
  SimpleMatrix coefMatrix = new SimpleMatrix((coords.length-1)*4, 2);

  //Used to round the calculated values
  DecimalFormat rounder = new DecimalFormat("###.####");

  
  //Used to order the matrix rows on the sim SmartDashboard (they come up in alphabetical order)
  String[] theAlphabet = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "X", "Y", "Z"};
  
  public PathGenerator() {


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


    //Create a new PathEQ

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

    //pathEQ = new PathEQ(pathEQXCoefs, pathEQYCoefs);

    //SmartDashboard.putNumberArray("PathEQ Solve 3", pathEQ.solve(3));
    //SmartDashboard.putNumberArray("PathEQ Solve 2.99", pathEQ.solve(2.99));
    //pathEQ.dashboardYCoefs();
    //SmartDashboard.putNumber("test mult infinity", Double.POSITIVE_INFINITY*-1);







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

      //SmartDashboard.putString("desmosURI.getQuery before", desmosURI.getQuery());

      
      
      URI newDesmosURI = new URI(desmosURI.getScheme(), desmosURI.getAuthority(),
      desmosURI.getPath(), queryString, desmosURI.getFragment());

      SmartDashboard.putString("Generated URL", newDesmosURI.toString());
      SmartDashboard.putString("desmosURI.getQuery after", newDesmosURI.getQuery());
      
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






    //Puts the query string onto SmartDashboard
    SmartDashboard.putString("QueryString", queryString);


    //Put megaMatrix onto SmartDashboard
    /*
    String[] megaString = matrixToStringArray(megaMatrix);

    for(int i = 1; i < megaString.length+1; i++){
      SmartDashboard.putString(theAlphabet[i-1] + " Row " + i , megaString[i-1]);
    }
    

    //Put solverMatrix onto SmartDashboard
    String solverString = "";

    for(int i = 0; i < solverMatrix.numRows(); i++){
      solverString = solverString + String.valueOf(solverMatrix.get(i, 0)) + ", ";
    }
    SmartDashboard.putString("Y Matrix", solverString);


    //Put coefMatrix onto SmartDashboard
    String coefString = "";

    for(int i = 0; i < coefMatrix.numRows(); i++){
      coefString = coefString + String.valueOf(coefMatrix.get(i, 0)) + ", ";
    }
    SmartDashboard.putString("Z X Coef Matrix", coefString);


    coefString = "";

    for(int i = 0; i < coefMatrix.numRows(); i++){
      coefString = coefString + String.valueOf(coefMatrix.get(i, 1)) + ", ";
    }
    SmartDashboard.putString("Z Y Coef Matrix", coefString);

    */

  }

  








  //Converts a SimpleMatrix to a String[]
  public String[] matrixToStringArray(SimpleMatrix matrix){

    String[] stringArray = new String[matrix.numRows()];
    
    for(int i = 0; i < matrix.numRows(); i++){
      stringArray[i] = " ";
      for(int j = 0; j < matrix.numCols(); j++){
        stringArray[i] = stringArray[i] + String.valueOf(matrix.get(i, j)) + ", ";
      }
    }
    
    return stringArray;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

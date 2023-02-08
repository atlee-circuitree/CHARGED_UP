// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Audio extends SubsystemBase {

  /* The orchestra object that holds all the instruments */
  public Orchestra orchestra;

  /* Talon FXs to play music through.  
  More complex music MIDIs will contain several tracks, requiring multiple instruments.  */
  WPI_TalonFX []  fxes =  { new WPI_TalonFX(2, "rio"), new WPI_TalonFX(13, "rio"), new WPI_TalonFX(15, "rio") };
  String[] songs = new String[] {
    "RickRoll.chrp",
    "WeAreTheChampions.chrp",
    "MiiChannel.chrp",
    "LowBatteryBeeping.chrp" 
  };

  int songSelection = 0;

  /* overlapped actions */
  int timeToPlayLoops = 0;

  /* A list of TalonFX's that are to be used as instruments */
  ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
 
  public Audio() {

  /* Initialize the TalonFX's to be used */
  for (int i = 0; i < fxes.length; ++i) {

    instruments.add(fxes[i]);
    
  }

  /* A list of TalonFX's that are to be used as instruments */
  ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
      
  /* Initialize the TalonFX's to be used */
  for (int i = 0; i < fxes.length; ++i) {
       _instruments.add(fxes[i]);
  }

  /* Create the orchestra with the TalonFX instruments */
  orchestra = new Orchestra(_instruments);

  }

  @Override
  public void periodic() {
 
  }
 
  public void playAudio(int selection, int loops) {

    LoadMusicSelection(selection);
    timeToPlayLoops = loops;
    orchestra.play();
   
  }

  public void stopAudio() {

    orchestra.stop();

  }

  void LoadMusicSelection(int offset) {

        /* increment song selection */
        songSelection += offset;
        /* wrap song index in case it exceeds boundary */
        if (songSelection >= songs.length) {
            songSelection = 0;
        }
        if (songSelection < 0) {
            songSelection = songs.length - 1;
        }
        /* load the chirp file */
        orchestra.loadMusic(songs[songSelection]); 

        /* print to console */
        System.out.println("Song selected is: " + songs[songSelection] + ".  Press left/right on d-pad to change.");
        
        /* schedule a play request, after a delay.  
            This gives the Orchestra service time to parse chirp file.
            If play() is called immedietely after, you may get an invalid action error code. */
        timeToPlayLoops = 1;

  }

}


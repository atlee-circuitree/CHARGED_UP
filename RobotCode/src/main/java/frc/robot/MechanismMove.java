// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class MechanismMove {

    private double encTicksOffset;
    private double absStart;
    private double encOffset;
    private double deadBand;
    private double cir;
    private DutyCycleEncoder absEncoder;

    public MechanismMove(DutyCycleEncoder absoluteEncoder, double absoluteStart, double circumference) {

        absEncoder = absoluteEncoder;
        absStart = absoluteStart;
        cir = circumference;

        if (absEncoder.getAbsolutePosition() >= absStart) {

            encOffset = absStart - absEncoder.getAbsolutePosition();

        } else {

            encOffset = absEncoder.getAbsolutePosition() + (1 - absoluteStart);

        }

        encTicksOffset = encOffset * cir;

    }

    
    public double getAbsStart() {
        
        return absStart;
        
    }

    public void setAbsStart(double absStart) {
        
        this.absStart = absStart;
        
    }


}

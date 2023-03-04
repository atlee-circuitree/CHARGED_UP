// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class MechanismMove {

    private double m_encoderTicksOffset;
    private double m_absoluteStart;
    private double m_encoderOffset;
    private double m_deadBand = .1;
    private double m_circumference;
    private DutyCycleEncoder m_absoluteEncoder;
    private WPI_TalonFX m_motor;

    public MechanismMove(DutyCycleEncoder absoluteEncoder, WPI_TalonFX motor, double absoluteStart, double circumference) {

        m_absoluteEncoder = absoluteEncoder;
        m_absoluteStart = absoluteStart;
        m_circumference = circumference;
        m_motor = motor;

        if (absoluteEncoder.getAbsolutePosition() >= m_absoluteStart - m_deadBand) {

            m_encoderOffset = m_absoluteEncoder.getAbsolutePosition() - m_absoluteStart;

        } else  {

            m_encoderOffset = m_absoluteEncoder.getAbsolutePosition() + (1 - m_absoluteStart);

        }

        m_encoderTicksOffset = m_encoderOffset * m_circumference;

        m_motor.setSelectedSensorPosition(m_encoderOffset);

    }

    
    public double getAbsStart() {
        
        return m_absoluteStart;
        
    }

    public void setAbsStart(double absStart) {
        
        this.m_absoluteStart = absStart;
        
    }


}

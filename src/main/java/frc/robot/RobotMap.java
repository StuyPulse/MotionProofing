/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public interface RobotMap {
    public interface PORTS{
        int TEMP_PORT = -1;
    }
    
    public interface MATH_CONSTANTS{
        double MAX_VEL = 50;
        double MAX_ACCEL = 0.2;
    }

    public interface HARDWARE_DETAIL{
        int ENCODER_TICKS_PER_REV = 100;
        double WHEEL_DIAMETER = 10.0;
    }
}
/*
    Ask if encoder is quad, and ask for ticks per rate
    Ask for max vel and max accel
    Ask Kevin if it goes clockwise or countercl
*/

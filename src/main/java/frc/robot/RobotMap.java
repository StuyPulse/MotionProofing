/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public interface RobotMap {
    int TEMP_PORT = -1;
    
    double KP = 0.8;
    double KI = 1;
    double KD = 1;
    double MAX_VEL = 50;
    double MAX_ACCEL = 0.2;

    int ENCODER_TPR = 100;
    double WHEEL_DIAMETER = 10.0;
}

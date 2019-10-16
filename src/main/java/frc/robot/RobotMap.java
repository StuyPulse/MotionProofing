/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public interface RobotMap {
    public interface PORTS{
        int FRONT_RIGHT_MOTOR_CHANNEL = 1;
        int REAR_RIGHT_MOTOR_CHANNEL = 2;
        int REAR_LEFT_MOTOR_CHANNEL = 3;
        int FRONT_LEFT_MOTOR_CHANNEL = 4;

        int LEFT_ENCODER_CHANNEL_A = 0;
        int LEFT_ENCODER_CHANNEL_B = 1;
        int RIGHT_ENCODER_CHANNEL_A = 2;
        int RIGHT_ENCODER_CHANNEL_B = 3;
    }
    
    public interface MATH_CONSTANTS{
        double MAX_VEL = 1.7;
        double MAX_ACCEL = 0.2;
    }

    public interface HARDWARE_DETAIL{
        int ENCODER_TICKS_PER_REV = 256;

        // In feet to match field maps
        double CHASSIS_WIDTH = 20.875/12;
        double WHEEL_TO_WHEEL_WIDTH = 25.5/12;
        double WHEEL_DIAMETER = 7.5;
    }
}
/*.
    Ask if encoder is quad, and ask for ticks per rate
    Test maxvel
    Ask Kevin if it goes clockwise or countercl
*/

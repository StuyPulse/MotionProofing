/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import frc.robot.RobotMap;

public class Drivetrain extends Subsystem {

  private Encoder leftEncoder, rightEncoder;

  private WPI_TalonSRX leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;

  private SpeedControllerGroup leftMotors, rightMotors;

  private DifferentialDrive differentialDrive;

  private Trajectory leftTrajectory, rightTrajectory;

  private EncoderFollower left, right;

  public Drivetrain() throws IOException{
    leftEncoder = new Encoder(RobotMap.TEMP_PORT, RobotMap.TEMP_PORT);
    rightEncoder = new Encoder(RobotMap.TEMP_PORT, RobotMap.TEMP_PORT);

    leftRearMotor = new WPI_TalonSRX(RobotMap.TEMP_PORT);
    leftFrontMotor = new WPI_TalonSRX(RobotMap.TEMP_PORT);
    rightRearMotor = new WPI_TalonSRX(RobotMap.TEMP_PORT);
    rightFrontMotor = new WPI_TalonSRX(RobotMap.TEMP_PORT);

    leftMotors = new SpeedControllerGroup(leftRearMotor, leftFrontMotor);
    rightMotors = new SpeedControllerGroup(rightRearMotor, rightFrontMotor);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

    leftTrajectory = Pathfinder.readFromCSV(new File("../CSV/Trajectory_left.csv"));
    rightTrajectory = Pathfinder.readFromCSV(new File("../CSV/Trajectory_right.csv"));

    left = new EncoderFollower(leftTrajectory);
    right = new EncoderFollower(rightTrajectory);

    left.configurePIDVA(RobotMap.KP, RobotMap.KI, RobotMap.KD, 1/RobotMap.MAX_VEL, RobotMap.MAX_ACCEL);
    right.configurePIDVA(RobotMap.KP, RobotMap.KI, RobotMap.KD, 1/RobotMap.MAX_VEL, RobotMap.MAX_ACCEL);
    left.configureEncoder(0, RobotMap.ENCODER_TPR, RobotMap.WHEEL_DIAMETER);
    right.configureEncoder(0, RobotMap.ENCODER_TPR, RobotMap.WHEEL_DIAMETER);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void autoDrive(){
    tankDrive(left.calculate(leftEncoder.get()), right.calculate(rightEncoder.get()));
  }
}

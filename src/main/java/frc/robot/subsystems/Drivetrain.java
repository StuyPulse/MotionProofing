/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

public class Drivetrain extends Subsystem {

  private WPI_TalonSRX leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;

  private Encoder leftEncoder, rightEncoder;

  private AHRS gyro;

  private SpeedControllerGroup leftMotors, rightMotors;

  private DifferentialDrive differentialDrive;

  public Drivetrain() {
    leftRearMotor = new WPI_TalonSRX(RobotMap.PORTS.REAR_LEFT_MOTOR_CHANNEL);
    leftFrontMotor = new WPI_TalonSRX(RobotMap.PORTS.FRONT_LEFT_MOTOR_CHANNEL);
    rightRearMotor = new WPI_TalonSRX(RobotMap.PORTS.REAR_RIGHT_MOTOR_CHANNEL);
    rightFrontMotor = new WPI_TalonSRX(RobotMap.PORTS.FRONT_RIGHT_MOTOR_CHANNEL);
    
    leftEncoder = new Encoder(RobotMap.PORTS.LEFT_ENCODER_CHANNEL_A, RobotMap.PORTS.LEFT_ENCODER_CHANNEL_B);
    rightEncoder = new Encoder(RobotMap.PORTS.RIGHT_ENCODER_CHANNEL_A, RobotMap.PORTS.RIGHT_ENCODER_CHANNEL_B);

    gyro = new AHRS(I2C.Port.kMXP);

    leftMotors = new SpeedControllerGroup(leftRearMotor, leftFrontMotor);
    rightMotors = new SpeedControllerGroup(rightRearMotor, rightFrontMotor);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public int getLeftEncoderTicks() {
    return leftEncoder.get();
  }

  public int getRightEncoderTicks() {
    return rightEncoder.get();
  }

  public void stop(){
    tankDrive(0, 0);
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }
}

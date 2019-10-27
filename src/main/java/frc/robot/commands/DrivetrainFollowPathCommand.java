/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

public class DrivetrainFollowPathCommand extends Command {
  String leftPathname;
  String rightPathname;

  Trajectory leftTrajectory;
  Trajectory rightTrajectory;

  DistanceFollower left;
  DistanceFollower right;

  Notifier notifier;

  double period;

  public DrivetrainFollowPathCommand(String leftPathname, String rightPathname) {
    requires(Robot.drivetrain);
    this.leftPathname = leftPathname;
    this.rightPathname = rightPathname;

    try {
      leftTrajectory = Pathfinder.readFromCSV(new File(Filesystem.getDeployDirectory(), leftPathname));
      rightTrajectory = Pathfinder.readFromCSV(new File(Filesystem.getDeployDirectory(), rightPathname));
    } catch (IOException e) {
      e.printStackTrace();
    }

    period = left.getSegment().dt/1000;
  }

  protected void follow() {
    double l = left.calculate(Robot.drivetrain.getLeftEncoderDistance());
    double r = right.calculate(Robot.drivetrain.getRightEncoderDistance());
    
    double gyro_heading = Robot.drivetrain.getGyroAngle();
    double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
    
    // This allows the angle difference to respect 'wrapping', where 360 and 0 are the same value
    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading) % 360.0;
    if (Math.abs(angleDifference) > 180.0) {
      angleDifference = (angleDifference > 0) ? angleDifference - 360 : angleDifference + 360;
    } 

    double turn = 0.8 * (-1.0/80.0) * angleDifference;

    Robot.drivetrain.tankDrive(l + turn, r - turn);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.resetEncoders();

    left = new DistanceFollower(leftTrajectory);
    right = new DistanceFollower(rightTrajectory);

    double Kp = SmartDashboard.getNumber("Kp", 0.8);
    double Ki = SmartDashboard.getNumber("Ki", 0.0);
    double Kd = SmartDashboard.getNumber("Kd", 0.0);

    left.configurePIDVA(Kp, Ki, Kd, 1/RobotMap.MATH_CONSTANTS.MAX_VEL, 1/RobotMap.MATH_CONSTANTS.MAX_ACCEL);
    right.configurePIDVA(Kp, Ki, Kd, 1/RobotMap.MATH_CONSTANTS.MAX_VEL, 1/RobotMap.MATH_CONSTANTS.MAX_ACCEL);
  
    notifier = new Notifier(() -> follow());
    notifier.startPeriodic(period);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return left.isFinished() && right.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drivetrain.stop();
  }
}

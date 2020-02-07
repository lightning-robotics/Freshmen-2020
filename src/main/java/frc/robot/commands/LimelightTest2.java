/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class LimelightTest2 extends Command {

  // variables for height of robot, height of target, angle of robot
  private double h1 = 7.02;
  private double h2 = 23.5;
  private double pheta = 18.123;

  // our current distance and the distance where we want to be
  private double currentDistance;
  private double targetDistance = 50;

  // the PID variables of the program
  private double kPDistance = .3;
  private double kIDistance = .001;
  private double kDDistance = .0001;

  // the error sum for the integral and previous error for the derivative
  private double errorSum = 0;
  private double previousError = 0;

  public LimelightTest2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  /**
   * gets the distance from robot to target
   * @return estimated distance
   */
  protected double getDistanceFrom() {
    double heightDegree = Robot.ty.getDouble(0.0);
    double radiansA1 = pheta * Math.PI / 180;
    double radiansA2 = heightDegree * Math.PI / 180;

    return (h2 - h1) / Math.tan(radiansA1 + radiansA2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    end();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // gets the current distance of the limelight
    currentDistance = getDistanceFrom();

    // calculates error
    double distanceError = targetDistance - currentDistance;

    // starts off with a P loop until 1/3 from target distance
    double driving_adjust = distanceError * kPDistance;

    // adds on ID to the loop when 1/3 from target distance
    if (Math.abs(distanceError) <= targetDistance * (1.0 / 3)) {
      double futureError = (distanceError - previousError) / .02;
      driving_adjust += errorSum * kIDistance + futureError * kDDistance;
    }

    // calculates sum of the area
    errorSum += distanceError * .02;

    // peaks the output from -12 to 12 for voltage
    driving_adjust = Math.min(Math.abs(driving_adjust), 12*.8) * (driving_adjust / driving_adjust);

    System.out.println("The current adjust is " + driving_adjust);
    System.out.println(Robot.driveTrain.frontLeftMotor.getSelectedSensorVelocity());

    // sets the robot to drive until 5 inches away
    if (Math.abs(distanceError) > 5) {
      // gets the current driving adjust from -1 to 1
      double percentAdjust = driving_adjust / 12;
      // sets the talons' speed to the percent adjust 
      Robot.driveTrain.frontLeftMotor.set(ControlMode.PercentOutput, -percentAdjust*.97);
      Robot.driveTrain.frontRightMotor.set(ControlMode.PercentOutput, -percentAdjust);
    } else {
      // breaks when 5 inches away
      Robot.driveTrain.frontLeftMotor.setNeutralMode(NeutralMode.Brake);
      Robot.driveTrain.frontRightMotor.setNeutralMode(NeutralMode.Brake);
    }
    // sets the previous error to the current error of interation
    previousError = distanceError;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.FRMset(0);
    Robot.driveTrain.FLMset(0);
    Robot.driveTrain.BRMset(0);
    Robot.driveTrain.BLMset(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

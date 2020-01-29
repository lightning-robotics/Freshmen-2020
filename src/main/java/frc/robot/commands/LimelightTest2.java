/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class LimelightTest2 extends Command {

  private double h1 = 7.02;
  private double h2 = 23.5;
  private double pheta = 14.3;
  private double currentDistance;

  private double kPDistance = .4;
  private double targetDistance = 50;
  private double kIDistance = .03;
  private double errorSum = 0;

  public LimelightTest2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  protected double getDistanceFrom() {
    double heightDegree = Robot.ty.getDouble(0.0);
    double radiansA1 = pheta * Math.PI / 180;
    double radiansA2 = heightDegree * Math.PI / 180;
    
    return (h2-h1) / Math.tan(radiansA1 + radiansA2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // DriveTrain.frontLeftMotor.selected

    // DriveTrain.frontLeftMotor.config_kF(0, 0);
    // DriveTrain.frontLeftMotor.config_kP(0, .15);
    // DriveTrain.frontLeftMotor.config_kI(0, 0);
    // DriveTrain.frontLeftMotor.config_kD(0, 1);

    
    // DriveTrain.frontRightMotor.config_kF(0, 0);
    // DriveTrain.frontRightMotor.config_kP(0, .15);
    // DriveTrain.frontRightMotor.config_kI(0, 0);
    // DriveTrain.frontRightMotor.config_kD(0, 1);


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {  
    
    currentDistance = getDistanceFrom();

    // DriveTrain.frontRightMotor. (currentDistance);

    double distanceError = targetDistance - currentDistance;
    // DriveTrain.frontLeftMotor.set(ControlMode.Position, targetDistance);

    errorSum += distanceError * .02;

    double driving_adjust = distanceError * kPDistance + errorSum * kIDistance;

    if (Math.abs(distanceError) <= 5) {
      distanceError = 0;
      driving_adjust = 0;
      kPDistance = -.2;
    }

    System.out.println("The current error is " + distanceError);
    System.out.println("The distance is " + currentDistance);
    System.out.println("the current adjust is " + driving_adjust);

    Robot.driveTrain.FRMset(driving_adjust);
    Robot.driveTrain.FLMset(driving_adjust);
    Robot.driveTrain.BLMset(driving_adjust);
    Robot.driveTrain.BRMset(driving_adjust);



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
    Robot.driveTrain.BRMset(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

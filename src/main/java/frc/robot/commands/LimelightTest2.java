/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class LimelightTest2 extends Command {

  private double h1 = 6.5;
  private double h2 = 23.5;
  private double pheta = 24.3;
  private double currentDistance;

  private double kPDistance = -0.02;
  private double targetDistance = 50;

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
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    currentDistance = getDistanceFrom();
    double distanceError = targetDistance - currentDistance;
    double driving_adjust = distanceError * kPDistance;
    System.out.println("The distance is " + currentDistance);
    System.out.println("the current adjust is " + driving_adjust);
    Robot.driveTrain.FRMset(-driving_adjust);
    Robot.driveTrain.BRMset(-driving_adjust);
    Robot.driveTrain.FLMset(-driving_adjust);
    Robot.driveTrain.BLMset(-driving_adjust);


    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

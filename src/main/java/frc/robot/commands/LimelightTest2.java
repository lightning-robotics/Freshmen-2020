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

  private double kPDistance = -.5;
  private double targetDistance = 50;
  private double kIDistance = -.01;
  private double errorSum = 0;
  private double maxSpeed = .4;

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
    if (Math.abs(distanceError) <= 5) distanceError = 0;

    errorSum += distanceError * .02;

    double driving_adjust = distanceError * kPDistance + errorSum * kIDistance;
    
    driving_adjust = driving_adjust > maxSpeed ? maxSpeed : driving_adjust;

    System.out.println("The current error is " + distanceError);
    System.out.println("The distance is " + currentDistance);
    System.out.println("the current adjust is " + driving_adjust);

    if (distanceError != 0) {
      Robot.driveTrain.FRMset(-driving_adjust);
      Robot.driveTrain.BRMset(-driving_adjust);
      Robot.driveTrain.FLMset(-driving_adjust);
      Robot.driveTrain.BLMset(-driving_adjust);
    }
    
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

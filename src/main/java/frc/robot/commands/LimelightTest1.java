/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.PIDController;
// import edu.wpi.first.wpilibj.PIDOutput;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class LimelightTest1 extends Command {
  double previousError = 0;
  double errorSum = 0;
  double Kp = .2;
  double Ki = .02;
  boolean stop = false;

  public LimelightTest1() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double x = Robot.tx.getDouble(0.0);
    // double y = Robot.ty.getDouble(0.0);

    double headingError = x;

    errorSum += headingError * .02;

    double p = Kp * headingError;
    double i = Ki * errorSum;
    double steering_adjust = p + i;

    if (Math.abs(headingError) <= 1 || stop) 
      steering_adjust = 0;
      System.out.println("Stopping");
      Kp = .1;
      Ki = .01;
      stop = (Math.abs(headingError) < 2);
    

    previousError = headingError;

    System.out.println("turn error " + steering_adjust + " actual error " + x);

    // System.out.println("x " + x + " Y " + y);

    // double Kp = 0.05;
    // double min_command = .1;

    // double heading_error = x;
    // double steering_adjust = 0.0;
    
    // if (x > 1.0){
    //   steering_adjust = Kp * heading_error - min_command;
      
    // }
    // else if (x < -1.0){
    //   steering_adjust = Kp * heading_error + min_command;
    // }
    
    Robot.driveTrain.FRMset(steering_adjust);
    // Robot.driveTrain.BRMset(steering_adjust);
    Robot.driveTrain.FLMset(-steering_adjust);
    // Robot.driveTrain.BLMset(-steering_adjust);
  //todo make more better and add face detection 


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

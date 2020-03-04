/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TankDrive extends Command {
  double reverse = 1;
  int buttonNum = 0;
  public TankDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.setLeftMotorSpeed(0);
    Robot.driveTrain.setRightMotorSpeed(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double driverAxis = Robot.oi.getControllerAxis(Robot.oi.driver, RobotMap.ROBOT_DRIVE_YAXIS);
    double driverAxis2 = Robot.oi.getControllerAxis(Robot.oi.driver, RobotMap.ROBOT_DRIVE_XAXIS);

    double varSpeed = 0.60;

    if (Robot.oi.driver.getBumper(Hand.kLeft))
      buttonNum += 1;
    
    switch (buttonNum) {
      case 1:
        break;
      case 2:
        reverse = -1;
        break;
      default:
        buttonNum = 0;
        reverse = 1;
        break;
    }

    driverAxis = Math.min(Math.abs(driverAxis), varSpeed) * (driverAxis/Math.abs(driverAxis));
    driverAxis2 = Math.min(Math.abs(driverAxis2), varSpeed) * (driverAxis2/Math.abs(driverAxis2));

    driverAxis *= reverse;
    driverAxis2 *= reverse;

    if (Robot.driveTrain.getDeadzone(driverAxis2)) 
      driverAxis2 = 0;
    if (Robot.driveTrain.getDeadzone(driverAxis))
      driverAxis = 0;
      
    Robot.driveTrain.setLeftMotorSpeed(driverAxis2 + driverAxis);
    Robot.driveTrain.setRightMotorSpeed(driverAxis2 + driverAxis);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    initialize();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

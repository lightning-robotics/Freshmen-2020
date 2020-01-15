/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TankDrive extends Command {
  public TankDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.setLeftMotorSpeed(0, 0);
    Robot.driveTrain.setRightMotorSpeed(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double driverAxis = Robot.oi.getDriverAxis(RobotMap.ROBOT_DRIVE_YAXIS);
    double driverAxis2 = Robot.oi.getDriverAxis(RobotMap.ROBOT_DRIVE_XAXIS);

    double varSpeed = 0.50;

    if (Math.abs(driverAxis2) > RobotMap.DEADZONE || Math.abs(driverAxis) > RobotMap.DEADZONE) {
      if (driverAxis2 > varSpeed) {
        driverAxis2 = varSpeed;
        Robot.driveTrain.setLeftMotorSpeed(0, -driverAxis2);
        Robot.driveTrain.setRightMotorSpeed(0, driverAxis2);
        System.out.println("turning right");
      } else if (driverAxis2 < -varSpeed) {
        driverAxis2 = -varSpeed;
        Robot.driveTrain.setRightMotorSpeed(0, driverAxis2);
        Robot.driveTrain.setLeftMotorSpeed(0, -driverAxis2);
        System.out.println("turning left");
      }
      if (driverAxis > varSpeed) {
        driverAxis = varSpeed;
        Robot.driveTrain.setLeftMotorSpeed(driverAxis, 0);
        Robot.driveTrain.setRightMotorSpeed(driverAxis, 0);
        System.out.println("Driving backward");
      } else if (driverAxis < -varSpeed) {
        driverAxis = -varSpeed;
        Robot.driveTrain.setLeftMotorSpeed(driverAxis, 0);
        Robot.driveTrain.setRightMotorSpeed(driverAxis, 0);
        System.out.println("Driving forward");
      }
    } else {
      Robot.driveTrain.setLeftMotorSpeed(0, 0);
      Robot.driveTrain.setRightMotorSpeed(0, 0);
    }
    // Robot.driveTrain.setLeftMotorSpeed(driverAxis, driverAxis2);
    // Robot.driveTrain.setRightMotorSpeed(driverAxis, driverAxis2);

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

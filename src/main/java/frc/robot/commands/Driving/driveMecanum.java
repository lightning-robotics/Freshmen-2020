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

public class driveMecanum extends Command {
  public driveMecanum() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.setLeftMotorSpeed(0, 0);
    Robot.driveTrain.setRightMotorSpeed(0, 0);
    // drive.setSafetyEnabled(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double driverAxisL = Robot.oi.getControllerAxis(Robot.oi.driver, RobotMap.ROBOT_DRIVE_YAXIS) * .5;
    double driverAxisR = Robot.oi.getControllerAxis(Robot.oi.driver, RobotMap.ROBOT_DRIVE_XAXIS_2) * .5;
    //System.out.println(driverAxisL);
    // turn right
    if (Robot.oi.driver.getTriggerAxis(Hand.kRight) > RobotMap.TRIGGER_DEADZONE) {
      double value = Robot.oi.driver.getTriggerAxis(Hand.kRight) / 2.0;
      Robot.driveTrain.setLeftMotorSpeed(value * -1, 0);
      Robot.driveTrain.setRightMotorSpeed(value, 0);
    } 
    // turn left
    else if (Robot.oi.driver.getTriggerAxis(Hand.kLeft) > RobotMap.TRIGGER_DEADZONE) {
      double value = Robot.oi.driver.getTriggerAxis(Hand.kLeft) / 2.0;
      Robot.driveTrain.setLeftMotorSpeed(value, 0);
      Robot.driveTrain.setRightMotorSpeed(value * -1, 0);
    }
    // drive forward right
    else if (driverAxisL < -1*RobotMap.DEADZONE && Robot.oi.driver.getBumper(Hand.kRight)) {
      Robot.driveTrain.FLMset(driverAxisL);
      Robot.driveTrain.BRMset(driverAxisL);
    }
    // drive backwards right
    else if (driverAxisL > RobotMap.DEADZONE && Robot.oi.driver.getBumper(Hand.kRight)) {
      Robot.driveTrain.FRMset(driverAxisL);
      Robot.driveTrain.BLMset(driverAxisL);
    }
    // drive forwards left
    else if (driverAxisL < -1*RobotMap.DEADZONE && Robot.oi.driver.getBumper(Hand.kLeft)) {
      Robot.driveTrain.FRMset(driverAxisL);
      Robot.driveTrain.BLMset(driverAxisL);
    }
    // drive backwards left
    else if (driverAxisL > RobotMap.DEADZONE && Robot.oi.driver.getBumper(Hand.kLeft)) {
      Robot.driveTrain.FLMset(driverAxisL);
      Robot.driveTrain.BRMset(driverAxisL);
    }
     // drive forward and backwards
     else if (Math.abs(driverAxisL) > RobotMap.DEADZONE){
      System.out.println("Driving forwards or backwards");
      Robot.driveTrain.BLMset(driverAxisL);
      Robot.driveTrain.BRMset(driverAxisL);
      Robot.driveTrain.FRMset(driverAxisL);
      Robot.driveTrain.FLMset(driverAxisL);
    }
    // drive right or left
    else if (Math.abs(driverAxisR) > RobotMap.DEADZONE) {
      System.out.println("Driving left or right");
      Robot.driveTrain.BLMset(driverAxisR);
      Robot.driveTrain.FRMset(driverAxisR);
      Robot.driveTrain.FLMset(driverAxisR * -1);
      Robot.driveTrain.BRMset(driverAxisR * -1);
    } 
    else {
      // turn off everything. don't kill me please :+)
      Robot.driveTrain.setLeftMotorSpeed(0, 0);
      Robot.driveTrain.setRightMotorSpeed(0, 0);
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
    initialize();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

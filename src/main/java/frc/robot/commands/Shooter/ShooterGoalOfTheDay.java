/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

public class ShooterGoalOfTheDay extends Command {

  public boolean done = false;

  public ShooterGoalOfTheDay() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Robot.shooter.stopShooter();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    double distance = Robot.limelight.getDistanceFrom();

    // TODO: change this to mechanism joystick once equations are found
    if (Robot.oi.driver.getXButton()) {
      Robot.shooter.shootDistance(distance);
      done = true;
      isFinished();
    }
    initialize();

  }

  // Called once after isFinished returns true
  @Override
  public void end() {
    initialize();
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return done;
  }
}

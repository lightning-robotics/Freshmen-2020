/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ShooterGoalOfTheDay extends CommandBase {

  boolean done = false;

  /**
   * Creates a new ShooterGoalOfTheDay.
   */
  public ShooterGoalOfTheDay() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double distanceInInches = Robot.limelight.getDistanceFrom();
    double distanceInTicks = Robot.limelight.distanceInTicks(RobotMap.SHOOTER_TICKS_PER_INCH);

    if (Robot.oi.mechanism.getBButton()) {
      Robot.shooter.shootDistance(distanceInTicks);
      done = true;
    } else {
      Robot.shooter.stopShooter();
    }
    isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}

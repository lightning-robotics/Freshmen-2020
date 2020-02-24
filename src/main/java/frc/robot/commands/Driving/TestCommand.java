/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TestCommand extends CommandBase {
  // this is how far we want the robot to drive
  // in this test, I want it to drive three full rotations
  private double distance;
  /**
   * Creates a new TestCommand.
   */
  public TestCommand(double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.driveTrain.driveAll(0);
    // TODO: Find good PID values for this
    Robot.driveTrain.setPID(
      .01, 
      .02, 
      .001, 
      0.0
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // TODO: run the pid here from the drive train subsystem
    // don't forget the distance

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.driveTrain.finishedPID();
  }
}

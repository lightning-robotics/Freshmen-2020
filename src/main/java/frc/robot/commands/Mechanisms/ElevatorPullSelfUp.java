/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ElevatorPullSelfUp extends CommandBase {
  private boolean done = false;
  private int targetRotation;
  /**
   * Creates a new ElevatorPullSelfUp.
   */
  public ElevatorPullSelfUp(int targetRotation) {
    // Use addRequirements() here to declare subsystem dependencies
    this.targetRotation = targetRotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // sets the elevator to zero power at the beginning
    Robot.elevator.setPower(0);
    // For later use, sets the elevator's PID 
    Robot.elevator.setPID(
      RobotMap.ELEVATOR_F, 
      RobotMap.ELEVATOR_P, 
      RobotMap.ELEVATOR_I, 
      RobotMap.ELEVATOR_D
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // runs the PID from the elevator subsystem
    Robot.elevator.runPID(targetRotation);

    // checks if the pid has finished
    if (Robot.elevator.donePID()) done = true;
    // stops the program if the robot is done
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

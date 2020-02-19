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

public class ElevatorUp extends CommandBase {
  
  /**
   * Creates a new Elevator.
   */
  public ElevatorUp() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Robot.elevator.setPower(0.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double axis = Robot.oi.getControllerAxis(Robot.oi.mechanism, RobotMap.ROBOT_DRIVE_YAXIS);

    axis = Math.min(Math.abs(axis), .5);

    if (Math.abs(axis) > RobotMap.DEADZONE) {
      Robot.elevator.setPower(axis);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

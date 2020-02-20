/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ElevatorPullSelfUp extends PIDCommand {
  /**
   * Creates a new ElevatorPullSelfUp.
   */
  // TODO: write PID to pull elevator to certain height and maintain position
  public ElevatorPullSelfUp() {
    super(
        // The controller that the command will use
        new PIDController(RobotMap.kPElevator, RobotMap.kIElevator, RobotMap.kDElevator),
        // This should return the measurement
        Robot.elevator::getCombinedDistance,
        // This should return the setpoint (can also be a constant)
        RobotMap.ELEVATOR_TARGET_HEIGHT,
        // This uses the output
        output -> {
          // Use the output here
          Robot.elevator.setPower(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

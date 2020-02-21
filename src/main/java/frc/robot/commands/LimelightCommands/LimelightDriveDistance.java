/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LimelightCommands;


import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Robot;
import frc.robot.RobotMap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class LimelightDriveDistance extends PIDCommand {
  /**
   * Creates a new LimelightDriveDistance.
   */
  public LimelightDriveDistance() {
    super(
        // The controller that the command will use
        new PIDController(RobotMap.kPDrive, RobotMap.kIDrive, RobotMap.kDDrive),
        // This should return the measurement
        Robot.limelight::getDistanceFrom,
        // This should return the setpoint (can also be a constant)
        RobotMap.TARGET_DISTANCE,
        // This uses the output
        output -> Robot.driveTrain.driveAll(output)
          // Use the output here
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(RobotMap.DISTANCE_TOLERANCE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }


}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class LimelightDriveDistance extends CommandBase {
  /**
   * Creates a new LimelightDriveDistance.
   */
  public LimelightDriveDistance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Robot.driveTrain.setPID(RobotMap.kFDrive, RobotMap.kPDrive, RobotMap.kIDrive, RobotMap.kDDrive);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distanceInTicks = Robot.limelight.distanceInTicks(RobotMap.DRIVER_TICKS_PER_INCH);

    Robot.driveTrain.runPID(distanceInTicks);

    isFinished();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.driveTrain.finishedPID();
  }
}

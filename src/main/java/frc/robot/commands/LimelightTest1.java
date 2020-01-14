/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightTest1 extends Command {
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  public LimelightTest1() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    System.out.println("x " + x + " Y " + y + " area " + area);

    double kSpeed = -0.3;
    double turnDeadzone = 3;

    double average = (double) (x + y) / 2;

    if (average > turnDeadzone) {
      Robot.driveTrain.FLMset(kSpeed);
      Robot.driveTrain.BLMset(kSpeed);
    } else {
      Robot.driveTrain.FLMset(0);
      Robot.driveTrain.BLMset(0);
    }

    if (average < -1 * turnDeadzone) {
      Robot.driveTrain.FRMset(kSpeed);
      Robot.driveTrain.BRMset(kSpeed);
    }

    else {
      Robot.driveTrain.FRMset(0);
      Robot.driveTrain.BRMset(0);
    }

    SmartDashboard.putNumber("limelight X", x);
    SmartDashboard.putNumber("limelight Y", y);
    SmartDashboard.putNumber("limelight area", area);

 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

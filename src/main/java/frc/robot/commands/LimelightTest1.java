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

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.PIDController;
// import edu.wpi.first.wpilibj.PIDOutput;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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

    double Kp = -0.04;
    double min_command = .07;

    double heading_error = -x;
    double steering_adjust = 0.0;
    
    if (x > 1.0){
      steering_adjust = Kp * heading_error - min_command;
    }
    else if (x < 1.0){
      steering_adjust = Kp * heading_error + min_command;
    }

    Robot.driveTrain.FRMset(steering_adjust);
    Robot.driveTrain.BRMset(steering_adjust);
    Robot.driveTrain.FLMset(-steering_adjust);
    Robot.driveTrain.BLMset(-steering_adjust);
    //todo make more better and add face detection 


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

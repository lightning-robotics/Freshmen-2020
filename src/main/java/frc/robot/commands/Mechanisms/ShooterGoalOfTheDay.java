/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.RobotMap;

public class ShooterGoalOfTheDay extends Command {

  TalonSRX shooter1 = new TalonSRX(RobotMap.SHOOTER_TOP); //top
  TalonSRX shooter2 = new TalonSRX(RobotMap.SHOOTER_BOTTOM);

  public ShooterGoalOfTheDay() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    shooter1.set(ControlMode.PercentOutput, 0);
    shooter2.set(ControlMode.PercentOutput, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    if (Robot.oi.driver.getXButton()) {
      shooter1.set(ControlMode.PercentOutput, .75);
      shooter2.set(ControlMode.PercentOutput, -.70);
    } else {
      end();
    }

  }

  // Called once after isFinished returns true
  @Override
  public void end() {
    initialize();
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return false;
  }
}

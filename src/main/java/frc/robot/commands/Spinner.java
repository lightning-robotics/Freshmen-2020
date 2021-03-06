/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Spinner extends Command {
  public Spinner() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  TalonSRX spinner = new TalonSRX(RobotMap.SPINNER_MOTOR);

  private boolean isOn = false;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    spinner.set(ControlMode.PercentOutput, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (Robot.oi.driver.getBButton()) {   //remove later just for testing
      isOn = true;
    }


    if (isOn == true) {

    spinner.set(ControlMode.PercentOutput, -0.40);
    Timer.delay(6.154); //4.7 upper limit 4.63 lowwer limit at 40%
    //slow at 20% is 15 lots less error
    spinner.set(ControlMode.PercentOutput, .20);
    Timer.delay(0.245);


  //  spinner.set(ControlMode.PercentOutput, -0.80);
  //   Timer.delay(2.2);

  //   spinner.set(ControlMode.PercentOutput, .40);
  //   Timer.delay(0.245);


  // spinner.set(ControlMode.PercentOutput, -1);
  //   Timer.delay(1.7);

  //   spinner.set(ControlMode.PercentOutput, .50);
  //   Timer.delay(0.245);

    isOn = false;
    } else {
      spinner.set(ControlMode.PercentOutput, 0);
    }

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

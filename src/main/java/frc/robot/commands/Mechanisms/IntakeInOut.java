/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Mechanisms;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class IntakeInOut extends CommandBase {
  /**
   * Creates a new IntakeInOut.
   */
  public IntakeInOut() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.intake.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.oi.driver.getBumper(Hand.kRight)) {
      Robot.intake.setSpeed(-RobotMap.INTAKE_SPEED);
      System.out.println("setting speed");
    }
    else if (Robot.oi.driver.getBumper(Hand.kLeft))
      Robot.intake.setSpeed(RobotMap.INTAKE_SPEED);
    else
    Robot.intake.setSpeed(0);
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

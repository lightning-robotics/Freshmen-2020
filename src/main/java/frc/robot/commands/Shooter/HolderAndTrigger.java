/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class HolderAndTrigger extends CommandBase {
  double time;
  /**
   * Creates a new HolderAndTrigger.
   */
  public HolderAndTrigger(double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.intake.setHolderSpeed(0);
    Robot.intake.setTriggerSpeed(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.intake.setHolderSpeed(RobotMap.HOLDER_SPEED);
    Robot.intake.setTriggerSpeed(RobotMap.TRIGGER_SPEED);
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

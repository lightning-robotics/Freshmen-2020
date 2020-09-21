/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TurnForTime extends CommandBase {
  double initialTurnAngle;

  PIDController turning;
  /**
   * Creates a new TurnForTime.
   */
  public TurnForTime(double initialTurnAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.initialTurnAngle = initialTurnAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.driveTrain.driveAll(0);
    Robot.limelight.navx.zeroYaw();
    turning.setPID(RobotMap.KPTurn, RobotMap.KITurn, RobotMap.KDTurn);
    turning.setSetpoint(initialTurnAngle);
    turning.setTolerance(3); // allowing 3 degrees of error
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double output = turning.calculate(Robot.limelight.navx.getAngle());

    Robot.driveTrain.setLeftMotorSpeed(-output);
    Robot.driveTrain.setRightMotorSpeed(output);

    isFinished();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    initialize(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turning.atSetpoint();
  }
}

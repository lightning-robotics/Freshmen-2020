/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LimelightCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class LimelightTurnToAngle extends CommandBase {
  double initialTurnAgle;

  PIDController turning;
  /**
   * Creates a new LimelightTurnToAngle.
   */
  public LimelightTurnToAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.limelight.navx.zeroYaw();
    Robot.driveTrain.driveAll(0);
    turning.setPID(RobotMap.KPTurn, RobotMap.KITurn, RobotMap.KDTurn);
    turning.setSetpoint(Robot.limelight.getXAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double output = turning.calculate(Robot.limelight.navx.getAngle());

    Robot.driveTrain.setRightMotorSpeed(output);
    Robot.driveTrain.setLeftMotorSpeed(-output);

    isFinished();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.driveAll(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turning.atSetpoint();
  }
}

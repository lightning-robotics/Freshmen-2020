/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.LimelightCommands.LimelightTurnToAngle;

import frc.robot.commands.Shooter.ShooterGoalOfTheDay;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousRunner extends SequentialCommandGroup {
  /**
   * Creates a new AutoCenter.
   */
  public AutonomousRunner(double driveSpeed, double driveTime, int turnTime) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super((Command) new DriveForTime(driveTime, driveSpeed), (Command) new LimelightTurnToAngle().withTimeout(turnTime), (Command) new ShooterGoalOfTheDay());
  }
}
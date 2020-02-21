/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  private TalonSRX shooter1 = new TalonSRX(RobotMap.SHOOTER_TOP); //top
  private TalonSRX shooter2 = new TalonSRX(RobotMap.SHOOTER_BOTTOM);
  /**
   * Creates a new Shooter.
   */
  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTopMotor(double speed) {
    shooter1.set(ControlMode.PercentOutput, speed);
  }

  public void setBottomMotor(double speed) {
    shooter2.set(ControlMode.PercentOutput, speed);
  }

  public void shootDistance(double distance) {
    double tSpeed = topSpeedDistance(distance);
    double bSpeed = bottomSpeedDistance(distance);
    setTopMotor(tSpeed);
    setBottomMotor(bSpeed);
  }

  private double topSpeedDistance(double distance) {
    // TODO: Find this equation
    return 5 * Math.pow(distance, 2) + 4 * distance;
  }

  private double bottomSpeedDistance(double distance) {
    // TODO: Find this equation
    return -3 * Math.pow(distance, 3) + 10 * distance;
  }

  public void stopShooter() {
    shooter1.set(ControlMode.PercentOutput, 0);
    shooter2.set(ControlMode.PercentOutput, 0);
  }
}

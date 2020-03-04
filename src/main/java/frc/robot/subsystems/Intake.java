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

public class Intake extends SubsystemBase {
  private static TalonSRX leftMotor = new TalonSRX(RobotMap.LEFT_INTAKE_MOTOR);
  private static TalonSRX rightMotor = new TalonSRX(RobotMap.RIGHT_INTAKE_MOTOR);
  /**
   * Creates a new Intake.
   */
  public Intake() {
    System.out.println("Created");
    // rightMotor.follow(leftMotor);
    // setLeftInverted();
    rightMotor.set(ControlMode.PercentOutput, .3);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // private static void setLeftInverted() {
  //   leftMotor.setInverted(true);
  // }

  public void setSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, -speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

}

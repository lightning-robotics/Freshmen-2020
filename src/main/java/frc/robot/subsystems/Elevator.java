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

public class Elevator extends SubsystemBase {

    // initializing the elevator motors
    private static TalonSRX elevatorMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR);

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    // inverts the left elevator motor to go up
    // elevatorMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sends power to the elevator motors
   * Can't send power to the both of them as they should be equal height
   * @param power the amount of power from [-1,1] to send to the motors
   */
  public void setPower(double power) {
    elevatorMotor.set(ControlMode.PercentOutput, power);
  }
}

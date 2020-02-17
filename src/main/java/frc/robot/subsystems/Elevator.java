/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {

    // initializing the elevator motors
    private TalonSRX elevatorRight = new TalonSRX(RobotMap.ELEVATOR_RIGHT);
    private TalonSRX elevatorLeft = new TalonSRX(RobotMap.ELEVATOR_LEFT);
  
    // initializing the encoders
    private Encoder encoderRight = new Encoder(RobotMap.ELEVATOR_RIGHT_ENCODER_A, RobotMap.ELEVATOR_RIGHT_ENCODER_B);
    private Encoder encoderLeft = new Encoder(RobotMap.ELEVATOR_LEFT_ENCODER_A, RobotMap.ELEVATOR_LEFT_ENCODER_B);

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    elevatorLeft.setInverted(true);
    encoderLeft.setDistancePerPulse(4);
    encoderRight.setDistancePerPulse(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    elevatorLeft.set(ControlMode.PercentOutput, power);
    elevatorRight.set(ControlMode.PercentOutput, power);
  }

  public double getLeftDistance() {
    return encoderLeft.getDistance();
  }
  
  public double getRightDistance() {
    return encoderRight.getDistance();
  }

  // ensure that we don't put too much power on a slow motor
  public double getCombinedDistance() {
    return Math.min(getLeftDistance(), getRightDistance());
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {

    // initializing the elevator motors
    private TalonSRX elevatorMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR);
  
    // // initializing the encoders
    // private Encoder encoderRight = new Encoder(RobotMap.ELEVATOR_RIGHT_ENCODER_A, RobotMap.ELEVATOR_RIGHT_ENCODER_B);
    // private Encoder encoderLeft = new Encoder(RobotMap.ELEVATOR_LEFT_ENCODER_A, RobotMap.ELEVATOR_LEFT_ENCODER_B);

    // resetting the encoders at the start of the PID
    private static boolean resetValue = true;

  public Elevator() {
    elevatorMotor.setInverted(true);
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    // elevatorLeft.
    // encoderLeft.setDistancePerPulse(RobotMap.DISTANCE_PER_PLUSE);
    // encoderRight.setDistancePerPulse(RobotMap.DISTANCE_PER_PLUSE);
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

  // public double getLeftDistance() {
  //   return encoderLeft.getDistance();
  // }
  
  // public double getRightDistance() {
  //   return encoderRight.getDistance();
  // }

  // // ensure that we don't put too much power on a slow motor
  // public double getCombinedDistance() {
  //   if (resetValue) {
  //     encoderLeft.reset();
  //     encoderRight.reset();
  //     resetValue = false;
  //   }
  //   return Math.min(getLeftDistance(), getRightDistance());
  // }
}

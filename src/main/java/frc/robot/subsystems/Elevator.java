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
    private static TalonSRX elevatorMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR);

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    // inverts the left elevator motor to go up
    // elevatorMotor.setInverted(true);

    // configuring the feedback device
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

		/* Set the peak and nominal outputs */
		elevatorMotor.configNominalOutputForward(0);
		elevatorMotor.configNominalOutputReverse(0);
		elevatorMotor.configPeakOutputForward(1);
    elevatorMotor.configPeakOutputReverse(-1);
    
    // configuring the PID
    elevatorMotor.config_kF(0, .01);
    elevatorMotor.config_kP(0, (.1 * 1023) / 600);
    elevatorMotor.config_kI(0, .001);
    elevatorMotor.config_kD(0, 1024 / 600);

    // setting up the cruise velocity and acceleration
    elevatorMotor.configMotionCruiseVelocity(3072);
    elevatorMotor.configMotionAcceleration(3072);

    // setting the initial poisiton
    elevatorMotor.setSelectedSensorPosition(0);
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

  public void setMagic(double target) {
    // updating magic
    // int ticks = elevatorMotor.getSensorCollection().getPulseWidthPosition();
    // elevatorMotor.setSelectedSensorPosition(ticks);
    // running the magic
    elevatorMotor.set(ControlMode.MotionMagic, target);
  }
}

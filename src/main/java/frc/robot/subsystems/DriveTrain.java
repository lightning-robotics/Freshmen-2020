/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Declartion of driver motors (hi)
  private static final TalonSRX frontRightMotor = new TalonSRX(RobotMap.FRONT_RIGHT_MOTOR);
  private static final TalonSRX backRightMotor = new TalonSRX(RobotMap.BACK_RIGHT_MOTOR);
  private static final TalonSRX frontLeftMotor = new TalonSRX(RobotMap.FRONT_LEFT_MOTOR);
  private static final TalonSRX backLeftMotor = new TalonSRX(RobotMap.BACK_LEFT_MOTOR);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // // setDefaultCommand(new MySpecialCommand());

    // create robotmap values for this
    frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        // place pid id number 0 - 1,
        0,
        // place time desired for pid to update 30 ms
        30);
    // do the same for the front right motor
    frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
    0, 30);

      // set the back motors to follow the front
      DriveTrain.backLeftMotor.follow(DriveTrain.frontLeftMotor);
      DriveTrain.backRightMotor.follow(DriveTrain.frontRightMotor);
  
      DriveTrain.backLeftMotor.setInverted(true);
      DriveTrain.frontLeftMotor.setInverted(true);
  
      DriveTrain.frontLeftMotor.setNeutralMode(NeutralMode.Brake);
      DriveTrain.backLeftMotor.setNeutralMode(NeutralMode.Brake);
      DriveTrain.frontRightMotor.setNeutralMode(NeutralMode.Brake);
      DriveTrain.backRightMotor.setNeutralMode(NeutralMode.Brake);

    // this creates a minumum output for the PID
    frontLeftMotor.configNominalOutputForward(0, // sets the minumum power to zero
        30 // sets the update time to 30
    );
    // for the reverse of the motors
    frontLeftMotor.configNominalOutputReverse(0, // sets the minumum speed to 0
        30 // sts the update time to 30
    );

    frontLeftMotor.configClosedLoopPeakOutput(0, .70, 30);
    frontRightMotor.configClosedLoopPeakOutput(0, 0.70, 30);

    frontRightMotor.configNominalOutputReverse(0, 30);
    frontRightMotor.configNominalOutputForward(0, 30);

    frontLeftMotor.configAllowableClosedloopError(0, // pid id
        0, // allowable error (in this case, 0 ticks)
        30 // update time (30 ms)
    );
    frontRightMotor.configAllowableClosedloopError(0, 0, 30);
    frontLeftMotor.configAllowableClosedloopError(0, 0, 30);
  }

  public void setRightMotorSpeed(double speed) {
    if (getDeadzone(speed)) 
      speed = 0;
    frontRightMotor.set(ControlMode.PercentOutput, speed);
    // backRightMotor.set(ControlMode.PercentOutput, speedY + speedX);
  }

  public void setLeftMotorSpeed(double speed) {
    if (getDeadzone(speed)) 
      speed = 0;
    frontLeftMotor.set(ControlMode.PercentOutput, speed);
    // backLeftMotor.set(ControlMode.PercentOutput, speedY + speedX);
  }

  public void FLMset(double speed) {
    if (getDeadzone(speed))
      speed = 0;

    frontLeftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void FRMset(double speed) {
    if (getDeadzone(speed))
      speed = 0;

    frontRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void BLMset(double speed) {
    if (getDeadzone(speed))
      speed = 0;

    backLeftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void BRMset(double speed) {
    if (getDeadzone(speed))
      speed = 0;

    backRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void driveAll(double speed) {
    frontLeftMotor.set(ControlMode.PercentOutput, speed);
    frontRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getDeadzone(double speed) {
    return Math.abs(speed) < RobotMap.DEADZONE;
  }

  /**
   * give an explanation of what this does Research may be required
   * 
   * @param F fill these out
   * @param P
   * @param I
   * @param D
   */
  public void setPID(double F, double P, double I, double D) {
    // example
    frontLeftMotor.config_kF(0, // the pid id
        F// the feed forward value
    );
    frontLeftMotor.config_kP(0, P);
    frontLeftMotor.config_kI(0, I);
    frontLeftMotor.config_kD(0, D);

    frontRightMotor.config_kF(0, F);
    frontRightMotor.config_kP(0, P);
    frontRightMotor.config_kI(0, I);
    frontRightMotor.config_kD(0, D);
  }

  // this one isn't super complicated, but I'll give you this one
  public void updatePID() {
    // gets the tick value of the encoders
    // this can be from 0 - 4096 (one full rotation)
    int leftRelativePosition = frontLeftMotor.getSensorCollection().getPulseWidthPosition();
    int rightRelativePosition = frontRightMotor.getSensorCollection().getPulseWidthPosition();
    // updates the motors with this value
    frontLeftMotor.setSelectedSensorPosition(leftRelativePosition);
    frontRightMotor.setSelectedSensorPosition(rightRelativePosition);
  }

  /**
   * This will set the distance that we want the
   * 
   * @param distance the distance away (in ticks)
   */
  public void runPID(double distance) {
    updatePID();
    frontLeftMotor.set(ControlMode.Position, distance);
    frontRightMotor.set(ControlMode.Position, distance);
  }

  /**
   * Checks if the pid is finished
   * 
   * @return whether the pid is still running or not
   */
  public boolean finishedPID() {
    return frontLeftMotor.getClosedLoopError() == 0 && frontRightMotor.getClosedLoopError() == 0;
  }

}

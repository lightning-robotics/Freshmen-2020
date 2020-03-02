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
    setLeftInverted();
    // sets up the magnetic encoder for later use
    elevatorMotor.configSelectedFeedbackSensor(
      FeedbackDevice.CTRE_MagEncoder_Relative, 
      RobotMap.ELEVATOR_PIDX,
      RobotMap.ELEVATOR_UPDATE_TIME
      );
    // has the right motor follow the left since we want the motors
    // to be similar in speed

    // sets limits to the PID
    // It can only go to the max and min established here
    elevatorMotor.configNominalOutputForward(RobotMap.ELEVATOR_MIN_OUTPUT, RobotMap.ELEVATOR_UPDATE_TIME);
    elevatorMotor.configNominalOutputReverse(RobotMap.ELEVATOR_MIN_OUTPUT, RobotMap.ELEVATOR_UPDATE_TIME);
    elevatorMotor.configPeakOutputForward(RobotMap.ELEVATOR_PEAK_OUTPUT, RobotMap.ELEVATOR_UPDATE_TIME);
    elevatorMotor.configPeakOutputReverse(-RobotMap.ELEVATOR_PEAK_OUTPUT, RobotMap.ELEVATOR_UPDATE_TIME);

    // the allowable error of the elevator
    // can be a little wrong since it only has
    // to reach a certain height
    elevatorMotor.configAllowableClosedloopError(
      RobotMap.ELEVATOR_PIDX,
      RobotMap.ELEVATOR_ALLOWABLE_ERROR, // in terms of encoder ticks
      RobotMap.ELEVATOR_UPDATE_TIME
    );


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private static void setLeftInverted() {
    elevatorMotor.setInverted(true);
  }

  /**
   * Sends power to the elevator motors
   * Can't send power to the both of them as they should be equal height
   * @param power the amount of power from [-1,1] to send to the motors
   */
  public void setPower(double power) {
    elevatorMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Establishes the PID for position closed loop.
   * Is used for the runPID method to run the pid
   * @param F the feed forward variable
   * @param P the proportional variable (decrease when it osciallates)
   * @param I the integral variable (useful for osciallating)
   * @param D the derivative variable (useful for more exact positions)
   */
  public void setPID(double F, double P, double I, double D) {
    elevatorMotor.config_kF(RobotMap.ELEVATOR_PIDX, F);
    elevatorMotor.config_kP(RobotMap.ELEVATOR_PIDX, P);
    elevatorMotor.config_kI(RobotMap.ELEVATOR_PIDX, I);
    elevatorMotor.config_kD(RobotMap.ELEVATOR_PIDX, D);
  }

  // will update the position of the robot using the encoders
  private void updatePID() {
    int relativePosition = elevatorMotor.getSensorCollection().getPulseWidthPosition();
    elevatorMotor.setSelectedSensorPosition(relativePosition);
  }

  /**
   * Runs the PID on the elevator 
   * Takes the PID set up by the setPID method
   * Only runs when error isn't zero
   * @param target the target position from the enocders
   */
  public void runPID(int target) {
    updatePID();
    if (!donePID()) elevatorMotor.set(ControlMode.Position, target);
  }

  /**
   * Checks if the PID is finished as error is zero
   * @return if the PID finished
   */
  public boolean donePID() {
    return elevatorMotor.getClosedLoopError() == 0;
  }
}

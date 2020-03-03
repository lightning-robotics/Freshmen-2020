/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // Motor Port Number
  // TODO: put right motor numbers
  public static final int FRONT_RIGHT_MOTOR = 20;
  public static final int BACK_RIGHT_MOTOR = 21;
  public static final int FRONT_LEFT_MOTOR = 22;
  public static final int BACK_LEFT_MOTOR = 23;

  //  public static final int FRONT_RIGHT_MOTOR = 3;
  //  public static final int BACK_RIGHT_MOTOR = 2;
  //  public static final int FRONT_LEFT_MOTOR = 10;
  //  public static final int BACK_LEFT_MOTOR = 9;

  // Drive Controller Port
  public static final int ROBOT_DRIVE_CONTROLLER = 0;
  public static final int ROBOT_DRIVE_YAXIS = 1;
  public static final int ROBOT_DRIVE_XAXIS = 0;
  public static final int ROBOT_DRIVE_XAXIS_2 = 4;

  // Elevator wheel diameter
  public static final double ELEVEATOR_WHEEL_DIAMETER = 4;

  // Mechanism Controller Port
  public static final int ROBOT_MECHANISM_CONTROLLER = 1;

  // spinner motor port
  // TODO: put right motor ports
  public static final int SPINNER_MOTOR = 24;

  // shooter motor ports
  // TODO: put right motor ports
  public static final int SHOOTER_TOP = 25;
  public static final int SHOOTER_BOTTOM = 26;

  // dead zone
  public static final double DEADZONE = 0.3; //why not
  public static final double TRIGGER_DEADZONE = 0.1;

  // elevator motor port
  // TODO: put right motor ports
  public static final int ELEVATOR_MOTOR = 27;
  public static final double ELEVATOR_F = .01;
  public static final double ELEVATOR_P = .03;
  public static final double ELEVATOR_I = .001;
  public static final double ELEVATOR_D = 0.0; 
  public static final int ELEVATOR_ALLOWABLE_ERROR = 20;
  public static final double ELEVATOR_PEAK_OUTPUT = .7;
  public static final double ELEVATOR_MIN_OUTPUT = 0;
  public static final int ELEVATOR_UPDATE_TIME = 30;
  public static final int ELEVATOR_PIDX = 0;
  public static final double DISTANCE_PER_PLUSE = ELEVEATOR_WHEEL_DIAMETER * Math.PI;

  // PID variables for elevator height
  public static final double ELEVATOR_TARGET_HEIGHT = -10;
  public static final double kPElevator = 0.2;
  public static final double kIElevator = 0.0001;
  public static final double kDElevator = 0.0;

  // variables for height of robot, height of target, angle of robot
  public static final double HEIGHT_1 = 7.02;
  public static final double HEIGHT_2 = 23.5;
  public static final double PHETA = 18.123;

  // PID variables for limelight drive distance
  public static final double kPDrive = 0.5;
  public static final double kIDrive = 0.001;
  public static final double kDDrive = 0.0;
  public static final double DISTANCE_TOLERANCE = 5; // error of the pid
  public static final double TARGET_DISTANCE = 50;

  // PID variables for limelight turn angle
  public static final double KPTurn = 0.1;
  public static final double KITurn = 0.001;
  public static final double KDTurn = 0.0;
  public static final double TURN_TOLERANCE = 1; // error of turn
  public static final double TARGET_ANGLE = 0;

  // Intake motor ports
  // TODO: put right motor ports
  public static final int LEFT_INTAKE_MOTOR = 28;
  public static final int RIGHT_INTAKE_MOTOR = 29;
  
  // Intake motor speed
  public static final double INTAKE_SPEED = .30;

  // Autonomous center
  public static final double AUTONOMOUS_CENTER_DRIVE_TIME = 1;
  public static final double AUTONOMOUS_CENTER_DRIVE_SPEED = .3;
  public static final int AUTONOMOUS_CENTER_TURN_TIME = 3;

  // Autonomous right or left
  public static final double AUTONOMOUS_POSITION_DRIVE_TIME = .5;
  public static final double AUTONOMOUS_POSITION_DRIVE_SPEED = .3;
  public static final int AUTONOMOUS_POSITION_TURN_TIME = 3;

  // Autonomous turn speed
  public static final double AUTONOMOUS_TURN_FOR_TIME = .5;
  public static final double AUTONOMOUS_TURN_FOR_TIME_SPEED = .4;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}

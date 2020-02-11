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
  public static final int FRONT_RIGHT_MOTOR = 1;
  public static final int BACK_RIGHT_MOTOR = 5;
  public static final int FRONT_LEFT_MOTOR = 12;
  public static final int BACK_LEFT_MOTOR = 6;

  //  public static final int FRONT_RIGHT_MOTOR = 3;
  //  public static final int BACK_RIGHT_MOTOR = 2;
  //  public static final int FRONT_LEFT_MOTOR = 10;
  //  public static final int BACK_LEFT_MOTOR = 9;

  // Drive Controller Port
  public static final int ROBOT_DRIVE_CONTROLLER = 0;
  public static final int ROBOT_DRIVE_YAXIS = 1;
  public static final int ROBOT_DRIVE_XAXIS = 0;
  public static final int ROBOT_DRIVE_XAXIS_2 = 4;

  // spinner motor port
  public static final int SPINNER_MOTOR = 3;

  // shooter motor ports
  public static final int SHOOTER_TOP = 2;
  public static final int SHOOTER_BOTTOM = 3;

  // dead zone
  public static final double DEADZONE = 0.3; //why not
  public static final double TRIGGER_DEADZONE = 0.1;

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

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}

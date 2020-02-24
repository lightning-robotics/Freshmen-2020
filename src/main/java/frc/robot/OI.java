/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Driving.TestCommand;
import frc.robot.commands.LimelightCommands.LimelightDriveDistance;
import frc.robot.commands.LimelightCommands.LimelightTurnToAngle;
import frc.robot.commands.Mechanisms.ElevatorPullSelfUp;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public OI() {
    // drive to distance button
    JoystickButton driveButton = new JoystickButton(driver, Button.kY.value);
    driveButton.whenPressed(new LimelightDriveDistance().withTimeout(5));

    // turn to face target button
    JoystickButton turnButton = new JoystickButton(driver, Button.kA.value);
    turnButton.whenPressed(new LimelightTurnToAngle().withTimeout(0.5));   //change!! testing only do like 3 or something

    // pratice command to ensure that configuring buttons is working
    JoystickButton testButton = new JoystickButton(driver, Button.kBumperRight.value);
    testButton.whenPressed(new TestCommand(3 * 4096).withTimeout(3));

    JoystickButton elevatorButton = new JoystickButton(mechanism, Button.kY.value);
    elevatorButton.whenPressed(new ElevatorPullSelfUp(RobotMap.ELEVATOR_DISTANCE));
    System.out.println("Buttons configured");
  }

  // Declaration of driver xbox controller blah blah blah and whatever
  public XboxController driver = new XboxController(RobotMap.ROBOT_DRIVE_CONTROLLER);
  public XboxController mechanism = new XboxController(RobotMap.ROBOT_MECHANISM_CONTROLLER);

  public double getControllerAxis(XboxController controller, int axis) {
    return controller.getRawAxis(axis);
  }

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three od.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}

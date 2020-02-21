/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Mechanisms.ElevatorUp;
import frc.robot.commands.Mechanisms.Spinner;
import frc.robot.commands.Shooter.ShooterGoalOfTheDay;
import frc.robot.commands.Shooter.ShooterSpeed;
import frc.robot.commands.Autonomous.AutoCenter;
import frc.robot.commands.Autonomous.AutoLeftOrRight;
import frc.robot.commands.Driving.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;

  public static final DriveTrain driveTrain = new DriveTrain();
  public static final Limelight limelight = new Limelight();
  public static final Elevator elevator = new Elevator();
  public static final Shooter shooter = new Shooter();

  Command tankDrive = new TankDrive();
  Command shooterGoalOfTheDay = new ShooterGoalOfTheDay();
  Command spinner = new Spinner();
  ElevatorUp elevatorUp = new ElevatorUp();
  ShooterSpeed shooterSpeed = new ShooterSpeed();
  SequentialCommandGroup m_autonomousCommand;
  SendableChooser<SequentialCommandGroup> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();

    m_chooser.addOption("center", new AutoCenter());
    m_chooser.addOption("left or right", new AutoLeftOrRight());
    
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    // set the back motors to follow the front
    DriveTrain.backLeftMotor.follow(DriveTrain.frontLeftMotor);
    DriveTrain.backRightMotor.follow(DriveTrain.frontRightMotor);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    tankDrive.cancel();
    spinner.cancel();
    // shooterGoalOfTheDay.cancel();
    shooterSpeed.cancel();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another cokmmand, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    // starting the drive function
    tankDrive.start();
  }

  /**
   * This function is called periodically during operator control.$
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    spinner.start();
    // shooterGoalOfTheDay.start();
    elevatorUp.schedule();
    shooterSpeed.schedule();

    // TODO: Intake
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

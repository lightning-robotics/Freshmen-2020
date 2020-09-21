/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterSpeed extends CommandBase {
  private double topSpeed = -.5;
  private final double speedChange = .05;

  // UsbCamera USBCamera = new UsbCamera("theCamera", 0);
  // MjpegServer mjpegServer = new MjpegServer("theServer", 5801);
  // CvSource outputStream = new CvSource("name", PixelFormat.kMJPEG, 640, 480, 30);
  // MjpegServer mjpegServer2 = new MjpegServer("mjpegServer2", 5801);
  // CvSink cvSink = new CvSink("opencv_USB Camera 0");


    /**
   * Creates a new ShooterSpeed.
   */
  public ShooterSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    // mjpegServer.setSource(USBCamera);
    // cvSink.setSource(USBCamera);
    // mjpegServer2.setSource(outputStream);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.shooter.stopShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.oi.mechanism.getYButton()) topSpeed += speedChange; // increases top speed
    if (Robot.oi.mechanism.getAButton()) topSpeed -= speedChange; // decreases top speed

    Robot.shooter.setBottomMotor(.3);

    // setting speed to the motors
    if (Robot.oi.mechanism.getBumper(Hand.kRight)) {
      System.out.println("Top speed" + topSpeed);
      Robot.shooter.setTopMotor(topSpeed);
    } else {
      initialize();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

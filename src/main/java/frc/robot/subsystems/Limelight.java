/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Limelight extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry tv = table.getEntry("tv");


  /**
   * Creates a new Limelight.
   */
  public Limelight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getXAngle() {
    return tx.getDouble(0.0);
  }

  public double getYAngle() {
    return ty.getDouble(0.0);
  }

  public boolean isVisible() {
    return tv.getDouble(0.0) == 1;
  }

    /**
   * gets the distance from robot to target
   * 
   * @return estimated distance
   */
  public double getDistanceFrom() {
    double heightDegree = getXAngle();
    double radiansA1 = RobotMap.PHETA * Math.PI / 180;
    double radiansA2 = heightDegree * Math.PI / 180;

    return (RobotMap.HEIGHT_2 - RobotMap.HEIGHT_1) / Math.tan(radiansA1 + radiansA2);
  }
}

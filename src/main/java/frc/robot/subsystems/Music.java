/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Music extends SubsystemBase {

  Orchestra orca = new Orchestra();

  /**
   * Creates a new Music.
   */
  public Music() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void playVista() {
    orca.loadMusic("src\\main\\deploy\\vista.chrp");
    orca.play();
  }

  public void addMotor(TalonFX motor) {
    orca.addInstrument(motor);
  }

}

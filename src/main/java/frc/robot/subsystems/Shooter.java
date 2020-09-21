/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  private static TalonSRX shooter1 = new TalonSRX(RobotMap.SHOOTER_TOP); //top
  private static TalonSRX shooter2 = new TalonSRX(RobotMap.SHOOTER_BOTTOM);
  /**
   * Creates a new Shooter.
   */
  public Shooter() {

    shooter1.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

    shooter1.config_kP(0, .07); // 0,07 for comp
		shooter1.config_kI(0, .0002); // 0.001 for comp
		shooter1.config_kD(10, 0); // 1.5		
	
		// Max RPM is 4,800, max change in native units per 100ms is 13,140
		// 1023 / 13,140 = 0.078
		//RobotMap.flywheel.setF(0.078);
		shooter1.config_kF(0, 0.0235); // 0.03122 0.015 COMP BOT
				
    shooter1.configNominalOutputForward(+0f);
    shooter1.configNominalOutputReverse(-0f);
    shooter1.configPeakOutputReverse(+1f); // Only drive forward
    shooter1.configPeakOutputForward(+1f); 
		shooter1.set(ControlMode.PercentOutput, 0);
		shooter1.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTopMotor(double speed) {

    shooter1.set(ControlMode.Velocity, speed);

  }

  public void setBottomMotor(double speed) {
    shooter2.set(ControlMode.PercentOutput, speed);
  }

  public void shootDistance(double distance) {
    double tSpeed = topSpeedDistance(distance);
    setTopMotor(tSpeed);
  }

  /**
   * 
   * @param distance Distance in ticks
   * @return speed of top motor
   */
  private double topSpeedDistance(double distance) {
    // TODO: Find this equation
    return 5 * Math.pow(distance, 2) + 4 * distance;
  }

  public void stopShooter() {
    shooter1.set(ControlMode.PercentOutput, 0);
    shooter2.set(ControlMode.PercentOutput, 0);
  }
}

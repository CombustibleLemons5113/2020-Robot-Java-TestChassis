/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.util.Units;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  
  public WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.kFLChassis);
  public WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.kFRChassis);
  public WPI_TalonFX leftSlave = new WPI_TalonFX(Constants.kBLChassis);
  public WPI_TalonFX rightSlave = new WPI_TalonFX(Constants.kBRChassis);
  private SpeedControllerGroup leftSide = new SpeedControllerGroup(leftMaster, leftSlave);
  private SpeedControllerGroup rightSide = new SpeedControllerGroup(rightMaster, rightSlave);
  private DifferentialDrive driveBase = new DifferentialDrive(leftSide, rightSide);
  private AHRS gyro = new AHRS(SPI.Port.kMXP, (byte)60);

  public DriveTrain() {

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSide.setInverted(false);
    rightSide.setInverted(false);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    
    gyro.enableLogging(true);
    System.out.println(gyro.getFirmwareVersion());
    //gyro.isCalibrating();
    System.out.println(gyro.isRotating());

  
  }
  public DifferentialDriveWheelSpeeds getSpeeds() 
  {
    return (new DifferentialDriveWheelSpeeds(
      leftMaster.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(2.0), 
      (-1) * rightMaster.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(2.0))); // change the wheel radius
  }
  public void driveCartesian(double left, double right) {
    //System.out.println("here");
    driveBase.tankDrive(left, right);
    //System.out.println(gyro.getAngle());
    
    /*
    if(getSpeeds().leftMetersPerSecond != 0.0 && getSpeeds().rightMetersPerSecond != 0.0)
    {
      System.out.println("************");
      System.out.println("gryo: " + gyro.getAngle());
      System.out.println("left: " + getSpeeds().leftMetersPerSecond);
      System.out.println("right: " + getSpeeds().rightMetersPerSecond);
    }
    */

    //System.out.println("Left Master Value: " + leftMaster.getSelectedSensorPosition());
    //System.out.println("Right Master Value: " + rightMaster.getSelectedSensorPosition());
  }


}

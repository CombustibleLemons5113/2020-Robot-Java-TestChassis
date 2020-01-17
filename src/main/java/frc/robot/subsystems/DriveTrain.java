/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  private WPI_TalonFX leftMaster = new WPI_TalonFX(Constants.frontLeftID);
  private WPI_TalonFX rightMaster = new WPI_TalonFX(Constants.rightLeftID);
  private WPI_TalonFX leftSlave = new WPI_TalonFX(Constants.backLeftID);
  private WPI_TalonFX rightSlave = new WPI_TalonFX(Constants.backRightID);
  private SpeedControllerGroup leftSide = new SpeedControllerGroup(leftMaster, leftSlave);
  private SpeedControllerGroup rightSide = new SpeedControllerGroup(rightMaster, rightSlave);

  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private DifferentialDrive driveBase = new DifferentialDrive(leftSide, rightSide);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(3)); //ADD WITDH OF ROBOT FROM WHEEL TO WHEEL
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  Pose2d pose;

  public DriveTrain() {

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSide.setInverted(false);
    rightSide.setInverted(false);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
  
  }

  public void driveCartesian(double left, double right) {
    driveBase.tankDrive(left, right);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return (new DifferentialDriveWheelSpeeds(
      leftMaster.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(2.0), 
      rightMaster.getSelectedSensorVelocity() / 7.29 * 2 * Math.PI * Units.inchesToMeters(2.0))); // change the wheel radius
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getSpeeds().leftMetersPerSecond, getSpeeds().rightMetersPerSecond);
  }
}

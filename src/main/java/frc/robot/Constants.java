/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class ChassisConstants {

		//CHASSIS MOTOR PORTS
		public static final int kFLChassis = 12;
		public static final int kFRChassis = 22;
		public static final int kBLChassis = 11;
		public static final int kBRChassis = 21;

		//CHARACTERIZATION VALUES FOR CHASSIS
		public static final double kTrackWidth = 0.4464852526536931;
		public static final double kSChassis = 0.0741;
		public static final double kVChassis = 2.87;
		public static final double kAChassis = 0.289;
		public static final double kPDriveVel = 0.45;

		//AUTON HELPERS
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
		public static final double kRamseteB = 2;
		public static final double kRamseteZ = 0.7;

	}

	public static final class IntakeConstants {

		//INTAKE PISTON AND MOTOR *********** PORTS NEED UPDATING
		public static final int kIntakeMotor = 40;
		public static final int kLIntakePiston = 0;
		public static final int kRIntakePiston = 0;

		//public static final double kIntakeSpeed = 0.75;

	}

	public static final class IndexerConstants {

		//INDEXER MOTORS NEED TO BE ADDED

		public static final int kExtIndexMotor = 41;
		public static final int kMidIndexMotor = 42;
	}

	public static final class HopperConstants {

		//HOPPER MOTORS
		public static final int kHopperMotor = 43;

	}
	
	public static final class TurretConstants {

		//TURRET MOTOR PORT
		public static final int kTurretMotor = 44; 
		
		
			}

	public static final class LimelightConstants {

		//LIMELIGHT DISTANCE CALCULATION *********** ALL NEED UPDATING
		public static final double groundToLimelight = Units.inchesToMeters(24);
		public static final double groundToTarget = Units.inchesToMeters(50);
		public static final double limelightAngleDegress = 25;

	}
	
	public static final class ShooterConstants {

		//ALL NEED UPDATING

		//SHOOTER MOTOR PORTS
		public static final int kLShooterMotor = 31;
		public static final int kRShooterMotor = 32;
		
		//HOOD PISTON PORTS
		public static final int kRHoodPiston = 0;
		
		public static final int kLHoodPiston = 0;

		//S AND V VALUES FOR SHOOTER
		public static final double kSShooter = 0;
		public static final double kVShooter = 0;

		//PID VALUES FOR SHOOTER
		public static final double kShooterkP = 0;
		public static final double kShooterkI = 0;
		public static final double kShooterkD = 0;

	}

	public static final class ClimberConstants {

		//CLIMBER MOTOR AND SPEED *********** NEED TO BE UPDATED
		public static final int kClimber = 0;
		public static final double kClimbSpeed = 0.25;

	}


		

	
	
	
	


}

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.DriveTrain;

//Import the path files ***************************************************
public class FollowPath extends CommandBase
{
    private double kp = 0.01, ki = 0.002, kd = 0.1;
    private PIDController pidControl = new PIDController(kp, ki, kd);
  
    //Define a ramsete command
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(3)); //ADD WITDH OF ROBOT FROM WHEEL TO WHEEL
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private DifferentialDriveWheelSpeeds postPIDwheelSpeeds;
    private RamseteController ramsete = new RamseteController();
    private ChassisSpeeds speeds;
    

    private double kS, kV, kA;

    private Trajectory Traj;

    Pose2d pose;
    Pose2d poseRef;

    public FollowPath()
    {
        addRequirements(Robot.driveTrain);

        boolean validPath = true;
        Trajectory trajectory = null;
        String trajectoryJSON = "path/PATHNAME.wpilib.json";

        try
        {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch(IOException ex)
        {
            validPath = false;
            DriverStation.reportError("Ur Mom Gay... Also unable to open trajectory lol", ex.getStackTrace());
        }

        if(validPath)
        {
            Traj = trajectory;
        }
    }


    public Rotation2d getHeading() 
    {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }
    
    
      
    
    public Pose2d getPoseRef()
    {
       return Traj.getInitialPose();  
    }
    /*
    public double getPoseRefLinVelocity() //meters per second
    {
        
    } 
      
    public double getPoseRefAngVelocity() //radians per second
    {
    
    }
    
    */
    @Override
    public void execute() 
    {
      // This method will be called once per scheduler run
      //pose = odometry.update(getHeading(), Robot.driveTrain.getSpeeds().leftMetersPerSecond, Robot.driveTrain.getSpeeds().rightMetersPerSecond);
      //poseRef = getPoseRef();
      //speeds = ramsete.calculate(pose, poseRef, getPoseRefLinVelocity(), getPoseRefAngVelocity());
        
      //wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        
      //postPIDwheelSpeeds = ;
        
      //postCharacterizationWheelSpeeds = ;
    
        System.out.print(Traj.getStates());
        
    }
}
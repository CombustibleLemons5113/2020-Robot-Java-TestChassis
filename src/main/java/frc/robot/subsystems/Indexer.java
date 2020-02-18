package frc.robot.subsystems;

//imports for the SparkMax's
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import for the Digital sensor aka beam break sensor 
import edu.wpi.first.wpilibj.InterruptableSensorBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANError;


import static frc.robot.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {

    //instantiate beam break sensors
    public int initBallCount = 3;  //starting ball count is 3
    public CANSparkMax inMotor = new CANSparkMax(initBallCount, null); //Arbitrary Arguments, just for now.
    public CANSparkMax outMotor = new CANSparkMax(initBallCount, null);

    //Digital Sensor port for Beambreak
    public DigitalInput beamBreakIn = new DigitalInput(initBallCount); //apparently beamBreak can count as a digital input, but with each input there needs to be an output.
    public DigitalOutput beamBreakOut = new DigitalOutput(initBallCount);

    //beambreak2 is the sensor that is closest to the turret
    public DigitalInput beamBreakIn2 = new DigitalInput(initBallCount); 
    public DigitalOutput beamBreakOut2 = new DigitalOutput(initBallCount);


    public Indexer() 
    {
        //instantiate the motors
        inMotor.restoreFactoryDefaults();
        outMotor.restoreFactoryDefaults();

        inMotor.follow(outMotor);
    }

    public int getBallCount() 
    {
        //This is the code for the beam break sensor. Ported over to Java. 
        if(beamBreakIn.get())
        {
            //unbroken val
            return initBallCount; //Have to find the correlative values for the inputs
        }
        else
        {
            //broken val
            return initBallCount + 1;
        }
    }

}
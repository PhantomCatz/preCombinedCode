package frc.Mechanisms;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.CatzLog;
import frc.robot.Robot;

public class LiDAR
{
    public boolean lidarDataCollectionOn = false;

    private I2C lidar;

    public double range    = -1.0;
    public double minRange = 0.0;
    public double maxRange = 0.0;

    //registers in hex from LiDAR Documentation : https://static.garmin.com/pumac/LIDAR-Lite%20LED%20v4%20Instructions_EN-US.pdf
    private final int LIDAR_DEFAULT_ADDRESS    = 0x62;
    private final int REGISTER_FACTORY_RESET   = 0xE4;
    private final int REGISTER_ACQ_COMMANDS    = 0x00;
    private final int REGISTER_STATUS          = 0x01;
    private final int REGISTER_FULL_DELAY_HIGH = 0x11;
    private final int REGISTER_FULL_DELAY_LOW  = 0x10;

    //delay between status reads during distance measurement
    private final double LIDAR_STATUS_LOOP_DELAY_SEC = 0.03;

    private final double CONV_CM_TO_IN = 0.3937;

    //variables for getLiDARRange()
    private byte       mask;
    private ByteBuffer regValue;
    private int        distInBits;
    private int        value;
    private boolean    done;

    private boolean runDebugOnce;

    public boolean inRange = false;

    public boolean manualOveride = false;

    Thread liDARThread;
    public final double LIDAR_THREAD_DELAY = 0.005;

    private int lidarTraceID = 0;
    private CatzLog data;
    private double inRangeDouble = 0.0;

    private boolean waitForNextMeasurement = true;
    private double waitForNextMeasurementDouble = 1.0;

    

    public LiDAR()
    {
        //initialization for LiDAR
        lidar = new I2C(I2C.Port.kMXP, LIDAR_DEFAULT_ADDRESS);
        
        //factory resets LiDAR
        lidar.write(REGISTER_FACTORY_RESET, 0x01);

        //----------------setup variables for getLiDARRange()-------------------------\\
        //mask set to 0000 0001 to compare status register value to
        mask = Byte.decode("0x01");

        //buffer allocated 1 byte
        regValue = ByteBuffer.allocate(0x01);


        runDebugOnce = false;

        setValidRanges(0.0, 7.8);

        startLiDAR();
    }


    public void startLiDAR() 
    {

        liDARThread = new Thread(() -> //start of thread
        {
            while(true)
            {
                lidarTraceID = 200;
                //constantly updates LiDAR measurement, in its own thread due to getting loop time overrun warnings.
                getLidarRange();

                if(lidarDataCollectionOn == true)
                {
                    data = new CatzLog(Robot.dataCollectionTimer.get(),(double)lidarTraceID,  minRange,
                                                                                          range,
                                                                                          maxRange,
                                                                                          inRangeDouble,
                                                                                          waitForNextMeasurementDouble,
                                                                                          -999.0, -999.0, -999.0, -999.0,
                                                                                          -999.0, -999.0, -999.0, -999.0,
                                                                                          -999.0);                      
                    Robot.dataCollection.logData.add(data);
                }
                

            }
        }); 
        //end of thread
        liDARThread.start();
    }



    public void getLidarRange()
    {

        done = false;
        int waitTime = (int)(0.3/LIDAR_STATUS_LOOP_DELAY_SEC);
        int timeoutCounter = 0;

        //Writes to the LiDAR to start a measurement
        lidar.write(REGISTER_ACQ_COMMANDS, 0x04);

        

        /*
            Loop to wait for status register to return LOW instead of HIGH, which means that the measurement has been
            completed. This is done by reading the register status, which will give 0x01 (0000 0001) if HIGH and 0x00
            (0000 0000) if low. Then it is compared to the mask, which has a value of 0x01 (0000 0001) with the & 
            command, which compares each byte and only returns 1 if both values are 1. So if the register reads 
            HIGH, then the value will be set to 0x01 (0000 0001), and if the register reads LOW, value will be set to
            0x00 (0000 0000) which means that it has finished taking the measurement and will end the loop.
        */
        while(done == false)
        {
            regValue.clear();

            lidar.read(REGISTER_STATUS, 0x01, regValue);
      
            value = mask & regValue.get();

            if (value == 0x00)
            {   
                regValue.clear();
                lidar.read(REGISTER_FULL_DELAY_HIGH, 0x01, regValue);
                distInBits = regValue.get() << 8;
        
                regValue.clear();
                lidar.read(REGISTER_FULL_DELAY_LOW, 0x01, regValue);
                distInBits = distInBits | regValue.get();
        
                range = ((double)distInBits * CONV_CM_TO_IN); 
                done = true;
                waitForNextMeasurement = false;
                waitForNextMeasurementDouble = 0.0;
            }
            else //measurement is not ready
            {
                timeoutCounter++;
                if(timeoutCounter >= waitTime)
                {
                    done = true;
                    range = -1.0;
                }   
            }
            
            Timer.delay(LIDAR_STATUS_LOOP_DELAY_SEC);
        }
   
    }

    public void setValidRanges(double minRan, double maxRan)
    {
        minRange = minRan;
        maxRange = maxRan;
    }

    public boolean isInRange()
    {
        if(manualOveride == true)
        {
            manualOveride = false;
            return true;
        }
        else
        {
            if(waitForNextMeasurement == false)
            {
                if(range < maxRange && range > minRange)
                {
                    inRange = true;
                    inRangeDouble = 1.0;
                }
                else
                {
                    inRange = false;
                    inRangeDouble = 0.0;
                }
                waitForNextMeasurement = true;
                waitForNextMeasurementDouble = 1.0;
            }
            else
            {
                inRange = false;
                inRangeDouble = 0.0;
            }

            return inRange;

        } 
    }

  //-------------------SmartDashboard-------------------\\

    public void smartDashboardLiDAR_Debug()
    {   
        //to only get the port and device address once
        if(runDebugOnce)
        {
            SmartDashboard.putNumber("LiDAR Port", lidar.getPort());
            SmartDashboard.putNumber("LiDAR Device Address", lidar.getDeviceAddress());

            runDebugOnce = false;
        }
        SmartDashboard.putBoolean("cargoInRange", inRange);

    }

    public void smartDashboardLiDAR()
    {
        boolean lidarFault = false;
        if(range == -1)
        {
            lidarFault = true;
        }
        SmartDashboard.putNumber("LiDAR Range", range); 
        SmartDashboard.putBoolean("LiDAR Fault", lidarFault); 
    }

}
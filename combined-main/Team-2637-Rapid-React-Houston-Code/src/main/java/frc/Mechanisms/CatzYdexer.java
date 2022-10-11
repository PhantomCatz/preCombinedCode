package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.*;
import frc.robot.Robot;


public class CatzYdexer 
{    
    public boolean dataCollectionYdexerON = false;
    // Thread
    private       Thread yDexerThread;
    private final double YDEXER_THREAD_DELAY = 0.005;

    public Timer ydexertimer;

    // Data Collection
    private CatzLog data;

    

    private int traceID = 0;

    private double yDexerOnDouble       = 0.0;
    private double btmSensorStateDouble = 0.0;
    private double topSensorStateDouble = 0.0;
    private double cargoInRangeDouble   = 0.0;

    // Sensor variables
   
    public boolean cargoInRange = false;

    //limit switch variables
    public SparkMaxLimitSwitch ydexerBtmLimitSwitch;  
    public SparkMaxLimitSwitch ydexerTopLimitSwitch;  
      
    private final boolean TOP_BALL_PRESENT     = false;
    private final boolean TOP_BALL_NOT_PRESENT = true;

    private final boolean BTM_BALL_PRESENT     = true;
    private final boolean BTM_BALL_NOT_PRESENT = false;

    private boolean btmSensorState = TOP_BALL_NOT_PRESENT;
    private boolean topSensorState = TOP_BALL_NOT_PRESENT;

    // Motor and Motor Controller
    public CANSparkMax yDexerMtrCtrl;

    private final int INDEXER_MC_CAN_ID        = 40;
    private final int INDEXER_MC_CURRENT_LIMIT = 60;

    public final double YDEXER_SHOOT_MTR_PWR    = -0.55;
    public final double YDEXER_INDEXING_MTR_PWR = -0.55;
    public final double YDEXER_MOTOR_POWER_OFF  =  0.0;
    public final double YDEXER_SHOOT_SECOND_MTR_PWR = YDEXER_SHOOT_MTR_PWR + 0.2;//0.14;
    public final double YDEXER_FAR_RPM_SHOOT_MTR_PWR = -0.8;
    public final double YDEXER_FAR_RPM_SHOOT_MTR_PWR_SECOND = -0.8;

    public double ydexerShootPower = 0.0;
    public double ydexerSecondShootPower = 0.0;
    

    private final int IGNORE_LIMIT_SWITCH_COUNT_TIME = ((int)(0.09 / YDEXER_THREAD_DELAY)) + 1;
    private final int YDEXER_SPEED_CHANGE_CNT = ((int)(0.02 / YDEXER_THREAD_DELAY)) + 1; ;

    private int yDexerCount = 0;

    private final boolean YDEXER_SHOOT_ALL = true;
    private final boolean YDEXER_SHOOT_ONE = false;

    // Ball Management
    public  boolean yDexerOn  = false;
    private boolean shooterOn = false;   
    private boolean turnShooterOff = false;
    private boolean checkStopConditions = true;
    private boolean yDexerShootMode = YDEXER_SHOOT_ALL;

    public DigitalInput beamBreakBtm;
    public DigitalInput beamBreakTop;
    private final int BEAM_BREAK_DIO_PORT_BTM = 1;
    private final int BEAM_BREAK_DIO_PORT_TOP = 2;

    public CatzYdexer() 
    {

        //setup for motor controller
        yDexerMtrCtrl = new CANSparkMax(INDEXER_MC_CAN_ID, MotorType.kBrushless); 

        yDexerMtrCtrl.restoreFactoryDefaults();
        yDexerMtrCtrl.setIdleMode(IdleMode.kBrake);
        yDexerMtrCtrl.setSmartCurrentLimit(INDEXER_MC_CURRENT_LIMIT);
        
        //setup for limit switch
        ydexerBtmLimitSwitch = yDexerMtrCtrl.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        ydexerTopLimitSwitch = yDexerMtrCtrl.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        //no stop on detection
        ydexerBtmLimitSwitch.enableLimitSwitch(false); 
        ydexerTopLimitSwitch.enableLimitSwitch(false);

        
        
        //setup for beamBreak
        beamBreakBtm = new DigitalInput(BEAM_BREAK_DIO_PORT_BTM);
        beamBreakTop = new DigitalInput(BEAM_BREAK_DIO_PORT_TOP);
        
        ydexertimer = new Timer();
        ydexertimer.reset();
        ydexertimer.start();

        startYDexerThread();
    }
 

    public void startYDexerThread()
    {
        yDexerThread = new Thread(() ->
        {     
            yDexerOn = false;
            yDexerCount = 0;
            checkStopConditions = true;
            
            while(true)
            {  
                if(shooterOn)
                {
                    traceID = 300;

                    yDexerOn = true;
                    
                    yDexerMtrCtrl.set(ydexerShootPower);
                    

                    if (yDexerShootMode == YDEXER_SHOOT_ALL)
                    {
                        traceID = 310;

                        if(yDexerCount > YDEXER_SPEED_CHANGE_CNT)
                        {
                            traceID = 312;

                            yDexerMtrCtrl.set(ydexerSecondShootPower);
                        }

                        if (turnShooterOff == true)
                        {
                            traceID = 314;

                            yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_OFF);
                            yDexerOn       = false;
                            turnShooterOff = false;  
                            shooterOn      = false;
                        }    
                    }
                    else //Shoot 1 Cargo
                    {
                        traceID = 320;

                        /*--------------------------------------------------------------------------------
                        *  If there is:
                        *    2 cargo stored       : Turn off when top limit is true 
                        *    1 cargo stored at Top: Turn off after timeout
                        *    1 cargo stored at Btm: Ignore next top limit switch true & Turn off after timeout 
                        *-------------------------------------------------------------------------------*/
                    }

                    if (turnShooterOff == true)
                    {
                        traceID = 330;

                        yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_OFF);
                        yDexerOn       = false;
                        turnShooterOff = false;  
                    }
                }
                else
                {
                    /*------------------------------------------------------------------------------------
                    *  We are indexing cargo.  Grab sensor status
                    *
                    *  cargoNearYdexerEntrance  S-Btm  S-Top
                    *           0                 0      0      Do nothing
                    *           0                 1      0      Do nothing
                    *           0                 0      1      Do nothing
                    *           0                 1      1      Do nothing
                    *           1                 0      0      MOVE BALL - No cargo currently stored
                    *           1                 1      0      MOVE BALL - ONE cargo currently stored AND SPACE AVAILABLE AT TOP
                    *           1                 0      1      Do nothing - Cargo at top
                    *           1                 1      1      Do nothing - Cargo at top
                    *-----------------------------------------------------------------------------------*/
                   /* btmSensorState = ydexerBtmLimitSwitch.isPressed();
                    topSensorState = ydexerTopLimitSwitch.isPressed();*/

                    traceID = 340;

                    btmSensorState = ydexerBtmLimitSwitch.isPressed();
                    topSensorState = beamBreakTop.get();

                    cargoInRange = Robot.lidar.isInRange();

                    if(yDexerOn == true)
                    {
                        /*--------------------------------------------------------------------------------
                        *  YDexer is ON, so check conditions to see if it is time to turn YDexer OFF. 
                        *  Since Cargo can still be on bottom sensor on next iteration after turning
                        *  YDexer ON, we need to wait for Cargo to move off of bottom sensor before
                        *  checking to turn YDexer OFF.
                        *-------------------------------------------------------------------------------*/
                        traceID = 350;

                        if(checkStopConditions == false)
                        {
                            traceID = 352;

                            /*if(btmSensorState == BALL_NOT_PRESENT)
                            {
                                checkStopConditions = true;
                            }*/

                            //checks if we have to timeout in case the btm sensor state never changes 
                            if(yDexerCount > IGNORE_LIMIT_SWITCH_COUNT_TIME)
                            {
                                traceID = 354;

                                checkStopConditions = true;
                            }
                            yDexerCount++;                         
                        }

                        if(checkStopConditions == true)
                        {   
                            traceID = 360;

                            if(btmSensorState == BTM_BALL_PRESENT || topSensorState == TOP_BALL_PRESENT)
                            { 
                                traceID = 362;

                                yDexerMtrCtrl.set(YDEXER_MOTOR_POWER_OFF);
                                yDexerOn       = false;
                            }
                        }
                    }
                    else
                    {
                        /*--------------------------------------------------------------------------------
                        *  YDexer is OFF, so check if Cargo is in range.  If it is in range, Turn YDexer ON
                        *-------------------------------------------------------------------------------*/
                        traceID = 370;

                        if(cargoInRange == true)
                        {
                            traceID = 372;

                            if(topSensorState == TOP_BALL_NOT_PRESENT)
                            {
                                traceID = 374;

                                yDexerMtrCtrl.set(YDEXER_INDEXING_MTR_PWR);
                                yDexerOn    = true;
                                yDexerCount = 0;

                                if(btmSensorState == BTM_BALL_PRESENT)
                                {
                                    traceID = 376;

                                    checkStopConditions = false;
                                }

                            }
                        } 
                    }
                }

                if(dataCollectionYdexerON == true)
                {
                    booleanDataLogging();

                    data = new CatzLog(Robot.dataCollectionTimer.get(), (double)traceID, btmSensorStateDouble,
                                                                                        topSensorStateDouble,
                                                                                        yDexerOnDouble,
                                                                                        cargoInRangeDouble,
                                                                                        (double)yDexerCount,
                                                                                        yDexerMtrCtrl.getAppliedOutput(),
                                                                                        -999.0, -999.0, -999.0, -999.0,
                                                                                        -999.0, -999.0, -999.0, -999.0);       
                                                                                
                    Robot.dataCollection.logData.add(data);
                }
            
               Timer.delay(YDEXER_THREAD_DELAY); 
            }
        }); //end of thread
        yDexerThread.start();
    }
    
    
    /*--------------------------------------------------------------------------------
    *
    *  Methods to notify Ydexer of Shooter state 
    *
    *-------------------------------------------------------------------------------*/
    public void setShooterOn()
    {
        shooterOn      = true;
        turnShooterOff = false;
        yDexerCount = 0;
    }
    
    public void setShooterOff()
    {
        turnShooterOff = true;  
    }

    public boolean getBtmSensorPressed()
    {
        btmSensorState = ydexerBtmLimitSwitch.isPressed();
        if(btmSensorState == BTM_BALL_PRESENT)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public boolean getTopSensorPressed()
    {
        topSensorState = beamBreakTop.get();
        if(topSensorState == TOP_BALL_PRESENT)
        {
            return true;
        }
        else
        {
            return false;
        }
    }



   
    

    /*--------------------------------------------------------------------------------
    *
    *  SmartDashboard
    *
    *-------------------------------------------------------------------------------*/
    public void smartDashboardYdexer()
    {
        SmartDashboard.putBoolean("Cargo In Range", cargoInRange);
        SmartDashboard.putBoolean("BotSensor", btmSensorState);
        SmartDashboard.putBoolean("TopSensor", topSensorState);
    }


    public void smartDashboardYdexer_DEBUG()
    {
        SmartDashboard.putBoolean("ShooterOn", shooterOn);
        SmartDashboard.putBoolean("Ydexer",    yDexerOn);
    }


    private void booleanDataLogging()
    {
        if(yDexerOn == true)
        {
            yDexerOnDouble = 1.0;
        }
        else
        {
            yDexerOnDouble = 0.0;
        }

        if(btmSensorState == BTM_BALL_PRESENT)
        {
            btmSensorStateDouble = 1.0;
        }
        else
        {
            btmSensorStateDouble = 0.0;
        }

        if(topSensorState == TOP_BALL_PRESENT)
        {
            topSensorStateDouble = 1.0;
        }
        else
        {
            topSensorStateDouble = 0.0;
        }

        if(cargoInRange == true)
        {
            cargoInRangeDouble = 1.0;
        }
        else
        {
            cargoInRangeDouble = 0.0;
        }
    }

}
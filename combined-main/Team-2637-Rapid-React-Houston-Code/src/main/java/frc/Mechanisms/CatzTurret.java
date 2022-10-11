package frc.Mechanisms;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.DataLogger.*;

public class CatzTurret
{
    public boolean turretDataCollectionOn      = false;
    private final int TURRET_MC_CAN_ID = 20;

    public CANSparkMax turretMC;

    public boolean inAutonomous = false;

    private final double TURRET_THREAD_PERIOD = 0.010;
    private Thread turretThread;

    public final int TURRET_STATE_MANUAL            = 0;
    public final int TURRET_STATE_MOVE_TO_POSITION  = 1;
    public final int TURRET_STATE_AUTON_AUTO_TARGET = 2;

    public int turretState = TURRET_STATE_MANUAL;

    private static final double ZERO_VELOCITY = 0.0;

    private final double MAX_POSITION  =  100.0;
    private final double MIN_POSITION  = -100.0;
    private final double HOME_POSITION = 0.0;

    private final double TURRET_POSITIVE_MAX_RANGE =  155.0;
    private final double TURRET_NEGATIVE_MAX_RANGE = -155.0;

    private final int TURRET_GEARBOX_VERSA_1          = 5;
    private final int TURRET_GEARBOX_VERSA_2          = 4;
    private final int TURRET_GEARBOX_TURRET_GEAR      = 140 / 10;
    private final int TURRET_GEAR_REDUCTION           = TURRET_GEARBOX_VERSA_1 * TURRET_GEARBOX_VERSA_2 * TURRET_GEARBOX_TURRET_GEAR; //Equals 280

    private final int TURRET_MC_CURRENT_LIMIT      = 40;

    private RelativeEncoder turretEncoder;
    private double turretPositionDeg;
    
    private final double MIN_TURRET_PWR =  0.1;

    public final double TURRET_LOW_RATE  = 0.15;
    public final double TURRET_HIGH_RATE = 0.9; // if want to set max pwr: TURRET_HIGH_RATE = 1

    private double targetPosOffset      = 1.0; 
    private double targetPos            = 45.0; 
    private double targetMtrPwr         = 0.0;


    private boolean TURN_POSITIVE = true;
    private boolean TURN_NEGATIVE = false;
    private boolean direction     = TURN_POSITIVE;

    private double accelStepSize = 0.0;
    private boolean firstAccelInTargetTracking = false;

    private double turretPosAngleError;

    private final boolean DONT_STOP_ON_LS_HIT = false;
    private final boolean STOP_ON_LS_HIT      =  true;

    private DigitalInput        limitSwitchHome;
    //public SparkMaxLimitSwitch magLimitSwitchLeft;
    //public SparkMaxLimitSwitch magLimitSwitchRight;

    private final int HOME_LIMIT_SWITCH_DIO_PORT = 0;

    private final boolean HOME_LIMIT_SWITCH_PRESSED     = false;
    private final boolean HOME_LIMIT_SWITCH_NOT_PRESSED = true;

   
    private boolean uniqueFirstLogData = false;
   
    private double currentTime = 0.0;
    private Timer turrettimer;

    private final double MOVE_TO_POSITION_CYCLE_TIMEOUT_SEC = 1.0; 
    private double moveToPositionTimeoutCnt = (MOVE_TO_POSITION_CYCLE_TIMEOUT_SEC / TURRET_THREAD_PERIOD) + 1;
    private int moveToPositionTimeoutCntr = 0;        
    
    private double thresholdPositive;
    private double thresholdNegative; 
    private double currentPos;
    private double turretMtrPwr;

    private double pidTurnDecelAngle = 10;
    private double pidTurnDecelRate = 0.5;

    private final boolean TRACKING_ON       = true;
    private final boolean TRACKING_OFF      = false;
    private boolean turretTrackingState     = TRACKING_OFF;
    private boolean turretTrackingStatePrev = TRACKING_OFF;

    private CatzLog data;

    private double traceID;

    public CatzTurret()
    {
        turretMC = new CANSparkMax(TURRET_MC_CAN_ID, MotorType.kBrushless);

        turretMC.restoreFactoryDefaults();      //reset configuration
        turretMC.setIdleMode(IdleMode.kCoast);  //set MC to idle brake mode

        limitSwitchHome       = new DigitalInput(HOME_LIMIT_SWITCH_DIO_PORT);

        //magLimitSwitchLeft    = turretMC.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        //magLimitSwitchRight   = turretMC.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

       /* magLimitSwitchLeft.enableLimitSwitch(STOP_ON_LS_HIT);
        magLimitSwitchRight.enableLimitSwitch(STOP_ON_LS_HIT);*/

        //magLimitSwitchLeft.enableLimitSwitch(DONT_STOP_ON_LS_HIT);
        //magLimitSwitchRight.enableLimitSwitch(DONT_STOP_ON_LS_HIT);

        turretEncoder = turretMC.getEncoder();
        turretEncoder.setPositionConversionFactor(360.0 / TURRET_GEAR_REDUCTION);

        turretMC.setSmartCurrentLimit(TURRET_MC_CURRENT_LIMIT);

        turretEncoder.setPosition(HOME_POSITION);

        turrettimer = new Timer();

        turrettimer.start();

        turretControl();

    }



    /*----------------------------------------------------------------------------------------------
    *
    *  turretControl()
    *
    *---------------------------------------------------------------------------------------------*/
    public void turretControl()
    {  
        resetTurretEncoder();
        
        turretThread = new Thread(() ->
        {

            while(true)
            {
                currentTime = turrettimer.get();

                currentPos = getTurretPositionDeg();
                /*if(currentPos > 90.0)
                {
                    magLimitSwitchLeft.enableLimitSwitch(STOP_ON_LS_HIT);
                }
                else if(currentPos < -90.0)
                {
                    magLimitSwitchRight.enableLimitSwitch(STOP_ON_LS_HIT);
                }
                else
                {
                    magLimitSwitchLeft.enableLimitSwitch(DONT_STOP_ON_LS_HIT);
                    magLimitSwitchRight.enableLimitSwitch(DONT_STOP_ON_LS_HIT);
                }*/

                switch(turretState)
                {
                    case TURRET_STATE_MANUAL:
                        //Do nothing in manual mode
                    break;

                    case TURRET_STATE_AUTON_AUTO_TARGET:

                        autoAimTurret();

                    break;
                    
                    case TURRET_STATE_MOVE_TO_POSITION:
                        currentPos = getTurretPositionDeg();

                        turretPosAngleError = Math.abs(targetPos - currentPos);

                        traceID = 110;
                        
                        if (currentPos >= thresholdNegative && currentPos <= thresholdPositive)
                        {
                            stopTurret();
                            traceID = 120;
                        }
                        else 
                        {
                            traceID = 130;

                            if (moveToPositionTimeoutCntr > moveToPositionTimeoutCnt)
                            {
                                stopTurret();
                                traceID = 140;
                            }
                            else
                            {
                                traceID = 150;
                                if (turretPosAngleError < pidTurnDecelAngle) 
                                {
                                    traceID = 160;
                                    if(Math.abs(turretMtrPwr) > MIN_TURRET_PWR)
                                    {
                                        traceID = 170;
                                        turretMtrPwr = turretMtrPwr * pidTurnDecelRate;

                                        //set motor power to minimum power if it is under minimun while decelerating
                                        if (Math.abs(turretMtrPwr) < MIN_TURRET_PWR)
                                        {
                                            traceID = 180;
                                            if(turretMtrPwr > 0.0)
                                            {
                                                traceID = 181;
                                                turretMtrPwr = MIN_TURRET_PWR;
                                            }
                                            else
                                            {
                                                traceID = 182;
                                                turretMtrPwr = -MIN_TURRET_PWR;
                                            }
                                        }
                                        turretMC.set(turretMtrPwr);
                                    }   
                                }
                                else
                                {
                                    traceID = 190;
                                    if(direction == TURN_POSITIVE)
                                    {
                                        traceID = 191;
                                        if(turretMtrPwr < targetMtrPwr)
                                        {
                                            traceID = 192;
                                            turretMtrPwr = turretMtrPwr + accelStepSize;

                                            if(turretMtrPwr > targetMtrPwr)
                                            {
                                                traceID = 193;
                                                turretMtrPwr = targetMtrPwr;
                                            }
                                            turretMC.set(turretMtrPwr);
                                        }
                                    }
                                    else
                                    {
                                        traceID = 194;
                                        if(turretMtrPwr > targetMtrPwr)
                                        {
                                            traceID = 195;
                                            turretMtrPwr = turretMtrPwr - accelStepSize;

                                            if(turretMtrPwr < targetMtrPwr)
                                            {
                                                traceID = 196;
                                                turretMtrPwr = targetMtrPwr;
                                            }
                                            turretMC.set(turretMtrPwr);
                                        }
                                    }
                                }
                            }
                            moveToPositionTimeoutCntr++;

                        }

                        /*if(turretTrackingStatePrev == TRACKING_ON && turretTrackingState == TRACKING_OFF)
                        {
                            stopTurret();
                        }

                        turretTrackingStatePrev = turretTrackingState;*/
                        

                        if (turretDataCollectionOn == true)
                        {
                            /*if (uniqueFirstLogData == false)
                            {
                
                                data = new CatzLog(targetPos, targetPosOffset,  turretMtrPwr, pidTurnDecelRate, pidTurnDecelAngle, 
                                                                                                                moveToPositionTimeoutCnt, 
                                                                                                                turretEncoder.getPositionConversionFactor(),
                                                    -999.0, -999.0, -999.0, -999.0, -999.0, 
                                                    -999.0, -999.0, -999.0, -999.0 );   //set to 16 values
                                Robot.dataCollection.logData.add(data);
                                uniqueFirstLogData = true;
                
                            }*/

                            data = new CatzLog(currentTime, traceID, targetPos, currentPos, turretPosAngleError, turretMtrPwr, turretMC.getAppliedOutput(),
                                                                                                            turretMC.getOutputCurrent(),
                                                                                                            turretMC.getBusVoltage(), 
                                                                                                            turretEncoder.getVelocity(),
                                                                                                            Robot.vision.getXErrorOffset(),
                                                                                                            Robot.vision.getYErrorOffset(),
                                                                                                            Robot.vision.getDistanceToTarget(), 
                                                                                                            -999.0, -999.0, -999.0);   //set to 16 values
                            Robot.dataCollection.logData.add(data);
                        }

                    break;

                    default:
                    break;
                }

                //checkIfAtHardstop();


                Timer.delay(TURRET_THREAD_PERIOD);
            }
        });

        turretThread.start();
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  setTargetPos()
    *
    *---------------------------------------------------------------------------------------------*/
    public void setTargetPos(double targetAngle)
    {
        setTurretBrakeMode();

        targetPos         = targetAngle;
        thresholdPositive = targetPos + targetPosOffset;
        thresholdNegative = targetPos - targetPosOffset;

        double turretAccelWindow = 0.01; //TBD
        int    turretAccelSteps  = (int)((turretAccelWindow / TURRET_THREAD_PERIOD) + 1);

        currentPos = getTurretPositionDeg();
        turretPosAngleError = Math.abs(targetPos - currentPos);

        //edit deceleration numbers to make rotation smoother and less jerky
        if(turretPosAngleError <= 10.0)
        {
            targetMtrPwr       = 0.2;
            pidTurnDecelRate   = 0.65;
            pidTurnDecelAngle  = 3.0; 

            moveToPositionTimeoutCnt = (1.0 / TURRET_THREAD_PERIOD) + 1;

            accelStepSize = 0.0;
        }
        else if(turretPosAngleError <= 20)
        {
            targetMtrPwr       = 0.6;
            pidTurnDecelRate   = 0.75; 
            pidTurnDecelAngle  = 9.0; 

            moveToPositionTimeoutCnt = (1.0 / TURRET_THREAD_PERIOD) + 1;

            turretAccelWindow = 0.08;
            turretAccelSteps  = (int)((turretAccelWindow / TURRET_THREAD_PERIOD) + 1);
            accelStepSize     = targetMtrPwr / turretAccelSteps;
        }
        else // (turretPosAngleError > 150.0)
        {
            targetMtrPwr       = 0.6;
            pidTurnDecelRate   = 0.7;
            pidTurnDecelAngle  = 11.0;
            
            moveToPositionTimeoutCnt = (1.0 / TURRET_THREAD_PERIOD) + 1;

            turretAccelWindow = 0.08;
            turretAccelSteps  = (int)((turretAccelWindow / TURRET_THREAD_PERIOD) + 1);
            accelStepSize     = targetMtrPwr / turretAccelSteps;
        }
        
        if(targetPos < currentPos)
        {
            direction    = TURN_NEGATIVE;
            targetMtrPwr = -targetMtrPwr;
            turretMtrPwr = -accelStepSize;
        }
        else
        {
            direction    = TURN_POSITIVE;
            turretMtrPwr = accelStepSize;
        }
        
        if(turretTrackingState == TRACKING_ON)
        {
            if(firstAccelInTargetTracking == false)
            {
                firstAccelInTargetTracking = true;
            }
            else
            {
                accelStepSize = 0.0;
            }
        }

        if(accelStepSize == 0.0)
        {
            turretMtrPwr = targetMtrPwr;
        }
        
        moveToPositionTimeoutCntr = 0;

        turretState = TURRET_STATE_MOVE_TO_POSITION;

        turretMC.set(turretMtrPwr);
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  Target Tracking with Limelight
    *
    *---------------------------------------------------------------------------------------------*/

    public void enableAutoAim()
    {
        turretState = TURRET_STATE_AUTON_AUTO_TARGET;
    }

    public void disableAutoAim()
    {
        turretState = TURRET_STATE_MANUAL;
        turretMC.set(0.0);
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  rotateTurret()
    *   - Manual control in Teleop
    *
    *---------------------------------------------------------------------------------------------*/
    public void autoAimTurret()
    {
        rotateTurret(Robot.vision.getXErrorOffset() * 0.06);
    }



    /*----------------------------------------------------------------------------------------------
    *
    *  rotateTurret()
    *   - Manual control in Teleop
    *
    *---------------------------------------------------------------------------------------------*/
    public void rotateTurret(double mtrPwr)
    {   

        turretMtrPwr = mtrPwr;

        if((getTurretPositionDeg() >= TURRET_POSITIVE_MAX_RANGE) && (turretMtrPwr > 0.0))
        {
            turretMtrPwr = 0.0;
            Robot.forwardSwitchHit = true;
        }
        else
        {
            Robot.forwardSwitchHit = false;
        }

        if((getTurretPositionDeg() <= TURRET_NEGATIVE_MAX_RANGE) && (turretMtrPwr < 0.0))
        {
            turretMtrPwr = 0.0;
            Robot.reverseSwitchHit = true;
        }
        else
        {
            Robot.reverseSwitchHit = false;
        }
        turretMC.set(turretMtrPwr);
        
    }



    /*----------------------------------------------------------------------------------------------
    *
    *  stopTurret()
    *
    *---------------------------------------------------------------------------------------------*/
    public void stopTurret()
    {
        turretMtrPwr = ZERO_VELOCITY;

        turretMC.set(turretMtrPwr);

        turretState = TURRET_STATE_MANUAL;
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  getTurretPositionDeg()
    *
    *---------------------------------------------------------------------------------------------*/
    public double getTurretPositionDeg()
    {
        turretPositionDeg = turretEncoder.getPosition();    //*(360/TURRET_GEAR_REDUCTION);
        return turretPositionDeg;
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  turretStartup()  TBD - USED ANYWHERE???
    *
    *---------------------------------------------------------------------------------------------*/
    public void turretStartup(double target)
    {
        resetTurretEncoder();

        targetPos = target;

        turretState = TURRET_STATE_MOVE_TO_POSITION;
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  setTurretCoastMode()
    *
    *---------------------------------------------------------------------------------------------*/    
    public void setTurretCoastMode()
    {
        turretMC.setIdleMode(IdleMode.kCoast);
    }


    /*----------------------------------------------------------------------------------------------
    *
    *  setTurretBrakeMode()
    *
    *---------------------------------------------------------------------------------------------*/    
    public void setTurretBrakeMode()
    {
        turretMC.setIdleMode(IdleMode.kBrake);
    }

    public void resetTurretEncoder()
    {
        turretEncoder.setPosition(HOME_POSITION);
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Smart Dashboard Data
    *
    *---------------------------------------------------------------------------------------------*/
    public void smartDashboardTurret_DEBUG()
    {
        double turretPos = turretEncoder.getPosition();
        SmartDashboard.putNumber("Turret Motor Revolutions", turretPos);
        SmartDashboard.putNumber("Turret Motor Position", turretPos * 360);
        SmartDashboard.putNumber("Turret Rotation", getTurretPositionDeg());
        
        SmartDashboard.putNumber("Turret State", moveToPositionTimeoutCntr);

        SmartDashboard.putNumber("TurretVel", turretEncoder.getVelocity());

    }

    public void smartDashboardTurret()
    {
        SmartDashboard.putBoolean("Home LS", limitSwitchHome.get());
        SmartDashboard.putNumber("Turret Angle", getTurretPositionDeg());
    }

}
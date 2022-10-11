package frc.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.CatzLog;
import frc.robot.*;


public class CatzAutonomous
{
    final public static double NAVX_RESET_WAIT_TIME = 0.2;

    public boolean runningRadialTurn = false;
    public double turnRateRadians;

    /*******************************************************************************
	*  Autonomous - Drive Straight
	*******************************************************************************/
    public double currentEncCountRt;
    public double currentEncCountLt;
    public double rightInitialEncoderCnt;

    public double distanceAbs;
    public double distanceMoved;

    public Timer driveStraightTimer;

    private final double DRIVE_STRAIGHT_LOOP_PERIOD  = 0.02;
    
    private final double DRIVE_STRAIGHT_ACCEL_PERIOD = 0.2;
    private final int    DRIVE_STRAIGHT_ACCEL_STEPS  = (int)((DRIVE_STRAIGHT_ACCEL_PERIOD / DRIVE_STRAIGHT_LOOP_PERIOD) + 1);
    

    public final double STOP_THRESHOLD_DIST =  1.0;
    public final double DECEL_THRESHOLD_DIST = 60.0;
    public final double TIME_TO_SLOW_IN_SEC = 0.5; 


    public final double FPS_TO_INCHES_PER_100MS         = (( 1.0 / 10.0) * (12.0 /  1.0));    //10 100 msec samples per sec & 12 inches per foot

    private final double DS_MIN_VELOCITY_LIMIT_FPS      = 2.0;
    private  double dsMinVelLimitCntsPer100ms = 0.0;

    public double targetVelocityCntsPer100ms   = 0.0;
    public double maxVelocityFPS               = 0.0;
    public double currentVelocityFPS;


    /***************************************************************************
	 * PID Turn Constants
	 ***************************************************************************/
    static public double PID_TURN_THRESHOLD   = 5.0;

	/***************************************************************************
	 * PID_TURN_DELTAERROR_THRESHOLD_HI - Delta Error Values larger than this are
	 * considered invalid and will be ignored 
     * PID_TURN_DELTAERROR_THRESHOLD_LO - When drivetrain power drops below the
     * PID_TURN_MIN_xxx_POWER, we will check to see if deltaError is below this 
     * threshold before setting power at PID_TURN_MIN_xxx_POWER.
	 ***************************************************************************/
	final static public double PID_TURN_DELTAERROR_THRESHOLD_HI = 4.0;
	final public static double PID_TURN_DELTAERROR_THRESHOLD_LO = 0.11;

	final static public double PID_TURN_FILTER_CONSTANT    = 0.7;
	      static public double PID_TURN_POWER_SCALE_FACTOR = 1.0;

	      static public double PID_TURN_KP = 0.08;
	      static public double PID_TURN_KI = 0.0;
	      static public double PID_TURN_KD = 0.012;

	final static public double PID_TURN_INTEGRAL_MAX =  1.0;
	final static public double PID_TURN_INTEGRAL_MIN = -1.0;

    // 0.4 is min power to move robot when it is stopped
	final public static double PID_TURN_MIN_POS_POWER = 0.6; 
    final public static double PID_TURN_MIN_NEG_POWER = -PID_TURN_MIN_POS_POWER;
    
    final public double TURN_MIN_VELOCITY_LIMIT_FPS       = 5.0;
    final public double TURN_MIN_VELOCITY_LIMIT_CONVERTED = convertVelocityToCntsPer100ms(TURN_MIN_VELOCITY_LIMIT_FPS);

    private static double PID_TURN_VELOCITY_FPS = 8.0;

	/***************************************************************************
	 * PID Turn Variables
	***************************************************************************/
	static Timer functionTimer;
    static Timer pdTimer;
    
    final public static double DRIVE_MAX_POS_POWER  =  1.0;
	final public static double DRIVE_MAX_NEG_POWER  = -1.0;

    private final double TURN_IN_PLACE_SCALE_FACTOR = 0.4474;


    static double pidTurnkP = PID_TURN_KP;
    static double pidTurnkI = PID_TURN_KI;
    static double pidTurnkD = PID_TURN_KD;

	static double currentError; 
	static double deltaError;
	static double derivative;
	static double deltaT;

	static double power;

	static double previousError;
	static double totalError;

	static double currentAngle;
	static double currentAngleAbs;
	static double targetAngle;
	static double targetAngleAbs;

	static double loopDelay = 0.005;

	static double previousDerivative = 0.0;

	static boolean tuningMode = false;
	static boolean debugMode  = false;

    private static double pidTurnDecelRate   = 0.5;
    private static double pidTurnDecelAngle  = 20.0;


    private final boolean INTAKE_ROLLER_ON  = true;
    private final boolean NO_INTAKE_ROLLING = false;

    public CatzAutonomous()
    {
        driveStraightTimer = new Timer();
    }

    /************************************************************************
    * 
    * Convert velocity from ft/sec to counts/100msec
    *
    ************************************************************************/
    public double convertVelocityToCntsPer100ms(double ftPerSec) 
    {
        double cntsPer100ms = ftPerSec * FPS_TO_INCHES_PER_100MS / Robot.driveTrain.currentEncCountsToInches; 

        return cntsPer100ms;
    }

    /************************************************************************
     * Distance: distance in inches that the robot has to go
     * targetVelocityFPS: speed in ft/sec
     * timeout: timeout time in sec
     * runIntake: if to also run intake
     ************************************************************************/
    public boolean driveStraight(double distance, double targetVelocityFPS, double timeout, Boolean runIntake)
    {
        CatzLog data;
        boolean autonLogData = true;

        boolean pathCompleted         = false;
        boolean driveStraightLoopDone = false;

        double currentTime        = 0.0;

        double currentVelocityRt  = 0.0;
        double currentVelocityLt  = 0.0;

        double distanceRemaining  = 0.0;
        double deltaCounts;
        double decelStartDistance = 0.0; 
        
        double accelVelStepSizeCntsPer100ms = 0.0;
        double velocityCntsPer100ms         = 0.0;

        boolean notSlowing            = true;
        double slowingSlope           = 0.0;
        double initialSlowingVelocity = 0.0;
        double initialSlowingTime     = 0.0;

        double intakeRollerOnStartDistance = -999.0;
        
        Robot.navx.reset(); //For determining drift error only
        Timer.delay(NAVX_RESET_WAIT_TIME);

        rightInitialEncoderCnt = Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);
        distanceAbs            = Math.abs(distance);

        if (runIntake)
        {
            intakeRollerOnStartDistance = distanceAbs * 0.8; //Distance when to turn rollers on
        }

        /*******************************************************************************
	    *  Determines minimum deceleration distance
        *      DECEL_THRESHOLD_DIST or 80% of total distance
	    *******************************************************************************/
        if(targetVelocityFPS <= 5.0)
        {
            decelStartDistance = 20.0;
        }
        else if(targetVelocityFPS <= 7.0)
        {
            decelStartDistance = 20.0;
        }
        else
        {
            decelStartDistance = 75.0;
        }
        
        
        /*decelStartDistance = distanceAbs * 0.8;

        if (decelStartDistance > DECEL_THRESHOLD_DIST)
        {
            decelStartDistance = DECEL_THRESHOLD_DIST;
        }*/

        /*******************************************************************************
	    *  TBD
	    *******************************************************************************/
        if (distance < 0.0)
        {
            targetVelocityCntsPer100ms   = -convertVelocityToCntsPer100ms(targetVelocityFPS);
            accelVelStepSizeCntsPer100ms = -convertVelocityToCntsPer100ms(targetVelocityFPS / DRIVE_STRAIGHT_ACCEL_STEPS);
        }
        else
        {
            targetVelocityCntsPer100ms   = convertVelocityToCntsPer100ms(targetVelocityFPS);
            accelVelStepSizeCntsPer100ms = convertVelocityToCntsPer100ms(targetVelocityFPS / DRIVE_STRAIGHT_ACCEL_STEPS);
        }
        velocityCntsPer100ms = accelVelStepSizeCntsPer100ms;

        dsMinVelLimitCntsPer100ms = convertVelocityToCntsPer100ms(DS_MIN_VELOCITY_LIMIT_FPS);


        /*******************************************************************************
	    *  TBD
	    *******************************************************************************/
        data = new CatzLog(distanceAbs, targetVelocityFPS, 
                            decelStartDistance, DS_MIN_VELOCITY_LIMIT_FPS, DECEL_THRESHOLD_DIST, 
                            Robot.driveTrain.LT_PID_P, Robot.driveTrain.LT_PID_F,
                            Robot.driveTrain.RT_PID_P, Robot.driveTrain.RT_PID_F,
                            Robot.driveTrain.currentEncCountsToInches,
                            Robot.driveTrain.currentDrvTrainGear,
                            Robot.pdp.getVoltage(),
                            -999.0, -999.0, -999.0, -999.0);

        Robot.dataCollection.logData.add(data);


        driveStraightTimer.reset();
        driveStraightTimer.start();
        
        Robot.driveTrain.setDrvStraightTargetVelocity(velocityCntsPer100ms);

        while(driveStraightLoopDone == false)
        {
            currentTime   = driveStraightTimer.get();

            /***************************************************************************
	        *  Calculates the distance left to travel
	        ***************************************************************************/
            currentEncCountRt = (double)Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);
            //currentEncCountLt = (double)Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(0);

            deltaCounts   = currentEncCountRt - rightInitialEncoderCnt;
            distanceMoved = Math.abs( (deltaCounts * Robot.driveTrain.currentEncCountsToInches) );
            
            //distance in inches (error)
            distanceRemaining   = distanceAbs - distanceMoved; 

            /***************************************************************************
	        *  Gets drift angle and velocity data to be logged (debugging)
	        ***************************************************************************/
            currentVelocityRt  = Robot.driveTrain.getIntegratedEncVelocity(Robot.driveTrain.DRVTRAIN_RT); 
            currentVelocityLt  = Robot.driveTrain.getIntegratedEncVelocity(Robot.driveTrain.DRVTRAIN_LT);

            currentVelocityFPS = currentVelocityLt * ((10.0 /  1.0) * (1.0 / 12.0)) * Robot.driveTrain.currentEncCountsToInches;

            if(currentVelocityFPS > maxVelocityFPS)
            {
                maxVelocityFPS = currentVelocityFPS;
            }

            currentAngle  = Robot.navx.getAngle(); //For determining drift error only

            /***************************************************************************
	        *  TBD
	        ***************************************************************************/
            if (distanceRemaining < STOP_THRESHOLD_DIST)
            {
                targetVelocityFPS = 0.0;
                targetVelocityCntsPer100ms = 0.0;
                Robot.driveTrain.setDrvStraightTargetVelocity(targetVelocityCntsPer100ms);
                pathCompleted = true;
                driveStraightLoopDone = true;
            }
            else 
            {
                /**
                  when the distance remaining is less than xx in, change the target velocity by the decrease rate
                 */
                if (distanceRemaining < decelStartDistance)
                {   
                    if (notSlowing)
                    {
                        initialSlowingVelocity = velocityCntsPer100ms;
                        initialSlowingTime     = currentTime;
                        slowingSlope           = ((initialSlowingVelocity - dsMinVelLimitCntsPer100ms) / TIME_TO_SLOW_IN_SEC);
                        notSlowing             = false;   
                    }

                    /*******************************************************************
                    *  TBD
                    *******************************************************************/
                    if (velocityCntsPer100ms > dsMinVelLimitCntsPer100ms)
                    {
                        velocityCntsPer100ms = initialSlowingVelocity - (slowingSlope * (currentTime - initialSlowingTime));
                        Robot.driveTrain.setDrvStraightTargetVelocity(velocityCntsPer100ms);
                    }
                }
                else
                {
                    if(distance < 0.0)
                    {
                        if(velocityCntsPer100ms > targetVelocityCntsPer100ms)
                        {
                            velocityCntsPer100ms = velocityCntsPer100ms + accelVelStepSizeCntsPer100ms;

                            if(velocityCntsPer100ms < targetVelocityCntsPer100ms)
                            {
                                velocityCntsPer100ms = targetVelocityCntsPer100ms;
                            }
                            Robot.driveTrain.setDrvStraightTargetVelocity(velocityCntsPer100ms);
                        }
                    }
                    else
                    {
                        if(velocityCntsPer100ms < targetVelocityCntsPer100ms)
                        {
                            velocityCntsPer100ms = velocityCntsPer100ms + accelVelStepSizeCntsPer100ms;

                            if(velocityCntsPer100ms > targetVelocityCntsPer100ms)
                            {
                                velocityCntsPer100ms = targetVelocityCntsPer100ms;
                            }
                            Robot.driveTrain.setDrvStraightTargetVelocity(velocityCntsPer100ms);
                        }
                        
                    }
                    
                    
                }

                /***********************************************************************
                *  Runs intake when robot reaches intakeStartDistance
	            ***********************************************************************/
                if(distanceRemaining < intakeRollerOnStartDistance)
                {
                    Robot.intake.intakeRollerInAuton();
                }

                /***********************************************************************
                *  TBD
	            ***********************************************************************/
                if(currentTime > timeout)
                {
                    targetVelocityFPS          = 0.0;
                    targetVelocityCntsPer100ms = 0.0;

                    Robot.driveTrain.setDrvStraightTargetVelocity(targetVelocityCntsPer100ms);
                    pathCompleted = false;
                    driveStraightLoopDone      = true;
                } 
            } // end:if (distanceRemaining < STOP_THRESHOLD_DIST)

            if (autonLogData == true)
            {
                data = new CatzLog(currentTime, velocityCntsPer100ms, currentVelocityRt,
                                    Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getClosedLoopError(0),
                                    Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getMotorOutputPercent(), 
                                    currentEncCountRt,
                                    currentVelocityLt,
                                    Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getClosedLoopError(0), 
                                    Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getMotorOutputPercent(),
                                    currentEncCountLt, 
                                    distanceRemaining,
                                    Robot.navx.getAngle(),
                                    -999.0, -999.0,-999.0,-999.0);     
                                    
                Robot.dataCollection.logData.add(data);
            }

            Timer.delay(DRIVE_STRAIGHT_LOOP_PERIOD); 
        } // end: while(done == false)

        if (runIntake)
        {
            //Timer.delay(0.25); //To ensure that cargo is collected before intake roller turns off
        }

        return pathCompleted;

    }   // end of driveStraight
    

    /***************************************************************************
    *
    *   monitorEncoderPosition which is linear slowing by RIC on 1/9/2022 based 
    *   off monitorEncoderPosition
    *   merged monitorEncoderPosition into DriveStraight on 1/15/2022
    *
    ***************************************************************************/     
    public void TurnInPlace(double degreesToTurn, double timeoutSeconds) 
    {
        CatzLog data;
        double  currentVelocityRt  = 0.0;
        double  currentVelocityLt  = 0.0;
        boolean turnInPlaceDone = false;
        
        double  currentTime       = 0.0;
        double  angleRemainingAbs = 999.0;

        Robot.navx.reset();
        Timer.delay(NAVX_RESET_WAIT_TIME);
    
        //switch lowgear to highgear when driving straight (put outside of method)
        Robot.driveTrain.shiftToHighGear();  

        functionTimer = new Timer();
        functionTimer.reset();
        functionTimer.start(); 
    
        setTurnValues(degreesToTurn);
    
        previousError = 0.0;
        totalError    = 0.0;
    
        currentAngle  = Robot.navx.getAngle();
        targetAngle   = degreesToTurn + currentAngle;
        currentError  = targetAngle   - currentAngle;
    
        targetAngleAbs = Math.abs(targetAngle);     
        
        data = new CatzLog( currentTime, currentAngle, currentError,
                            Robot.driveTrain.LT_PID_P, 
                            Robot.driveTrain.LT_PID_F,
                            Robot.driveTrain.RT_PID_P, 
                            Robot.driveTrain.RT_PID_F,
                            Robot.driveTrain.currentEncCountsToInches,
                            Robot.driveTrain.currentDrvTrainGear,
                            Robot.pdp.getVoltage(),
                            -999.0, -999.0, -999.0, -999.0, -999.0, -999.0);
        Robot.dataCollection.logData.add(data);
        
        /*----------------------------------------------------------------------
        *  Set Initial Turn Velocity
        ----------------------------------------------------------------------*/
        targetVelocityCntsPer100ms = convertVelocityToCntsPer100ms(PID_TURN_VELOCITY_FPS);

        if (targetAngle < 0.0) 
        {
            targetVelocityCntsPer100ms = -targetVelocityCntsPer100ms;
        }

        Robot.driveTrain.setTurnInPlaceTargetVelocity(targetVelocityCntsPer100ms); 

        /*----------------------------------------------------------------------
        *  Monitor Angle To Determine When to Decelerate and Stop
        ----------------------------------------------------------------------*/
        while (turnInPlaceDone == false) 
        {
            currentTime  = functionTimer.get();
            currentAngle = Robot.navx.getAngle();
    
            // calculates proportional term
            currentError      = targetAngle - currentAngle;
            angleRemainingAbs = Math.abs(currentError);

            //clean up PID TURN THRESHOLD without mulitply
            if (angleRemainingAbs < PID_TURN_THRESHOLD) 
            { 
                turnInPlaceDone = true;
            }

            else
            {
                if (currentTime > timeoutSeconds) 
                {
                    turnInPlaceDone = true;
                }
                else
                {
                    if (angleRemainingAbs < pidTurnDecelAngle) 
                    {
                        if(targetVelocityCntsPer100ms > TURN_MIN_VELOCITY_LIMIT_CONVERTED )
                        {
                            targetVelocityCntsPer100ms = targetVelocityCntsPer100ms * pidTurnDecelRate;
                            Robot.driveTrain.setTurnInPlaceTargetVelocity(targetVelocityCntsPer100ms); 
                        }   
                    }
                }
            }

            data = new CatzLog(currentTime, targetAngle, currentAngle, currentError,
                                Robot.driveTrain.drvTrainMtrCtrlRTFrnt.getClosedLoopError(0), 
                                currentEncCountRt, 
                                currentVelocityLt, 
                                currentVelocityRt, 
                                Robot.driveTrain.drvTrainMtrCtrlLTFrnt.getClosedLoopError(0), 
                                currentEncCountLt, 
                                Robot.navx.getAngle(), 
                                targetVelocityCntsPer100ms,
                                -999.0, -999.0,-999.0, -999.0);
            
            Robot.dataCollection.logData.add(data);

        }   //End of while (done == false)

        /*----------------------------------------------------------------------
        *  We either hit target angle or have timed out - stop
        ----------------------------------------------------------------------*/
        targetVelocityCntsPer100ms = 0.0;
        Robot.driveTrain.setDrvStraightTargetVelocity(targetVelocityCntsPer100ms);
     
    }   //end of TurnInPlace()

   
    public static void setTurnValues(double degreesToTurn) 
    {
        double degreesToTurnAbs;
    
        degreesToTurnAbs = Math.abs(degreesToTurn);
    
        if (degreesToTurnAbs <= 60.0) 
        {
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 10.0;
        }
        else if (degreesToTurnAbs <= 75.0) 
        {
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 12.0;
        }
        else if (degreesToTurnAbs <= 105.0) 
        {
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 14.0;
        }
        else if (degreesToTurnAbs <= 140.0) 
        {
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 16.0;
        }
        
        // degreesToTurnAbs > 140.0
        else 
        { 
            pidTurnDecelRate   = 0.98;
            pidTurnDecelAngle  = 18.0;
        }
    }   //End of setTurnValues()


    /***************************************************************************
    *
    * Autonomous Mechanisms Controls
    * 
    ***************************************************************************/

    public void autonShootTarmac()
    {
        Robot.shooter.setTargetRPM(Robot.shooter.SHOOTER_TARGET_RPM_TARMAC);
        while(Robot.shooter.isShooterStateOff() == false)
        {
            Timer.delay(0.15);
        }
    }

    public void autonIntakeOn()
    {
        Robot.intake.intakeRollerIn(); 
        Robot.shydexer.shyDexerOn();
    }

    public void autonIntakeOff()
    {
        Robot.intake.intakeRollerOff();
        Robot.shydexer.shyDexerOff();
    }

    public void autonShootFender()
    {
        Robot.shooter.setTargetRPM(Robot.shooter.SHOOTER_TARGET_RPM_FENDER); 

        while(Robot.shooter.isShooterStateOff() == false)
        {
            Timer.delay(0.02);
        }
    }

    public void autonShootFormula()
    {
        Robot.shooter.setTargetRPM(Robot.shooter.SHOOTER_TARGET_RPM_VARIABLE_SHOOTER); 

        while(Robot.shooter.isShooterStateOff() == false)
        {
            Timer.delay(0.02);
        }
    }

    public void autonDeployIntake()
    {
        Robot.intake.deployIntake();
    }

    
   

    /***************************************************************************
    *
    * Autonomous Paths - BLUE
    * 
    ***************************************************************************/
    public void blueLeftFender() 
    {
        autonDeployIntake();

        autonIntakeOn();

        driveStraight(45.0, 3.5, 1.2, NO_INTAKE_ROLLING);       //Intakes second cargo

        Robot.intake.stowIntake();

        autonIntakeOff();

        //driveStraight(-77.0, 3.5, 2.5, NO_INTAKE_ROLLING);     //Drives to Fender Wall & makes sure it
                                                                // bumps into the wall & straightens itself out
        driveStraight(-40.0, 3.5, 2.5, NO_INTAKE_ROLLING);
        Timer.delay(1.5);                                       //Waiting to space out shots compared to other robots

        //autonShootFender();
        autonShootFormula();

        Timer.delay(2.0);

        driveStraight(65.0, 4.0, 2.5, NO_INTAKE_ROLLING);
        
        if(Robot.ydexer.getTopSensorPressed() || Robot.ydexer.getBtmSensorPressed())
        {
            autonShootFormula();
        }
    }

    public void blueMiddleFourCargo()                           //UNTESTED PATH
    {
        blueMiddleTwoCargo();

        autonDeployIntake();
    

        driveStraight(210.0, 9.0, 3.0, NO_INTAKE_ROLLING); 

        Timer.delay(0.75);                                       //Waiting to eat human player cargo



        Robot.turret.enableAutoAim();


        driveStraight(-174.0, 9.0, 3.0, NO_INTAKE_ROLLING); 

        Timer.delay(0.75);

       // autonShootTarmac();  
        autonShootFormula();                                   //Shoots 2 cargo

        Robot.turret.disableAutoAim();
        autonIntakeOff();
    }

    public void blueMiddleTwoCargo()
    {
        autonDeployIntake();

        Robot.intake.intakeRollerIn(); 
        Robot.shydexer.shyDexerOn();

        driveStraight(64.0, 4.0, 1.5, NO_INTAKE_ROLLING);  
        
        Robot.turret.enableAutoAim();

        driveStraight(-28.0, 3.5, 1.0, NO_INTAKE_ROLLING);     //Drives to Fender Wall & makes sure it
                                                                // bumps into the wall & straightens itself out
        

        //autonShootTarmac();
        autonShootFormula();

        Robot.turret.disableAutoAim();

        Timer.delay(2.5); //wait for shooting to complete, before executing remainder of fourball auto
    }

    public void blueRightFender() 
    {
        blueLeftFender(); //It's the same distance so we can use the same code
    }


    /***************************************************************************
    *
    * Autonomous Paths - RED
    * 
    ***************************************************************************/

    public void redLeftFender()
    {
        blueLeftFender(); //The field is symmetrical so it is the same as blue
    }

    public void redMiddleFourCargo()
    {
        blueMiddleFourCargo(); //The field is symmetrical so it is the same as blue
    }

    public void redMiddleTwoCargo()
    {
        blueMiddleTwoCargo(); //The field is symmetrical so it is the same as blue
    }

    public void redRightFender()
    {
        blueRightFender(); //The field is symmetrical so it is the same as blue
    }



    /************************************************************************
    * 
    *  smartDashBoardAutonomous()
    *  smartDashBoardAutonomous_DEBUG()
    *
    ************************************************************************/
    public void smartDashBoardAutonomous_DEBUG()
    {
        SmartDashboard.putNumber("DS-distAbs",     distanceAbs);
        SmartDashboard.putNumber("DS-distMoved",   distanceMoved);

        SmartDashboard.putNumber("TIP-CurrAngle",  currentAngle);
        SmartDashboard.putNumber("TIP-currError",  currentError);
        SmartDashboard.putNumber("xx-CurrVelFps",  currentVelocityFPS);
  
        SmartDashboard.putNumber("DS-TrgtVelFps", targetVelocityCntsPer100ms * ((10.0 /  1.0) * (1.0 / 12.0)) * Robot.driveTrain.currentEncCountsToInches);
        SmartDashboard.putNumber("encoderCounts", Robot.driveTrain.currentEncCountsToInches);
        SmartDashboard.putNumber("MaxVelocity",   maxVelocityFPS);
    }

}
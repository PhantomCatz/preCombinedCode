package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.DataLogger.CatzLog;
import frc.robot.Robot;


public class CatzShooter
{
    public boolean shooterDataCollectionOn = false;
    public boolean shooterOn = false;

    //establishing components
    public WPI_TalonFX shtrMCTop;
    public WPI_TalonFX shtrMCBtm;

    public Servo hoodServo1;
    public Servo hoodServo2;

    //Ports currently based on AntiSigma
    private final int SHTR_MC_ID_TOP     = 10;  
    private final int SHTR_MC_ID_BTM     = 11;  

    private final int HOOD_SERVO_PWM_PORT1 = 1;
    private final int HOOD_SERVO_PWM_PORT2 = 2;
   

    //Conversions
    final double COUNTS_PER_REVOLUTION      = 2048.0;
    final double SEC_PER_MIN                = 60.0;
    final double ENCODER_SAMPLE_RATE_MSEC   = 100.0;
    final double ENCODER_SAMPLE_PERIOD_MSEC = (1.0 / ENCODER_SAMPLE_RATE_MSEC);
    final double MSEC_TO_SEC                = 1000.0;
    final double HUNDRED_MSEC_PER_SEC       = 10.0;

    //converts velocity to RPM
    final double CONV_QUAD_VELOCITY_TO_RPM       = ( ((ENCODER_SAMPLE_PERIOD_MSEC * MSEC_TO_SEC * SEC_PER_MIN) / COUNTS_PER_REVOLUTION) ); 
    final double CONV_RPM_TO_COUNTS_PER_100_MSEC = (COUNTS_PER_REVOLUTION) * (1.0/SEC_PER_MIN) * (1.0/HUNDRED_MSEC_PER_SEC);

    private static final int SHOOTER_STATE_OFF                     = 0;
    private static final int SHOOTER_STATE_WAIT_FOR_STEADY_STATE   = 1;
    private static final int SHOOTER_STATE_READY                   = 2;
    private static final int SHOOTER_STATE_START_SHOOTING          = 3;
    private static final int SHOOTER_STATE_WAIT_FOR_SHOOT_DONE     = 4;
    private static final int SHOOTER_STATE_DECELERATION            = 5;

    private static       int shooterState = SHOOTER_STATE_OFF;
    
    public double shtrTopTargetRPM = 0.0; 
    public double shtrBtmTargetRPM = 0.0; 

    public final double SHOOTER_OFF_RPM = 0.0;

    public final double SHOOTER_TARGET_RPM_PREREV = 2000.0;
    public final double SHOOTER_TARGET_RPM_FENDER  = 2225.5;//2175.0;
    public final double SHOOTER_TARGET_RPM_TARMAC  = 2500.0;
    public final double SHOOTER_TARGET_RPM_VARIABLE_SHOOTER = 3000.0;
    public final double SHOOTER_TARGET_RPM_DEFENSE_MODE = 6000.0;
    public final double SHOOTER_TARGET_RPM_LAUNCHPAD = 3295.0;

    private final double SERVO_POSITION = 0.3;
    private final double SERVO_ONE_OFFSET = 0.06;
    private final double SERVO_POSITION_FENDER = 0.0;
    private final double SERVO_POSITION_TARMAC = 0.245;
    private final double SERVO_POSITION_DEFENSE = 0.0;
    private final double SERVO_POSITION_LAUNCHPAD = 0.43;


    /*-----------------------------------------------------------------------------------------
    *  Velocity offset to account for drop in RPM when applying kP. Values should be set such 
    *  that actual velocity is within 5 RPM of desired velocity. 
    *  This value needs to be periodically reviewed as mechanical updates, motor wear, etc can 
    *  cause this value to change. Recommend reviewing this value at the beginning of each day. 
    *----------------------------------------------------------------------------------------*/
    public final double SHOOTER_RPM_PID_OFFSET_TOP      = -35.0;
    public final double SHOOTER_RPM_PID_OFFSET_BTM      = -45.0; 
    
    public final double SHOOTER_RPM_BACKSPIN_OFFSET = 300.0; 

    // adds/subtracts from the targetRPM to create a zone where the code should maintain the speed at
    public final double SHOOTER_MAX_RPM_OFFSET = 50.0;  
    public final double SHOOTER_MIN_RPM_OFFSET = 50.0;

    private double Top_MinRPM = 0.0;
    private double Top_MaxRPM = 0.0; 
    private double Btm_MinRPM = 0.0;
    private double Btm_MaxRPM = 0.0;


    //Counts to SHOOTER_RPM_STEADY_THRESHOLD before moving to the next state
    public int shooterRPMSteadyCounter            = 0; 
    public final int SHOOTER_RPM_STEADY_THRESHOLD = 5;

    //setting up thread/timeouts

    final int    SHOOTER_THREAD_PERIOD_MS        = 20;
    final double SHOOTER_THREAD_PERIOD           = (double) (SHOOTER_THREAD_PERIOD_MS / MSEC_TO_SEC);
    final double INDEXER_SHOOT_TIME_SEC          = 4.00;
    final double SHOOTER_AVG_VEL_SAMPLE_TIME_SEC = 0.100;

    private Thread  shooterThread;
    private Timer   shootTimer;

    //auton
    public final double SHOOT_COUNTER_LIMIT_SECONDS      = 1.5;//2.0;
    public final double SECONDS_TO_COUNTS                = 1.0 / SHOOTER_THREAD_PERIOD;
    public final double SHOOT_COUNTER_CONV_SEC_TO_COUNTS = SHOOT_COUNTER_LIMIT_SECONDS * SECONDS_TO_COUNTS;

    public final double READY_COUNTER_LIMIT_SECONDS      = 1.0;
    public final double READY_COUNTER_CONV_SEC_TO_COUNTS = READY_COUNTER_LIMIT_SECONDS * SECONDS_TO_COUNTS;
    public int waitForShooterReadyTimeoutCounter         = 0;  

    public boolean inAutonomous     = false;
    public boolean shooterAutonInit = false;

    //for recording data
    private double MCTopMtrVelocityRPM = -999.0;
    private double MCBtmMtrVelocityRPM = -999.0;

    //setup for PID closed loop
    private final int SHOOTER_PID_IDX          = 0;
    private double PID_P_TOP                   = 0.1;
    private double PID_P_BTM                   = 0.1;
    private double PID_I                       = 0.0;
    private double PID_D                       = 0.0;
    private double PID_F_TOP                   = (1023.0/20666.0); 
    private double PID_F_BTM                   = (1023.0/20666.0);

    private int PID_TIMEOUT_MS      = 10;
    private int shooterTraceID      = -1;

    public int shootCounter          = 0;

    public final double VELOCITY_DECEL_STEP_SIZE_RPM = 100.0;

    //variables for servo 
    private final double SERVO_PW_MS_MAX_POSITION = 2.0;
    private final double SERVO_PW_MS_MAX_DEADBAND_POSITION = 1.8;
    private final double SERVO_PW_MS_CENTER_POSITION = 1.5;
    private final double SERVO_PW_MS_MIN_DEADBAND_POSITION = 1.2;
    private final double SERVO_PW_MS_MIN_POSITION = 1.0;

    private double servoPosition = 0.0;

    //for bounds if needed
    private double MIN_SERVO_POSITION = -0.1;
    private double MAX_SERVO_POSITION = 0.9;
    private double INITIAL_SERVO_POSITION = 0.0;

    


    //Variables for Testing/Data Collection
    private CatzLog data;
    public boolean configDataPrinted  = false;

    boolean decelDone      = false;
    boolean shotSolution = false;

    public double targetRPM = 0.0;
    public double distance = 0.0;

    public final double OUT_OF_RANGE_SCALE_FACTOR = 0.85;

    public CatzShooter()
    {
        shootTimer = new Timer();

        //initialize motor controllers
        shtrMCTop = new WPI_TalonFX(SHTR_MC_ID_TOP);
        shtrMCBtm = new WPI_TalonFX(SHTR_MC_ID_BTM);

        hoodServo1 = new Servo(HOOD_SERVO_PWM_PORT1);
        hoodServo2 = new Servo(HOOD_SERVO_PWM_PORT2);
        

        hoodServo1.setBounds(SERVO_PW_MS_MAX_POSITION, SERVO_PW_MS_MAX_DEADBAND_POSITION, SERVO_PW_MS_CENTER_POSITION, SERVO_PW_MS_MIN_DEADBAND_POSITION, SERVO_PW_MS_MIN_POSITION);
        hoodServo2.setBounds(SERVO_PW_MS_MAX_POSITION, SERVO_PW_MS_MAX_DEADBAND_POSITION, SERVO_PW_MS_CENTER_POSITION, SERVO_PW_MS_MIN_DEADBAND_POSITION, SERVO_PW_MS_MIN_POSITION);

        //reset Motor Controllers to Factory Default
        shtrMCTop.configFactoryDefault();
        shtrMCBtm.configFactoryDefault();

        //set Idle modes of Motor Controllers
        shtrMCTop.setNeutralMode(NeutralMode.Coast);
        shtrMCBtm.setNeutralMode(NeutralMode.Coast);

        shtrMCTop.setStatusFramePeriod(1,15);
        shtrMCBtm.setStatusFramePeriod(1,15);

        shtrMCTop.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, SHOOTER_PID_IDX, PID_TIMEOUT_MS); 
        shtrMCBtm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, SHOOTER_PID_IDX, PID_TIMEOUT_MS);

        //set states
        shooterState = SHOOTER_STATE_OFF;

        //setServoPosition(INITIAL_SERVO_POSITION);

        //start thread
        setShooterVelocity();

        
        
    }

    //will make shooter run and etc
    public void setShooterVelocity() 
    {

        shooterThread = new Thread(() -> //start of thread
        {

            shootTimer.start();
            double  shootTime      = 0.0;
            boolean rumbleSet      = false;
           

            //hoodSetBot();

            disableShooterPID();

            while(true)
            {
                shootTime = shootTimer.get();

                MCTopMtrVelocityRPM = (Math.abs((double) shtrMCTop.getSensorCollection().getIntegratedSensorVelocity() * CONV_QUAD_VELOCITY_TO_RPM));
                MCBtmMtrVelocityRPM = (Math.abs((double) shtrMCBtm.getSensorCollection().getIntegratedSensorVelocity() * CONV_QUAD_VELOCITY_TO_RPM));

                switch (shooterState)
                {
                    //targetRPM is set to 0, if shooter is already running it will slow it down till it stops.
                    case SHOOTER_STATE_OFF: 

                        shooterTraceID = 10;

                        /*-------------------------------------------------------------------------
                        *  
                        *------------------------------------------------------------------------*/
                        if(shtrTopTargetRPM > 0.0)
                        {
                            shooterState            = SHOOTER_STATE_WAIT_FOR_STEADY_STATE;
                            shooterRPMSteadyCounter = 0;
                            decelDone               = false;
                            rumbleSet               = false;
                            shooterAutonInit        = false;
                            shootCounter            = 0;
                            waitForShooterReadyTimeoutCounter = 0;

                            setTargetVelocity(shtrTopTargetRPM, shtrBtmTargetRPM);

                            enableShooterPID(); 

                            shooterTraceID = 1;
                        }

                        
                     break;

                    // sets shooter target velocites to 
                    case SHOOTER_STATE_WAIT_FOR_STEADY_STATE: 

                        shooterTraceID = 20;

                        if( (MCTopMtrVelocityRPM >= Top_MinRPM && MCTopMtrVelocityRPM <= Top_MaxRPM) &&
                            (MCBtmMtrVelocityRPM >= Btm_MinRPM && MCBtmMtrVelocityRPM <= Btm_MaxRPM) )
                        {
                            shooterTraceID = 22;
                            shooterRPMSteadyCounter++;

                            if(shooterRPMSteadyCounter >= SHOOTER_RPM_STEADY_THRESHOLD)
                            {
                                shooterTraceID = 23;
                                shooterState   = SHOOTER_STATE_READY;                                
                            }
                        }
                        else
                        {    
                            shooterTraceID          = 24;
                            shooterRPMSteadyCounter = 0;
                        }
                        waitForShooterReadyTimeoutCounter++;

                        if(waitForShooterReadyTimeoutCounter > READY_COUNTER_CONV_SEC_TO_COUNTS)
                        {
                            shooterTraceID = 25;
                            if(inAutonomous)
                            {
                                shooterTraceID = 26;
                                shooterState = SHOOTER_STATE_READY;
                            }
                        }


                    break;

                    // makes the controller vibrate so that aux driver knows to shoot, if in auton will count on timer
                    case SHOOTER_STATE_READY:
                        shooterTraceID = 30;

                        if(inAutonomous)
                        {
                            shooterTraceID = 34;
                            if(shooterAutonInit == false)
                            {
                                shooterTraceID = 35;
                                shoot();
                                shooterAutonInit = true;
                                shooterState     = SHOOTER_STATE_WAIT_FOR_SHOOT_DONE;

                            }
                        }

                        if(rumbleSet == false)
                        {
                            shooterTraceID = 31;
                            Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 1.0);
                            rumbleSet = true;
                        }

                    break;

                    case SHOOTER_STATE_START_SHOOTING:
                    break;

                    case SHOOTER_STATE_WAIT_FOR_SHOOT_DONE:
                        shooterTraceID = 40;

                        shootCounter++;    
                        if (shootCounter > SHOOT_COUNTER_CONV_SEC_TO_COUNTS)
                        {
                            shooterTraceID = 41;
                            shooterOff();
                        }
                        
                    break;

                    case SHOOTER_STATE_DECELERATION:
                        shooterTraceID = 50;
                        
                        /*-------------------------------------------------------------------------
                        *  Ramp shooter rollers down to zero
                        *------------------------------------------------------------------------*/
                        if(shtrTopTargetRPM > 0.0)
                        {   
                            shtrTopTargetRPM -= VELOCITY_DECEL_STEP_SIZE_RPM;
                            if(shtrTopTargetRPM < 0.0)
                            {
                                shtrTopTargetRPM = 0.0;
                            }
                            
                        }
                        else
                        {   
                            shtrTopTargetRPM = SHOOTER_OFF_RPM;
                            decelDone = true;
                        }

                        if(shtrBtmTargetRPM > 0.0)
                        {   
                            shtrBtmTargetRPM -= VELOCITY_DECEL_STEP_SIZE_RPM;
                            if(shtrBtmTargetRPM < 0.0)
                            {
                                shtrBtmTargetRPM = 0.0;
                            }  
                        }
                        else
                        {   
                            shtrBtmTargetRPM = SHOOTER_OFF_RPM;
                            decelDone = true;
                        }

                        //sets Motors to Target RPMs
                        setTargetVelocity(shtrTopTargetRPM, shtrBtmTargetRPM);

                        //sets states Ready to False
                        disableShooterPID();
                         
                        if(decelDone == true)
                        {
                            shooterState = SHOOTER_STATE_OFF;
                        }  
                        
                    break;

                    default: 
                        shooterState = SHOOTER_STATE_OFF;
                    break;
                }


                /*---------------------------------------------------------------------------------
                *  Data Collection
                *--------------------------------------------------------------------------------*/
                if(shooterDataCollectionOn == true)
                {
                    if(shooterTraceID > 0 && shooterState != SHOOTER_STATE_OFF)
                    {
                        if(configDataPrinted == false)
                        {
                            data = new CatzLog(PID_F_TOP, PID_P_TOP, PID_F_BTM, PID_P_BTM, Top_MaxRPM, Top_MinRPM, SHOOTER_RPM_PID_OFFSET_TOP,
                                                                                        Btm_MaxRPM, Btm_MinRPM, SHOOTER_RPM_PID_OFFSET_BTM,
                                                                                        SHOOTER_RPM_BACKSPIN_OFFSET,
                                                                                        -999.0, -999.0, -999.0, -999.0, -999.0);
                            configDataPrinted = true;
                        }
                        else
                        {
                            data = new CatzLog(shootTime, shooterTraceID, 
                                            shtrTopTargetRPM, MCTopMtrVelocityRPM, shtrMCTop.getClosedLoopError(SHOOTER_PID_IDX), 
                                                                                shtrMCTop.getMotorOutputPercent(),
                                                                                shtrMCTop.getMotorOutputVoltage(), 
                                                                                shtrMCTop.getSupplyCurrent(),
                                            shtrBtmTargetRPM, MCBtmMtrVelocityRPM, shtrMCBtm.getClosedLoopError(SHOOTER_PID_IDX), 
                                                                                shtrMCBtm.getMotorOutputPercent(),
                                                                                shtrMCBtm.getMotorOutputVoltage(), 
                                                                                shtrMCBtm.getSupplyCurrent(),
                                            shooterRPMSteadyCounter, -999.0);
                        }
                        

                        Robot.dataCollection.logData.add(data);
                    }  
                }
                
                
                Timer.delay(SHOOTER_THREAD_PERIOD);
            }
        });   //end of thread

        shooterThread.start();
    }


    public void shoot()
    {
        if(MCBtmMtrVelocityRPM > 1000)
        {
            Robot.ydexer.setShooterOn();
            shootCounter = 0;
            Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0.0);
            shooterState = SHOOTER_STATE_WAIT_FOR_SHOOT_DONE;
        }
    }


    public void shooterOff()
    { 
        Robot.ydexer.setShooterOff();
        Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0.0);
        decelDone = false;
        shooterState = SHOOTER_STATE_DECELERATION;
    }

    public boolean isShooterStateOff()
    {
        if(shooterState == SHOOTER_STATE_OFF)
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    //gets input in RPM, converts and sets motor controllers to reach RPMs
    public void setTargetVelocity(double setRPMTop, double setRPMBtm) 
    {
        double velocityTop = setRPMTop * CONV_RPM_TO_COUNTS_PER_100_MSEC;
        double velocityBtm = setRPMBtm * CONV_RPM_TO_COUNTS_PER_100_MSEC;

        shtrMCTop.set(TalonFXControlMode.Velocity, -velocityTop);
        shtrMCBtm.set(TalonFXControlMode.Velocity,  velocityBtm);
    }

    public void formulatedShots()
    {
        /*
            //targetRPM = (-0.0001*Math.pow(distance,3)) + (0.0641*Math.pow(distance,2)) - (4.208*distance) + 2295.6; //3675;
            //setServoPosition( (0.00000004*Math.pow(distance,3)) - (0.00003*Math.pow(distance,2)) + (0.0064*distance) - 0.2135 );
            targetRPM = (0.008337*Math.pow(distance,2)) + (3.210261 * distance) + 2067.687315;
            setServoPosition((-4.936475*Math.pow(10,-6)*Math.pow(distance,2)) + (0.003556*distance) - 0.125712); 
        distance = Robot.vision.getDistanceToTarget();
        if(true)//distance <= 70.0 && distance > 30.0)
        {
            //hoodSetBot();
            targetRPM = (0.3247*Math.pow(distance, 2) - (10.39*distance) + 2038.6); 
            shotSolution = true;
            
        }
        else if(distance > 100.0 && distance <= 400)
        {
            //hoodSetTop();
            
            targetRPM = (0.95) * (0.000005*Math.pow(distance, 2) + (6.2056*distance) + (1755.4)); 
            shotSolution = true;
        }

        if(targetRPM > 7000.0 || targetRPM < 500)
        {
            targetRPM = 0.0;
            System.out.println("Dynamic shot invalid RPM");
            shotSolution = false;
            shooterOff();
        }
        if(distance>190.05 || distance<5.5)
        {
            System.out.println("Dynamic shot invalid Distance");
            shotSolution = false;
        }
        
            
        if(targetRPM >= 3400)
        {
            Robot.ydexer.ydexerShootPower       = Robot.ydexer.YDEXER_FAR_RPM_SHOOT_MTR_PWR;
            Robot.ydexer.ydexerSecondShootPower = Robot.ydexer.YDEXER_FAR_RPM_SHOOT_MTR_PWR_SECOND;
        }
        else
        {
            Robot.ydexer.ydexerShootPower = Robot.ydexer.YDEXER_SHOOT_MTR_PWR;
            Robot.ydexer.ydexerSecondShootPower = Robot.ydexer.YDEXER_SHOOT_SECOND_MTR_PWR;
        }*/
    }


    public void setTargetRPM(double RPM)
    {
        shooterState = SHOOTER_STATE_OFF;

        targetRPM = 0.0;
        distance = 0.0;
        /*-----------------------------------------------------------------------------------------
        *  Determine RPM Based on Hood position
        *----------------------------------------------------------------------------------------*/
        if(RPM == SHOOTER_TARGET_RPM_VARIABLE_SHOOTER)
        {
            distance = Robot.vision.getDistanceToTarget();
            targetRPM = (0.0096*Math.pow(distance,2)) + (2.9341* distance) + 2037.9;
            setServoPosition((-7.0*Math.pow(10,-6)*Math.pow(distance,2)) + (0.004*distance) - 0.1272);

            /*System.out.println("dist: " + distance + ", trgtRPM: " + targetRPM + ", servo:"
            + ((-7.0*Math.pow(10,-6)*Math.pow(distance,2)) + (0.004*distance) - 0.1272));*/
            if(distance < 5.5)
            {
                targetRPM = SHOOTER_TARGET_RPM_FENDER;
                setServoPosition(SERVO_POSITION_FENDER);
            }
            else if(distance < 42)
            {
                targetRPM = SHOOTER_TARGET_RPM_FENDER;
                setServoPosition(SERVO_POSITION_FENDER);
            }
            else if(distance < 210.0)
            {
                //Use target RPM as is
            }
            else if(distance <= 250.0)
            {
                targetRPM = targetRPM * 0.95;
            }
            else if(distance <= 292)
            {
                targetRPM = targetRPM * 0.90;
            }
            else if(distance <= 340.0)
            {
                targetRPM = targetRPM * 0.86;
            }
            else
            {
                System.out.println("Dynamic shot invalid Distance");
                Robot.inShootingRange = false;
                targetRPM = targetRPM * OUT_OF_RANGE_SCALE_FACTOR;
            }

            Robot.ydexer.ydexerShootPower = Robot.ydexer.YDEXER_SHOOT_MTR_PWR;
            Robot.ydexer.ydexerSecondShootPower = Robot.ydexer.YDEXER_SHOOT_SECOND_MTR_PWR;
            
        }   
        else if(RPM == SHOOTER_TARGET_RPM_FENDER)
        {
            targetRPM = SHOOTER_TARGET_RPM_FENDER;
            setServoPosition(SERVO_POSITION_FENDER);
            Robot.ydexer.ydexerShootPower = Robot.ydexer.YDEXER_SHOOT_MTR_PWR;
            Robot.ydexer.ydexerSecondShootPower = Robot.ydexer.YDEXER_SHOOT_SECOND_MTR_PWR;
        }
        else if(RPM == SHOOTER_TARGET_RPM_TARMAC)
        {
            targetRPM = SHOOTER_TARGET_RPM_TARMAC;
            setServoPosition(SERVO_POSITION_TARMAC);
            /*if(inAutonomous)
            {
                targetRPM = targetRPM * 0.95;
            }*/
            Robot.ydexer.ydexerShootPower = Robot.ydexer.YDEXER_SHOOT_MTR_PWR;
            Robot.ydexer.ydexerSecondShootPower = Robot.ydexer.YDEXER_SHOOT_SECOND_MTR_PWR;
        }
        else if(RPM == SHOOTER_TARGET_RPM_DEFENSE_MODE)
        {
            targetRPM = SHOOTER_TARGET_RPM_DEFENSE_MODE;
            setServoPosition(SERVO_POSITION_DEFENSE);
            Robot.ydexer.ydexerShootPower = Robot.ydexer.YDEXER_SHOOT_MTR_PWR;
            Robot.ydexer.ydexerSecondShootPower = Robot.ydexer.YDEXER_SHOOT_SECOND_MTR_PWR;
        }
        else if(RPM == SHOOTER_TARGET_RPM_PREREV)
        {
            targetRPM = SHOOTER_TARGET_RPM_LAUNCHPAD; 
            setServoPosition(SERVO_POSITION_LAUNCHPAD);
            Robot.ydexer.ydexerShootPower = Robot.ydexer.YDEXER_SHOOT_MTR_PWR;
            Robot.ydexer.ydexerSecondShootPower = Robot.ydexer.YDEXER_SHOOT_SECOND_MTR_PWR;
        }
        else
        {
            targetRPM = 0.0;
        }
        
        
        /*-----------------------------------------------------------------------------------------
        *  Add offsets for Backspin
        *----------------------------------------------------------------------------------------*/
        shtrTopTargetRPM = targetRPM - SHOOTER_RPM_BACKSPIN_OFFSET / 2.0 ;
        shtrBtmTargetRPM = targetRPM + SHOOTER_RPM_BACKSPIN_OFFSET / 2.0 ;

        /*-----------------------------------------------------------------------------------------
        *  Calc Thresholds for top and bottom velocities
        *  Thresholds should not take into account PID offsets, therefore, we need to set the 
        *  threshold offsets before we add the PID Offsets
        *----------------------------------------------------------------------------------------*/
        Top_MaxRPM       = shtrTopTargetRPM + SHOOTER_MAX_RPM_OFFSET;
        Top_MinRPM       = shtrTopTargetRPM - SHOOTER_MIN_RPM_OFFSET;

        Btm_MaxRPM       = shtrBtmTargetRPM + SHOOTER_MAX_RPM_OFFSET;
        Btm_MinRPM       = shtrBtmTargetRPM - SHOOTER_MIN_RPM_OFFSET;

        /*-----------------------------------------------------------------------------------------
        *  Add offsets to target velocities to account for kP dropping velocity.  Once this issue
        *  is resolved, we can get rid of this.
        *----------------------------------------------------------------------------------------*/
        shtrTopTargetRPM = shtrTopTargetRPM + SHOOTER_RPM_PID_OFFSET_TOP;
        shtrBtmTargetRPM = shtrBtmTargetRPM + SHOOTER_RPM_PID_OFFSET_BTM;


    }


    
    public void disableShooterPID()
    {
        shtrMCTop.config_kP(SHOOTER_PID_IDX, 0.0);
        shtrMCTop.config_kI(SHOOTER_PID_IDX, 0.0);
        shtrMCTop.config_kD(SHOOTER_PID_IDX, 0.0);
        shtrMCTop.config_kF(SHOOTER_PID_IDX, 0.0);

        shtrMCBtm.config_kP(SHOOTER_PID_IDX, 0.0);
        shtrMCBtm.config_kI(SHOOTER_PID_IDX, 0.0);
        shtrMCBtm.config_kD(SHOOTER_PID_IDX, 0.0);
        shtrMCBtm.config_kF(SHOOTER_PID_IDX, 0.0);

    }

    public void enableShooterPID()
    {
        shtrMCTop.config_kP(SHOOTER_PID_IDX, PID_P_TOP);
        shtrMCTop.config_kI(SHOOTER_PID_IDX, PID_I);
        shtrMCTop.config_kD(SHOOTER_PID_IDX, PID_D);
        shtrMCTop.config_kF(SHOOTER_PID_IDX, PID_F_TOP);

        shtrMCBtm.config_kP(SHOOTER_PID_IDX, PID_P_BTM);
        shtrMCBtm.config_kI(SHOOTER_PID_IDX, PID_I);
        shtrMCBtm.config_kD(SHOOTER_PID_IDX, PID_D);
        shtrMCBtm.config_kF(SHOOTER_PID_IDX, PID_F_BTM);

    }

    

    /*-----------------------------Hood-----------------------------*/

    /*public void hoodSetBot()
    {   
        hoodSolenoid.set(Value.kReverse);
        hoodPosition = HOOD_BOT_POS;
    }

    public void hoodSetTop()
    {
        hoodSolenoid.set(Value.kForward);
        hoodPosition = HOOD_TOP_POS;
    }

    public int getHoodPosition()
    {
        return hoodPosition;
    }*/

    public boolean getShotSolutionAvail()
    {
        return shotSolution;
    }


    public void getServoPosition()
    {
        servoPosition = hoodServo1.getPosition();
        
        
        SmartDashboard.putNumber("Servo Position", servoPosition);
        
    }

    public void setServoPosition(double targetPosition)
    {
        hoodServo1.set(targetPosition);
        hoodServo2.set(targetPosition + SERVO_ONE_OFFSET);
        /*if(targetPosition < MIN_SERVO_POSITION)
        {
            hoodServo.set(MIN_SERVO_POSITION + 0.05);
        }
        else if(targetPosition > MAX_SERVO_POSITION)
        {
            hoodServo.set(MAX_SERVO_POSITION + 0.05);
        }
        else
        {
            hoodServo.set(targetPosition);
           
        }*/
    }

    /*-----------------------------------------------------------------------------------------
    *  
    * Smart Dashboard
    *
    *----------------------------------------------------------------------------------------*/
    public void smartDashboardShooter()
    {
        SmartDashboard.putNumber("topCurrRPM", MCTopMtrVelocityRPM);
        SmartDashboard.putNumber("btmCurrRPM", MCBtmMtrVelocityRPM);
        SmartDashboard.putNumber("topError", shtrTopTargetRPM - MCTopMtrVelocityRPM);
        SmartDashboard.putNumber("btmError", shtrBtmTargetRPM - MCBtmMtrVelocityRPM);
       
    }

    public void smartDashboardShooter_DEBUG()
    {
        SmartDashboard.putNumber("TMa" , Top_MaxRPM);
        SmartDashboard.putNumber("TMi" , Top_MinRPM);
        SmartDashboard.putNumber("BMa" , Btm_MaxRPM);
        SmartDashboard.putNumber("BMi" , Btm_MinRPM);
        SmartDashboard.putNumber("TargetRPMTop", shtrTopTargetRPM);
        SmartDashboard.putNumber("TargetRPMBot", shtrBtmTargetRPM);
    }

}
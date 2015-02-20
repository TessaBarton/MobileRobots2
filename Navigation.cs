using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 2;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;
        public double e_L_last = 0;
        public double e_R_last = 0;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;
        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    //WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            double K_p = 25;;
            double K_i = 0.1;
            double K_d = 1;

            double maxErr = 8000 / deltaT;


            e_L = desiredRotRateL - diffEncoderPulseL / deltaT;
            e_R = desiredRotRateR - diffEncoderPulseR / deltaT;

            e_sum_L = .9 * e_sum_L + e_L * deltaT;
            e_sum_R = .9 * e_sum_R + e_R * deltaT;

            e_sum_L = Math.Max(-maxErr, Math.Min(e_sum_L, maxErr));
            e_sum_R = Math.Max(-maxErr, Math.Min(e_sum_R, maxErr));

            u_L = ((K_p * e_L) + (K_i * e_sum_L) + (K_d * (e_L - e_L_last) / deltaT));
            e_L_last = e_L;

            u_R = ((K_p * e_R) + (K_i * e_sum_R) + (K_d * (e_R - e_R_last) / deltaT));
            e_R_last = e_R;
            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            //motorSignalL = (short)(zeroOutput + desiredRotRateL * 100);// (zeroOutput + u_L);
            //motorSignalR = (short)(zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            motorSignalL = (short)(zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

 
        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
            {
                // Setup Control
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(3, K_P, K_D, K_I);
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(4, K_P, K_D, K_I);

                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            }
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        double newNormalizeAngle(double angle)
        {
            double newAngle = angle;
            while (newAngle <= -3.1415) newAngle += 6.283;
            while (newAngle > 3.1415) newAngle -= 6.283;
            return newAngle;
        }
        private void FlyToSetPoint()
        {


            double goalX = desiredX -x;
            double goalY = desiredY-y;
            Console.WriteLine("X,Y (" + x.ToString() + " , " + y.ToString() + " )");
            Console.WriteLine("desired X,Y (" + desiredX.ToString() + " , " + desiredY.ToString() + " )");

            double desiredV, desiredW,pho,alpha,beta;
            if (goalX < 0)
            { //we are headed in the backward direction
                pho = Math.Sqrt(Math.Pow(goalX, 2.0) + Math.Pow(goalY, 2.0));// distance from curLoc to desLoc
                alpha = -dTheta + Math.Atan2(-goalY, -goalX);//
                beta = -dTheta - alpha;

                alpha = newNormalizeAngle(alpha);
                beta = newNormalizeAngle(beta);
                desiredV = -Kpho * pho;
                desiredW = Kalpha * alpha + Kbeta * beta;

            }
            else
            {// we are headed forward
                //transform coordinate systems
                pho = Math.Sqrt(Math.Pow(goalX, 2.0) + Math.Pow(goalY, 2.0)); //pho is linear distance
                alpha = -dTheta + Math.Atan2(goalY, goalX); // alpha is angle between robot facing and destination
                beta = -dTheta - alpha; //angle between pho idk
                alpha = newNormalizeAngle(alpha);
                beta = newNormalizeAngle(beta);
                desiredV = Kpho * pho;
                desiredW = (Kalpha * alpha) + (Kbeta * beta); //correct

            }
            Console.WriteLine("desired pho, alpha,beta (" + pho.ToString() + " , " + alpha.ToString() + " , " + beta.ToString() + " )");
            Console.WriteLine("desiredV, desiredW (" + desiredV.ToString() + " , " + desiredW.ToString() + " )");
            //calculate rotational velocities, convert to wheel rotation rates
            double rotVelocity1 = desiredW - (((desiredV / robotRadius) - desiredW) / 2);
            double rotVelocity2 = ((desiredV / robotRadius) - desiredW) / 2;
            desiredRotRateR = (short)(2 * robotRadius * rotVelocity1 / wheelRadius);
            desiredRotRateL = (short)(-2 * robotRadius * rotVelocity2 / wheelRadius);
            // convert to encoder pulses per second

            desiredRotRateR = (short)(desiredRotRateR * (1 / (2 * Math.PI * wheelRadius)) * 190);
            desiredRotRateL = (short)((-1) * desiredRotRateL * (1 / (2 * Math.PI * wheelRadius)) * 190);

            desiredRotRateR = Math.Min((short)encoderMax, desiredRotRateR);
            desiredRotRateL = Math.Min((short)encoderMax, desiredRotRateL);

            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, (short)desiredRotRateR, (short)desiredRotRateL, 0);
            else
            {
                realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, (short)desiredRotRateR, (short)desiredRotRateL, 0);
            }
        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()//CWiRobotSDK* m_MOTSDK_rob)
        {
            if ((lastEncoderPulseR - currentEncoderPulseR) > 16000)
            { // rollover 0 case 
                diffEncoderPulseR = currentEncoderPulseR + (encoderMax - lastEncoderPulseR);
            }
            else if ((lastEncoderPulseR - currentEncoderPulseR) < -16000) // rollover 32000 c
            {
                diffEncoderPulseR = lastEncoderPulseR + (encoderMax - currentEncoderPulseR);

            }
            else
            {
                diffEncoderPulseR = currentEncoderPulseR - lastEncoderPulseR;
            }// encoder ranges from 0 to 32,767 (encoderMax)
            if ((lastEncoderPulseL - currentEncoderPulseL) > 16000)
            {
                diffEncoderPulseL = currentEncoderPulseL + (encoderMax - lastEncoderPulseL);
            }
            else if ((lastEncoderPulseL - currentEncoderPulseL) < -16000) // if we are at 0 and drive backwards
            {
                diffEncoderPulseL = lastEncoderPulseL + (encoderMax - currentEncoderPulseL);

            }
            else
            {
                diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;
            }




            // update last encoder measurements
            lastEncoderPulseR = currentEncoderPulseR;
            lastEncoderPulseL = currentEncoderPulseL;
            //calculate distance traveled by wheels
            double angleTraveledR = diffEncoderPulseR * ((2 * Math.PI) / 190);
            double angleTraveledL = diffEncoderPulseL * ((2 * Math.PI) / 190);
            wheelDistanceR = wheelRadius * -angleTraveledR; //changed to minus sign
            wheelDistanceL = wheelRadius * angleTraveledL;
            // calculate angle and distance traveled from wheel speeds
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2; // distance calculated is the average of the wheel distances
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);

            // Console.WriteLine("DPulseL, DPulseR " + diffEncoderPulseL + ", " + diffEncoderPulseR + " WDL, WDR " + wheelDistanceL + ", " + whee

            
            
        }
 


        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // Update the actual
            // Console.WriteLine( "angle Travelled" + angleTravelled +"new angle" + (t+angleTravelled) +"new Theta" + normalizeAngle(t + angleTravelled, angleTravelled,t));
            double newTheta = normalizeAngle(t + angleTravelled / 2, angleTravelled / 2, t); // rotation is negative if angle travled is counterclockwise vice versa
            double deltaX = distanceTravelled * Math.Cos(newTheta); //deltaX is the x component of robot motion
            double deltaY = distanceTravelled * Math.Sin(newTheta);//deltaY is the y component of robot motion
            x = x + deltaX;
            y = y + deltaY;
            t = normalizeAngle(t + angleTravelled, angleTravelled, t);

            
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.

        public double normalizeAngle(double angle, double rotationDirection, double oldTheta)
        {
            //makes input angle between negative pi and pi
            bool CLOCKWISE = (rotationDirection > 0) ? false : true;//

            {
                if ((angle > Math.PI) && !CLOCKWISE && (oldTheta > 0)) // if we have been roating counterclockwise and transition below pi 
                {
                    return ((angle) % (Math.PI)) - Math.PI;
                }
                else if ((Math.Abs(angle) > Math.Abs(oldTheta)) && !CLOCKWISE && (angle < 0)) // cross zero going counterclockwise
                {
                    return angle + Math.PI;
                }
                else if ((Math.Abs(angle) > Math.PI) && CLOCKWISE && (oldTheta < 0)) // switch to positive angle if transitioning clockwise over pi
                {
                    return (Math.PI - (Math.Abs(angle) % (Math.PI)));
                }
                else if ((Math.Abs(angle) > Math.PI) && CLOCKWISE && (oldTheta >= 0))
                {
                    return angle % (Math.PI);
                }
                else
                {
                    return angle;
                }

            }
        }









        public void LocalizeRealWithIMU()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }
        #endregion

    }
}

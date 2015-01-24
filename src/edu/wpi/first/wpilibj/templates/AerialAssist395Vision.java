/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.NIVision.Rect;
   
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class AerialAssist395Vision extends IterativeRobot {
    /**
     * NOTE TO PROGRAMMERS: Constants can be initialized before the robotInit()
     * constructor, but variables should be initialized within robotInit(). In
     * general, constant names should be all caps, and variables should start
     * with a lower case letter. Attempt to make code as modular and logically
     * organized as possible.
     * 
     */
    
    /**
     * ------------------------------------------------------------------------
     * VISION PROCESSING INSTANCE VARIABLES (modified from 
     * VisionSampleProject2013.java)
     * ------------------------------------------------------------------------
     */
    final int XMAXSIZE = 24;
    final int XMINSIZE = 24;
    final int YMAXSIZE = 24;
    final int YMINSIZE = 48;
    final double xMax[] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 
                           .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
    final double xMin[] = {.4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1,
                           .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0};
    final double yMax[] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5,
                           .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
    final double yMin[] = {.4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05,
                           .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, 
                           .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, 
                           .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, 
                           .05, .05, .05, .05, .05, .6, 0};
    
    final int RECTANGULARITY_LIMIT = 60;
    final int ASPECT_RATIO_LIMIT = 75;
    final int X_EDGE_LIMIT = 40;
    final int Y_EDGE_LIMIT = 60;
    
    final int X_IMAGE_RES = 320;          // X Image resolution in pixels
    // final double VIEW_ANGLE = 43.5;       //Axis 206 camera
    final double VIEW_ANGLE = 48;       //Axis M1011 camera
    
    AxisCamera camera;       // the axis camera object (connected to the switch)
    CriteriaCollection cc;   // the criteria for doing particle filtering
    
    // Scores is a collection of instance variables characterizing the 
    // contents of images as either goals random blobs.
    public class Scores {
        double rectangularity;
        double aspectRatioInner;
        double aspectRatioOuter;
        double xEdge;
        double yEdge;
    }
    
    /**
     * ------------------------------------------------------------------------
     * OPERATION MODE CONSTANTS AND VARIABLES
     * ------------------------------------------------------------------------
     */

    final static boolean USE_PID_CONTROL_FOR_ARM = false;
    boolean autonTimerOn;
    final static double AUTON_DRIVE_TIME = 0.85; 
    boolean hotGoal;
    /**
     * ------------------------------------------------------------------------
     * ARM CONSTANTS AND VARIABLES
     * ------------------------------------------------------------------------
     */
    
    // CONSTANTS
    final static double ARM_POT_BOTTOM_LIMIT = 1.75;    // PICK-UP POSITION
    final static double ARM_POT_LOW_GOAL_SHOT = 1.84;   // LOW-GOAL POSITION
    final static double ARM_POT_HIGH_GOAL_SHOT = 2.30;  // SHOOT POSITION
    final static double ARM_POT_TOP_LIMIT = 2.51;       // START POSITION
    final static double ARM_POT_BUFFER = 0.03;          // position tolerance
    final static double ARM_MOTOR_SPEED = 0.80;         // speed for arm
    final static double ARM_PID_VALUE_P = 1.70;         // PID
    final static double ARM_PID_VALUE_I = 0.50;         // PID
    final static double ARM_PID_VALUE_D = 0.00;         // PID
    
    // VARIABLES 
    double DESIRED_ARM_MOTOR_SPEED;                     // target speed
    double CURRENT_ARM_MOTOR_SPEED;                     // current speed
    double DESIRED_ARM_MOTOR_POSITION;                  // target position
    double ARM_PID_PREVIOUS_ERROR;                      // target offset
    double ARM_PID_PREVIOUS_TIME;                       // Track time
    double ARM_PID_INTEGRAL;                            // (PID)
    
    
    
    /**
     * ------------------------------------------------------------------------
     * INTAKE ROLLER CONSTANTS AND VARIABLES
     * ------------------------------------------------------------------------
     */
    
    // CONSTANTS
    final static double INTAKE_MOTOR_SPEED = 0.8;
    final static int INTAKE_MOTOR_FORWARD_CORRECTION = 1;
   
    // VARIABLES
    double DESIRED_INTAKE_MOTOR_SPEED;
    double CURRENT_INTAKE_MOTOR_SPEED;     // Used to display current intake
                                           // motor speed
 
    
    
    /**
     * ------------------------------------------------------------------------
     * INTAKE JAW CONSTANTS AND VARIABLES
     * ------------------------------------------------------------------------
     */
    
    // VARIABLES
    boolean JAW_CLOSED;                          
    boolean JAW_OPEN;
    boolean JAW_CLOSING;
    boolean JAW_OPENING;
    
    
    
    /**
     * ------------------------------------------------------------------------
     * SHOOTER CONSTANTS AND VARIABLES
     * ------------------------------------------------------------------------
     */
    
    // CONSTANTS
    final static double SHOOTER_RETRACTOR_SPEED = -0.8; // Motor controller 
                                                        // input must be
                                                        // NEGATIVE!!!
    final static double SHOOTER_RELEASER_SPEED = -0.6;  // Motor controller input
    final static double SHOOTER_RELEASE_DESIRED_DELTA_T = 2.0;     // wait time 
                                                                   // for dog 
                                                                   // reset
    final static double DESIRED_SHOOT_DELAY = 2.0; // wait time for shooter
                                                   // mechanism rest
    boolean SHOOTER_MANUAL_CONTROL;                // For releaser, retractor
                                                   // and jaw
    final static double OPTIMUM_SHOOT_DISTANCE = 5.0;
            //HP Zone-12-24''
            //Zone Line-18''
            //Goal line -36''
    // VARIABLES
    double SHOOTER_PREVIOUS_TIME;               // Store Previous Time
    boolean SHOOTER_RELEASER_RESETTING;
    boolean SHOOTER_RETRACTOR_RESETTING;        // If retractor isn't initially reset, reset it
    boolean SHOOT_DELAY;
    boolean SHOOTING;
    int shootState;
    int autonStage;
    
    /**
     * ------------------------------------------------------------------------
     * SONAR CONSTANTS AND VARIABLES
     * ------------------------------------------------------------------------
     */
    
    // CONSTANTS
    final static double SONAR_INCHES_PER_VOLT = 102.4;      // 512/(Vcc = 5V)
    final static double SONAR_OFFSET = 0.0;                 // Calibration
    final static double SAMPLE_TIME = 0.01;                 // Seconds
    final static int DESIRED_SAMPLE_SIZE = 50;              // # of samples
    
    // VARIABLES
    double[] sonarSamples;
    int sampleCount;
    double distanceSum;
    double distance;
    boolean sonarSampling;
    
    /**
     * ------------------------------------------------------------------------
     * I/O BREAKOUT PINS AND I/O SENSOR INSTANCE DECLARATIONS
     * ------------------------------------------------------------------------
     */
    
    // DIGITAL I/O PINS
    final static int SHOOTER_RELEASE = 1;               // Digital I/O Pin
    final static int SHOOTER_RETRACTOR1 = 2;            // Digital I/O Pin
    final static int SHOOTER_RETRACTOR2 = 3;            // Digital I/O Pin
    final static int JAW_OPEN_PIN = 7;                  // Digital I/O Pin
    final static int JAW_CLOSED_PIN = 6;                // Digital I/O Pin
    
    // ANALOG BREAKOUT I/O PINS
    final static int ARM_POTENTIOMETER_PORT = 1;        // Analog I/O Pin
    final static int ULTRASONIC_PORT = 2;               // Analog I/O Pin

    // DIGITAL SENSORS
    DigitalInput shooterReleaseLimit;                   // I/O Pin 1
    DigitalInput shooterRetractorLimit1;                // I/O Pin 2
    DigitalInput shooterRetractorLimit2;                // I/O Pin 3
    DigitalInput jawOpenLimit;                          // I/O Pin 7
    DigitalInput jawClosedLimit;                        // I/O Pin 6

    // ANALOG SENSORS
    AnalogChannel armPot;                               // Analog Pin 1                    
    AnalogChannel ultraSonic;                           // Analog Pin 2
    
    
    
    /**
     * ------------------------------------------------------------------------
     * ROBOT CONTROL
     * ------------------------------------------------------------------------
     */
    
    // MOTOR CONTROLLERS
    Talon intakeMotor;      // INTAKE ROLLER
    Talon armMotor;         // ARM
    Talon leftFront;        // DRIVE
    Talon rightFront;       // DRIVE
    Talon leftBack;         // DRIVE
    Talon rightBack;        // DRIVE
    Talon releaser;         // SHOOTER
    Victor retractor;        // SHOOTER
    
    // WINDOW MOTOR ON SPIKE CONNECTED TO RELAY CHANNEL 1
    Relay intakeJawMotor;   // INTAKE JAW
    
    // DRIVE BASE
    RobotDrive chassis;
    
    
    /**
     * ------------------------------------------------------------------------
     * JOYSTICKS AND GAMEPADS
     * ------------------------------------------------------------------------
     */
    
    // JOYSTICKS AND GAME PADS
    ExtremePro3DJoystick extremeJoystick;   // JOYSTICK
    Joystick xbox360;                       // XBOX 360 CONTROLLER
    
    // JOYSTICK BUTTONS
    final static int triggerRight = 1;      // JOYSTICK TRIGGER
    final static int pro7 = 7;              // Enable Manual Shooter Control
    final static int pro8 = 8;              // Engage Retractor
    final static int pro9 = 9;              // 
    final static int pro10 = 10;            // Button 10
    final static int pro11 = 11;            // Button 11
    final static int pro12 = 12;            // Button 12
    
    // JOYSTICK CONSTANTS
    final static double TURN_TOLERANCE_PERCENT = 0.1;
    final static double DRIVE_TOLERANCE_PRECENT = 0.25;
    
    // JOYSTICK VARIABLES
    double forwardBack;           // y-axis value from extreme 3d pro joystick
    double twist;                 // z-axis value from extreme 3d pro joystick
    double jsButtonPreviousTime;  // Debounce for single button toggle
    boolean debounceDelay;        // debounce delay is occuring or not

    // XBOX 360 BUTTONS
    final static int AButton = 1;       // PICK-UP POSITION
    final static int BButton = 2;       // LOW GOAL POSITION
    final static int XButton = 3;       // START POSITION
    final static int YButton = 4;       // SHOOT POSITION
    final static int xleftBumper = 5;   // unused
    final static int xrightBumper = 6;  // unused
    final static int xBackButton = 7;    // unused
    final static int xStartButton = 8;   // SHOOT BALL (automated)
    final static int xLeftAxisX = 1;    // unused
    final static int xLeftAxisY = 2;    // ARM ANGLE [Up (-) / Down (+)]
    final static int xTriggers = 3;     // unused [Right (-) / Left (+)]
    final static int xRightAxisX = 4;   // unused
    final static int xRightAxisY = 5;   // GRABBER INTAKE [Up (-) / Down (+)]

    /**
     * ------------------------------------------------------------------------
     * TIMERS
     * ------------------------------------------------------------------------
     */
    Timer armPidTimer;
    Timer shootTimer;
    Timer joyStickDebounce;
    Timer sonarTimer;
    Timer autonTimer;
    
    
    
    public void robotInit() {
        
        /**
         * --------------------------------------------------------------------
         * INITIALIZE VARIABLES
         * --------------------------------------------------------------------
         */
        
        // OPERATION MODE VARIABLE
        autonTimerOn = false;
        hotGoal = false;
        
        // ARM VARIABLES 
        DESIRED_ARM_MOTOR_SPEED = 0.0; 
        CURRENT_ARM_MOTOR_SPEED = 0.0; // used to display the current arm speed
        DESIRED_ARM_MOTOR_POSITION = ARM_POT_TOP_LIMIT;  
        ARM_PID_PREVIOUS_ERROR = 0.0;
        ARM_PID_PREVIOUS_TIME = 0.0;
        ARM_PID_INTEGRAL = 0.0;
   
        // INTAKE MOTOR VARIABLES
        DESIRED_INTAKE_MOTOR_SPEED = 0;
        CURRENT_INTAKE_MOTOR_SPEED = 0; // used to display current intake
                                        // motor speed
        // SHOOTER VARIABLES
        SHOOTER_PREVIOUS_TIME = 0.0;     
        SHOOTER_RELEASER_RESETTING = false;
        SHOOTER_RETRACTOR_RESETTING = true;  // RESET RETRACTOR
        SHOOT_DELAY = false;
        SHOOTING = false;
        shootState = 0;
        SHOOTER_MANUAL_CONTROL = false;            // For releaser, retractor
                                                   // and jaw
        autonStage = 0;
        

        // SONAR VARIABLES
        sonarSamples = new double[DESIRED_SAMPLE_SIZE];
        sampleCount = 0;
        distanceSum = 0;
        distance = 0.0;
        sonarSampling = false;
        
        
        /**
         * --------------------------------------------------------------------
         * I/O BREAKOUT PINS AND I/O SENSOR INSTANCES
         * --------------------------------------------------------------------
         */
        
        // DIGITAL
        shooterReleaseLimit = new DigitalInput(SHOOTER_RELEASE);
        shooterRetractorLimit1 = new DigitalInput(SHOOTER_RETRACTOR1);
        shooterRetractorLimit2 = new DigitalInput(SHOOTER_RETRACTOR2);
        jawOpenLimit = new DigitalInput(JAW_OPEN_PIN);
        jawClosedLimit = new DigitalInput(JAW_CLOSED_PIN);
        
        // ANALOG
        armPot = new AnalogChannel(ARM_POTENTIOMETER_PORT);
        ultraSonic = new AnalogChannel(ULTRASONIC_PORT);
        
        
        /**
         * --------------------------------------------------------------------
         * ROBOT CONTROL: INSTANTIATE AND CALIBRATE ROBOT DRIVE CONTROL OBJECTS
         * --------------------------------------------------------------------
         */
        
        //armController = new PIDController(0.1, 0.001, 0.0, &armPot, &armMotor);

        // MOTOR CONTROLLERS
        intakeMotor = new Talon (3);        // INTAKE ROLLER
        armMotor = new Talon (4);           // ARM
        leftFront = new Talon (5);          // DRIVE
        rightFront = new Talon (6);         // DRIVE
        leftBack = new Talon (7);           // DRIVE
        rightBack = new Talon (8);          // DRIVE
        releaser = new Talon(9);            // SHOOTER
        retractor = new Victor(10);          // SHOOTER
        
        // WINDOW MOTOR ON SPIKE CONNECTED TO RELAY CHANNEL 1
        intakeJawMotor = new Relay(1);      // INTAKE JAW
        
        // ROBOT BASE
        chassis = new RobotDrive(leftFront, leftBack, rightFront, rightBack);
        chassis.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        chassis.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        chassis.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        chassis.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        
        
        /**
         * --------------------------------------------------------------------
         * JOYSTICK AND GAMEPAD INSTANCES AND VARIABLES
         * --------------------------------------------------------------------
         */

        // JOYSTICK INSTANCES
        extremeJoystick = new ExtremePro3DJoystick(1);
        xbox360 = new Joystick (2);
        
        // JOYSTICK VARIABLES
        forwardBack = 0.0;      // y-axis value from extreme 3d pro joystick
        twist = 0.0;            // z-axis value from extreme 3d pro joystick
        jsButtonPreviousTime = 0.0;     // delta-t for debounce calculation
        debounceDelay = false;
        
        
        /**
         * --------------------------------------------------------------------
         * TIMERS
         * --------------------------------------------------------------------
         */
        
        armPidTimer = new Timer();
        shootTimer = new Timer();
        joyStickDebounce = new Timer();
        sonarTimer = new Timer();
        autonTimer = new Timer();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        
        computeDistance();
        
        if (!autonTimerOn && autonStage == 0){
            autonTimerOn = true;
            autonTimer.reset();
            autonTimer.start();
            
        }
        if(autonTimerOn && autonTimer.get() > AUTON_DRIVE_TIME) {
            autonTimerOn = false;
            autonTimer.stop();
        }
        
        if(!autonTimerOn)
        {
            if (autonStage == 0) {
                
                forwardBack = 0.0;
                
                chassis.drive(forwardBack, 0.0);
                
                vision();
                
                autonStage = 1;
                
            }
            else if (autonStage == 1) {
                
                shootBall();
                
                if(!SHOOTING){
                    
                    autonStage = 2;
                    }
  
            }
        }
        
        else if ((distance > OPTIMUM_SHOOT_DISTANCE) && autonStage == 0){
            
             // Forward / Back acceleration
        
            double desiredFB = 1.0; // Max speed

            double fBerror = desiredFB - forwardBack;

            forwardBack += fBerror * DRIVE_TOLERANCE_PRECENT;
            
            chassis.drive(-forwardBack, 0.0); // Direction flipped
            
        }
        
        // The following should be placed in a separate method.
        autoSetArmShootPosition();
        
        printToConsole();
        
    }
    
    public void vision() {
        try {
            /**
             * Do the image capture with the camera and apply the algorithm
             * described above. This sample will either get images from the 
             * camera or from an image file stored in the top level 
             * directory in the flash memory on the cRIO. The file name in 
             * this case is "testImage.jpg"
             * 
             */
            ColorImage image = camera.getImage(); 

            // keep only red objects
            BinaryImage thresholdImage = image.thresholdHSV(40, 120, 40,
                                                            255, 20, 255);   
            // thresholdImage.write("/threshold.bmp"); // saves to cRio

            // fill in occluded rectangles
            BinaryImage convexHullImage = thresholdImage.convexHull(false);

            // convexHullImage.write("/convexHull.bmp"); // saves to cRio

            // filter out small particles
            BinaryImage filteredImage = convexHullImage.particleFilter(cc);           

            // filteredImage.write("/filteredImage.bmp"); // saves to cRio

            //Check each particle and score to see if it is a target
            Scores scores[] 
                    = new Scores[filteredImage.getNumberParticles()];

            for (int i = 0; i < scores.length; i++) {

                ParticleAnalysisReport report = 
                        filteredImage.getParticleAnalysisReport(i);

                scores[i] = new Scores();

                scores[i].rectangularity = scoreRectangularity(report);

                scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage,
                                                              report, i);

                scores[i].xEdge = scoreXEdge(thresholdImage, report);

                scores[i].yEdge = scoreYEdge(thresholdImage, report);

                // if any hot goal is detected among the particles identified,
                // and that hot goal is in the left hemisphere of the screen,
                // set hotGoal to true
                
                if (scoreCompare(scores[i])
                        && report.center_mass_x_normalized < - 0.1) {

                    hotGoal = true;

                }
                
            }

            /**
             * all images in Java must be freed after they are used since 
             * they are allocated out of C data structures. Not calling 
             * free() will cause the memory to accumulate over each pass of 
             * this loop.
             */
            filteredImage.free();
            convexHullImage.free();
            thresholdImage.free();
            image.free();

        } // End Try

         // this is needed if the camera.getImage() is called
        catch (AxisCameraException ex) {       
            ex.printStackTrace();
        } 

        catch (NIVisionException ex) {
            ex.printStackTrace();
        } 
    }
    
    /**
     * Computes the estimated distance to a target using the height of the
     * particle in the image. For more information and graphics showing the math
     * behind this approach see the Vision Processing section of the
     * ScreenStepsLive documentation.
     * 
     * @param image The image to use for measuring the particle estimated
     * rectangle
     * @param report The Particle Analysis Report for the particle
     * @param outer True if the particle should be treated as an outer target,
     * false to treat it as a center target
     * @return The estimated distance to the target in Inches.
     */
    double computeDistance (BinaryImage image, ParticleAnalysisReport report,
            
        int particleNumber) throws NIVisionException {
        
        double rectShort, height;
        
        int targetHeight;

        rectShort = NIVision.MeasureParticle(image.image, particleNumber,
                false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        
        //using the smaller of the estimated rectangle short side and the
        //bounding rectangle height results in better performance on skewed
        //rectangles
        height = Math.min(report.boundingRectHeight, rectShort);
        
        targetHeight = 4;

        return X_IMAGE_RES * targetHeight / 
                (height * 12 * 2 * Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
    }
    
    /**
     * Computes a score (0-100) comparing the aspect ratio to the ideal aspect
     * ratio for the target. This method uses the equivalent rectangle sides to
     * determine aspect ratio as it performs better as the target gets skewed by
     * moving to the left or right. The equivalent rectangle is the rectangle
     * with sides x and y where particle area= x*y
     * and particle perimeter= 2x+2y
     * 
     * @param image The image containing the particle to score, needed to
     * perform additional measurements
     * @param report The Particle Analysis Report for the particle, used for the
     * width, height, and particle number
     * @param outer Indicates whether the particle aspect ratio should be
     * compared to the ratio for the inner target or the outer
     * @return The aspect ratio score (0-100)
     */
    public double scoreAspectRatio(BinaryImage image,
        ParticleAnalysisReport report, int particleNumber)
        throws NIVisionException {
        
        double rectLong, rectShort, aspectRatio, idealAspectRatio;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false,
                MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        
        rectShort = NIVision.MeasureParticle(image.image, particleNumber, false,
                MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        
        //Dimensions of goal opening + 4 inches on all 4 sides for tape
        idealAspectRatio = 47/8;
	
        //Divide width by height to measure aspect ratio
        if(report.boundingRectWidth > report.boundingRectHeight){
            
            //particle is wider than it is tall, divide long by short
            aspectRatio = 100*(1-Math.abs((1-((rectLong/rectShort)/
                                                           idealAspectRatio))));
        } 
        
        else {
            
            //particle is taller than it is wide, divide short by long
                aspectRatio = 100*(1-Math.abs((1-((rectShort/rectLong)
                                                          /idealAspectRatio))));
        }
        
        //force to be in range 0-100
	return (Math.max(0, Math.min(aspectRatio, 100.0)));		
    }
    
    /**
     * Compares scores to defined limits and returns true if the particle
     * appears to be a target
     * 
     * @param scores The structure containing the scores to compare
     *
     * 
     * @return True if the particle meets all limits, false otherwise
     */
    boolean scoreCompare(Scores scores){
        
            boolean isTarget = true;

            isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
            
            isTarget &= scores.aspectRatioOuter > ASPECT_RATIO_LIMIT;
            
            isTarget &= scores.xEdge > X_EDGE_LIMIT;
            
            isTarget &= scores.yEdge > Y_EDGE_LIMIT;

            return isTarget;
    }
    
    /**
     * Computes a score (0-100) estimating how rectangular the particle is by
     * comparing the area of the particle to the area of the bounding box
     * surrounding it. A perfect rectangle would cover the entire bounding box.
     * 
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0-100)
     */
    double scoreRectangularity(ParticleAnalysisReport report){
            
        if(report.boundingRectWidth*report.boundingRectHeight !=0){
            
            return 100*report.particleArea 
                    /(report.boundingRectWidth*report.boundingRectHeight);
        } 
            
        else {
            
            return 0;
        }	
    }
    
    /**
     * Computes a score based on the match between a template profile and the
     * particle profile in the X direction. This method uses the the column
     * averages and the profile defined at the top of the sample to look for the
     * solid vertical edges with
     * a hollow center.
     * 
     * @param image The image to use, should be the image before the convex hull
     * is performed
     * 
     * @param report The Particle Analysis Report for the particle
     * 
     * @return The X Edge Score (0-100)
     */
    public double scoreXEdge(BinaryImage image, ParticleAnalysisReport report)
        throws NIVisionException {
        
            double total = 0;
            
            LinearAverages averages;

            Rect rect = new Rect(report.boundingRectTop,
                                 report.boundingRectLeft,
                                 report.boundingRectHeight,
                                 report.boundingRectWidth);
            
            averages = NIVision.getLinearAverages(image.image,
                  LinearAverages.LinearAveragesMode.IMAQ_COLUMN_AVERAGES, rect);
            
            float columnAverages[] = averages.getColumnAverages();
            
            for(int i=0; i < (columnAverages.length); i++){
                    if(xMin[(i*(XMINSIZE-1)/columnAverages.length)] 
                            < columnAverages[i] 
                       && columnAverages[i] < xMax[i*(XMAXSIZE-1)
                            /columnAverages.length]){
                            total++;
                    }
            }
            
            total = 100*total/(columnAverages.length);
            
            return total;
    }
    
    /**
	 * Computes a score based on the match between a template profile and
         * the particle profile in the Y direction. This method uses the row
         * averages and the profile defined at the top of the sample to look for
         * the solid horizontal edges with
	 * a hollow center
	 * 
	 * @param image The image to use, should be the image before the convex
         * hull is performed
	 * @param report The Particle Analysis Report for the particle
	 * 
	 * @return The Y Edge score (0-100)
	 *
    */
    public double scoreYEdge(BinaryImage image, ParticleAnalysisReport report)
        throws NIVisionException {
        
        double total = 0;
        
        LinearAverages averages;
        
        Rect rect = new Rect(report.boundingRectTop, report.boundingRectLeft,
                           report.boundingRectHeight, report.boundingRectWidth);
        
        averages = NIVision.getLinearAverages(image.image,
                    LinearAverages.LinearAveragesMode.IMAQ_ROW_AVERAGES, rect);
        
        float rowAverages[] = averages.getRowAverages();
        
        for(int i=0; i < (rowAverages.length); i++){
            
            if(yMin[(i*(YMINSIZE-1)/rowAverages.length)] < rowAverages[i] 
               && rowAverages[i] < yMax[i*(YMAXSIZE-1)/rowAverages.length]){
                
                total++;
                
            }
        }
        
        total = 100*total/(rowAverages.length);
        
        return total;
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        if(autonStage != 0 ) autonStage = 0;
        computeDistance();
        xBox360ControllerInput();
        driveControl();
        printToConsole();
    }
    
    /**
     * This method controls the Robot Drive based off values from the extreme
     * 3D pro joystick
     */
    public void driveControl(){
        
        // Turn acceleration
        
        double desiredTwist = extremeJoystick.getRawAxis(3);// Extreme 3D Pro Z-Axis

        double error = desiredTwist - twist;

        twist += error * TURN_TOLERANCE_PERCENT;
        
        // Forward / Back acceleration
        
        double desiredFB = extremeJoystick.getRawAxis(2); // Extreme 3D Pro Y-Axis

        double fBerror = desiredFB - forwardBack;
        
        forwardBack += fBerror * DRIVE_TOLERANCE_PRECENT;

        
        // Old joystick control
        // forwardBack = extremeJoystick.getRawAxis(2);   
        // twist = extremeJoystick.getRawAxis(3);
        
        chassis.arcadeDrive(forwardBack, twist);
    }
    
    
    /**
     * This method calls all methods responsive to XBox 360 controller button
     * clicks and XBox 360 controller thumb-stick axis changes, whether manual
     * or automated.
     */
    public void xBox360ControllerInput() {
        intakeRollerControl();  // manual control of intake roller
        manualArmControl();     // manual control of arm position
        presetArmControl();     // Preset arm position control
        shootBall();            // Automoated and Manual shooting (toggle with
                                // extreme pro 3D joystick button 8
    }
    
    /**
     * 
     */
    public void computeDistance() {
        
        if (!sonarSampling){
            sonarSampling = true;
            sonarTimer.reset();
            sonarTimer.start();
        }
        else {
            
            if (sonarTimer.get() > SAMPLE_TIME) {
                
                sonarSampling = false;
                sonarTimer.stop();
                
                if(sampleCount < sonarSamples.length){
                    
                    sonarSamples[sampleCount++] = ultraSonic.getVoltage();
                    
                }
                else {
                    
                    distanceSum = 0.0;
                    
                    for(int i = 0; i < sampleCount; i++) {
                        distanceSum += sonarSamples[i];
                    }
                    
                    distance = (distanceSum / ((double)sampleCount)) 
                                    * SONAR_INCHES_PER_VOLT;
                    
                    sampleCount = 0;
                }
            }
                
        }
        
    
    }
    
    /**
     * Set the intake motor speed
     * @param speed {double} Desired speed of motor 
     */
    public void setIntakeMotorSpeed(double speed) {
        DESIRED_INTAKE_MOTOR_SPEED = speed;
        CURRENT_INTAKE_MOTOR_SPEED = speed;
        intakeMotor.set(INTAKE_MOTOR_FORWARD_CORRECTION * speed);
    }
    
    
    /**
     * Set the arm position using PID control
     * 
     * This attempts to implement a very naive PID algorithm to determine the 
     * speed at which to drive the arm motor, then drives the arm motor
     * at that speed.
     * @param position {Double} Arm potentiometer value we want to attain
     */
    public void setArmPosition(double position) {
        // calculate difference in time 
        double currentTime = armPidTimer.getFPGATimestamp();
        double dt = currentTime - ARM_PID_PREVIOUS_TIME;
        ARM_PID_PREVIOUS_TIME = currentTime;
        
        // calculate error and update our error term
        double error = position - armPot.getVoltage();
        ARM_PID_INTEGRAL += dt * error;
        double derivative = (error - ARM_PID_PREVIOUS_ERROR)/dt;
        
        double output = ARM_PID_VALUE_P * error + 
                ARM_PID_VALUE_I * ARM_PID_INTEGRAL +
                ARM_PID_VALUE_D * derivative;
        ARM_PID_PREVIOUS_ERROR = error;
        
        setArmMotorSpeed(output);
    }
    
    
    /**
     * Set the arm motor speed.  
     * 
     * This will check the potentiometer values before running the motor to 
     * insure we do not violate the limits.
     * @param speed {double} Desired speed of motor
     */
    public void setArmMotorSpeed(double speed) {
        DESIRED_ARM_MOTOR_SPEED = speed;
        double actualSpeed;
        
        if (armPot.getVoltage() > ARM_POT_TOP_LIMIT) {
            // if we are too high only let us go down
            if (speed < 0) {
                actualSpeed = 0;
            } else {
                actualSpeed = speed;
            }
        } else if (armPot.getVoltage() < ARM_POT_BOTTOM_LIMIT) {
            // if we are too low only let us go up
            if (speed > 0) {
                actualSpeed = 0;
            } else {
                actualSpeed = speed;
            }
        } else {
            // We're okay to go!
            actualSpeed = speed;
        }
        
        armMotor.set(actualSpeed);
        CURRENT_ARM_MOTOR_SPEED = actualSpeed;
    }
    
    
    /**
     * This method sets the intake roller speed depending on X-Box controller
     * input, and then sets the motor to that speed by calling 
     * setIntakeMotorSpeed
     */
    public void intakeRollerControl() {
        
        double speed;
        if (xbox360.getRawAxis(xLeftAxisY) < - 0.5) {
            //Go in
            speed = -INTAKE_MOTOR_SPEED;
        } else if (xbox360.getRawAxis(xLeftAxisY) > 0.5){
            //Go out
            speed = INTAKE_MOTOR_SPEED;
        } else {
           // Do nothing
            speed = 0;
        }
        setIntakeMotorSpeed(speed);
    }
    
    
    /**
     * This method controls the arm motor based off input from the arm 
     * potentiometer and the x-box controller
     */
    public void manualArmControl () {
        double speed;
        if (xbox360.getRawAxis(xRightAxisY)> 0.5) {
            //go up!
            speed = -ARM_MOTOR_SPEED;
        } else if (xbox360.getRawAxis(xRightAxisY)< -0.5){
            //go down!
            speed = ARM_MOTOR_SPEED;
        } else {
            // do nothing
            speed = 0;
        }
        setArmMotorSpeed(speed);

    }
    
    
    /**
     * This method determines whether or not the shooter is aligned with the
     * pre-set optimum shooting angle, and returns a boolean representing
     * whether the angle is or is not aligned with the preset angle.
     * 
     * @return boolean value representing whether or not arm is aligned with
     * optimum shooting angle.
     */
    public boolean optimumShootAngle() {
        return (armPot.getVoltage() < ARM_POT_HIGH_GOAL_SHOT + ARM_POT_BUFFER 
                    && armPot.getVoltage()> ARM_POT_HIGH_GOAL_SHOT 
                        - ARM_POT_BUFFER);
    }
    
    
    /**
     * This method determines whether or not the shooter is aligned with the
     * pre-set optimum pick-up angle, and returns a boolean representing
     * whether the angle is or is not aligned with the preset angle.
     * 
     * @return boolean value representing whether or not arm is aligned with
     * optimum pick-up angle.
     */
    public boolean optimumPickUpAngle() {
        return (armPot.getVoltage() < ARM_POT_BOTTOM_LIMIT + ARM_POT_BUFFER 
                    && armPot.getVoltage()> ARM_POT_BOTTOM_LIMIT 
                        - ARM_POT_BUFFER);
    }
    
    
    /**
     * This method determines whether or not the shooter is aligned with the
     * pre-set optimum start angle, and returns a boolean representing
     * whether the angle is or is not aligned with the preset angle.
     * 
     * @return boolean value representing whether or not arm is aligned with
     * optimum start angle.
     */
    public boolean optimumLowGoalAngle() {
        return (armPot.getVoltage() < ARM_POT_TOP_LIMIT + ARM_POT_BUFFER 
                    && armPot.getVoltage()> ARM_POT_TOP_LIMIT 
                        - ARM_POT_BUFFER);
    }
    
    
    /**
     * This method determines whether or not the shooter is aligned with the
     * pre-set optimum low goal angle, and returns a boolean representing
     * whether the angle is or is not aligned with the preset angle.
     * 
     * @return boolean value representing whether or not arm is aligned with
     * optimum low-goal angle.
     */
    public boolean optimumStartAngle() {
        return (armPot.getVoltage() < ARM_POT_LOW_GOAL_SHOT + ARM_POT_BUFFER 
                    && armPot.getVoltage()> ARM_POT_LOW_GOAL_SHOT 
                        - ARM_POT_BUFFER);
    }
    
    
    /**
     * This method returns -1 if the value is less than the target, 0 if the
     * value is within the tolerance value of the target, and +1 if the value
     * is greater than the target
     * 
     * @param value double that is the current value for comparison with target
     * @param target double that is the target value
     * @param tolerance double that is the acceptable difference from the target
     * @return integer representing whether the value is less than, equal to
     * (within tolerance), or greater than target value [-1, 0, +1, 
     * respectively]
     */
    public int valOnTarget(double value, double target, double tolerance) {
        if(value < target - tolerance) {
            return -1;
        }
        else if (value > target + tolerance) {
            return 1;
        }
        else {
            return 0;
        }
    }
    
    
    /**
     * This method sets the arm to preset positions based off button clicks.
     * There are two available schemes: (1) PID (Untested), and (2) Naive 
     * boolean comparison based preset seeking
     */
    public void presetArmControl() {
        if (USE_PID_CONTROL_FOR_ARM) {
                if (xbox360.getRawButton(XButton)) {
                    DESIRED_ARM_MOTOR_POSITION = ARM_POT_TOP_LIMIT;
                } 
                else if (xbox360.getRawButton(AButton)){
                    DESIRED_ARM_MOTOR_POSITION = ARM_POT_BOTTOM_LIMIT;
                }
                else if (xbox360.getRawButton(BButton)) {
                    DESIRED_ARM_MOTOR_POSITION = ARM_POT_LOW_GOAL_SHOT;
                }
                else if (xbox360.getRawButton(YButton)) {
                    DESIRED_ARM_MOTOR_POSITION = ARM_POT_HIGH_GOAL_SHOT;
                }
                setArmPosition(DESIRED_ARM_MOTOR_POSITION);
        } else {
            
            // Relies on a physical break, backdrive protection will be jerky
            double speed;
            double potVal = armPot.getVoltage();
            int onTarget = 0; 
 
            if(xbox360.getRawButton(XButton)){
                DESIRED_ARM_MOTOR_POSITION = ARM_POT_TOP_LIMIT;
            }
            else if (xbox360.getRawButton(AButton)){
                DESIRED_ARM_MOTOR_POSITION = ARM_POT_BOTTOM_LIMIT;
            }
            else if (xbox360.getRawButton(BButton)) {
                DESIRED_ARM_MOTOR_POSITION = ARM_POT_LOW_GOAL_SHOT;
            }
            else if (xbox360.getRawButton(YButton)) {
                DESIRED_ARM_MOTOR_POSITION = ARM_POT_HIGH_GOAL_SHOT;
            }
            
            onTarget = valOnTarget(potVal, DESIRED_ARM_MOTOR_POSITION, 
                                    ARM_POT_BUFFER);
            
            if (onTarget < 0) {
                speed = -ARM_MOTOR_SPEED;
            }
            else if (onTarget > 0) {
                speed = ARM_MOTOR_SPEED;
            }
            else {
                speed = 0.0;
            }
            
            setArmMotorSpeed(speed);
        }
    }
    
    public void autoSetArmShootPosition() {
        DESIRED_ARM_MOTOR_POSITION = ARM_POT_HIGH_GOAL_SHOT;
        double potVal = armPot.getVoltage();
        double onTarget = valOnTarget(potVal, DESIRED_ARM_MOTOR_POSITION, 
                                ARM_POT_BUFFER);
        double speed;
        if (onTarget < 0) {
            speed = -ARM_MOTOR_SPEED;
        }
        else if (onTarget > 0) {
            speed = ARM_MOTOR_SPEED;
        }
        else {
            speed = 0.0;
        }

        setArmMotorSpeed(speed);
    }
    
    
    /**
     * This method toggles the SHOOTER_MANUAL_CONTROL boolean, and should be
     * called in the main teleopPeriodic() loop so that drivers and pit crew
     * can manually change the physical state of the shooting mechanisms. This
     * might need to be done occasionally for testing, or if the robot is shut
     * off during one of its automated sequences.
     */
    public void manualShootMode() {
        
        double delay = 0.5;
        
        if (extremeJoystick.getRawButton(pro8) && !debounceDelay){
            SHOOTER_MANUAL_CONTROL = !SHOOTER_MANUAL_CONTROL;
            debounceDelay = true;
            joyStickDebounce.reset();
            joyStickDebounce.start();
            
        }
        if(debounceDelay && joyStickDebounce.get() > delay) {
            debounceDelay = false;
            joyStickDebounce.stop();
        }
        
    }
    
    
    /**
     * If the relevant extreme 3D Pro joystick button is pressed, this method
     * opens and closes the jaw until the limit switches are the extremes of
     * the jaw's motion are engaged. This method should only be called if 
     * SHOOTER_MANUAL_CONTROL is true.
     */
    public void manualIntakeJaw() {
        
        if (SHOOTER_MANUAL_CONTROL) {
        
            if (extremeJoystick.getRawButton(pro11)){
                intakeJawMotor.set(Relay.Value.kReverse); // close
                //closeJaw();
            }
            else if (extremeJoystick.getRawButton(pro12)){
                intakeJawMotor.set(Relay.Value.kForward); // open
                //openJaw();
            }
            else {
                intakeJawMotor.set(Relay.Value.kOff);
            }
        }
    }
    
    
    /**
     * If the relevant extreme 3D Pro joystick buttons are pressed, this method
     * retracts the releaser until the releaser limit switch reads true. There
     * is no limit switch for extending the releaser back into position, and 
     * therefore when extending the releaser attention should be paid to whether
     * or not the dog will be engaged with the retraction mechanism. This
     * method should only be called if SHOOTER_MANUAL_CONTROL is true.
     */
    public void manualReleaser() {
        
        boolean isReleased = shooterReleaseLimit.get();

        if (extremeJoystick.getRawButton(pro9)){
            releaser.set(-SHOOTER_RELEASER_SPEED);
        }
        else if (extremeJoystick.getRawButton(pro10) && !isReleased) {
            releaser.set(SHOOTER_RELEASER_SPEED);
        }
        else {
            releaser.set(0.0);
        }
        
    }
    
    
    /**
     * If the relevant extreme 3D Pro joystick button is pressed, this method
     * retracts the shooter until the retractor limit switches read true. This
     * method should only be called if SHOOTER_MANUAL_CONTROL is true.
     */
    public void manualRetractor() {
        
        if(extremeJoystick.getRawButton(pro7) && (!shooterRetractorLimit1.get()
                && !shooterRetractorLimit2.get()) ) {
            retractor.set(SHOOTER_RETRACTOR_SPEED);
        }
        else {
            retractor.set(0.0);
        }
    }
    
    
    /**
     * This method calls the manual shooting mechanism control methods if the
     * boolean SHOOTER_MANUAL_CONTROL is true. Otherwise, the method does
     * nothing.
     */
    public void manualShootBall() {
        
        if (SHOOTER_MANUAL_CONTROL) {
            
            manualReleaser();
            manualRetractor();
            manualIntakeJaw();
        }

    }
    
    
    /**
     * This method will drive the window motor controlling the jaw until the 
     * jawOpenLimit limit switch reads true.
     */
    public void openJaw() {
        JAW_OPEN = jawOpenLimit.get();
        
        if(!JAW_OPEN){
            intakeJawMotor.set(Relay.Value.kForward); // open
            JAW_OPENING = true;
        }
        else {
            intakeJawMotor.set(Relay.Value.kOff);
            JAW_OPENING = false;
        }
    }
    
    
    /**
     * This method will drive the window motor controlling the jaw until the 
     * jawCl.osedLimit limit switch reads true.
     */
    public void closeJaw() {
        JAW_CLOSED = jawClosedLimit.get();
        if(!JAW_CLOSED){
            intakeJawMotor.set(Relay.Value.kReverse); // close
            JAW_CLOSING = true;
        }
        else {
            intakeJawMotor.set(Relay.Value.kOff);
            JAW_CLOSING = false;
        }
    }
    
    
    /**
     * This method automatically engages the retractor until the shooter is in
     * the loaded position (when the retractor limit switches read true, the
     * shooter is in the loaded position).
     */
    public void resetRetractor() {
        boolean isRetracted1 = shooterRetractorLimit1.get();
        boolean isRetracted2 = shooterRetractorLimit2.get();
        if(SHOOTER_RETRACTOR_RESETTING) {
            if(isRetracted1 || isRetracted2){
                retractor.set(0.0);
                SHOOTER_RETRACTOR_RESETTING = false;
            }
            else {
                retractor.set(SHOOTER_RETRACTOR_SPEED);
            }
        }
    }
    
    
    /**
     * This method defines the shoot sequence for manual tele-operated mode,
     * automated tele-operated mode (single button control), and for autonomous
     * mode [calling shootBall() automatically shoots the ball within 
     * autonomousPeriodic()]. For proper operation, shootState must be
     * initialized to zero, and SHOOTING must be set to false.
     */   
    public void shootBall(){
        
        manualShootMode();
        
        boolean isReleased = shooterReleaseLimit.get();
        
        if (!SHOOTER_MANUAL_CONTROL) {
        
            double deltaT = shootTimer.getFPGATimestamp()
                                    - SHOOTER_PREVIOUS_TIME;

            if(!SHOOTING) {
                if(optimumShootAngle() && (xbox360.getRawButton(xStartButton)
                        || isAutonomous())&& !SHOOTER_RETRACTOR_RESETTING
                            && shootState == 0){
                    SHOOTER_PREVIOUS_TIME = shootTimer.getFPGATimestamp(); // might need to do this here...
                    SHOOTING = true;
                    shootState = 1;
                }
                
                resetRetractor();
                
            }
            else {

                if(SHOOT_DELAY && shootState == 3) {
                    if(deltaT > DESIRED_SHOOT_DELAY){
                        SHOOTER_RELEASER_RESETTING = true;
                        SHOOTER_RETRACTOR_RESETTING = true;
                        SHOOT_DELAY = false;
                        shootState = 4;
                        
                    }
                    else {
                        if(isReleased){
                            releaser.set (-SHOOTER_RELEASER_SPEED);
                        }
                        else{
                            releaser.set(0.0);     // stop releaser once mechanism backs off limit switch
                            retractor.set(0.0);    // redundancy not needed?
                        }
                    }
                }

                if (SHOOTER_RELEASER_RESETTING && shootState == 4) {

                    if (deltaT > SHOOTER_RELEASE_DESIRED_DELTA_T 
                            + DESIRED_SHOOT_DELAY){
                        SHOOTER_RELEASER_RESETTING = false;
                        releaser.set (0.0);
                    }
                    else {
                        releaser.set (-SHOOTER_RELEASER_SPEED);
                    }

                }
                else {
                    
                    if(shootState == 1) {
                        openJaw();
                        if(jawOpenLimit.get()) {
                            shootState = 2;
                        }
                    }

                    if(isReleased && shootState == 2) {
                        SHOOTER_PREVIOUS_TIME = shootTimer.getFPGATimestamp();
                        SHOOT_DELAY = true;
                        shootState = 3;

                    }
                    else{
                        if(!SHOOT_DELAY && shootState == 2 && !isReleased){
                            releaser.set (SHOOTER_RELEASER_SPEED);
                        }

                    }

                }
                
                resetRetractor();
                

                if(!SHOOTER_RETRACTOR_RESETTING && !SHOOTER_RELEASER_RESETTING
                        && (deltaT > SHOOTER_RELEASE_DESIRED_DELTA_T 
                            + DESIRED_SHOOT_DELAY) && shootState == 4) {
                    closeJaw();
                    if(jawClosedLimit.get()) {
                        SHOOTING = false;
                        shootState = 0;
                    }     
                }

            }
            
        }
        
        else {        
             manualShootBall();  
        }
       
    }
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        // Unused
    }
    
    
    /**
     * Pad the input string to be the length of a line on the driver's LCD.
     * @param input {String} string to pad
     * @return {String} string that is the length of the driver's lcd
     */
    public String padStringForLcd(String input) {
        String output = input;
        while (output.length() < DriverStationLCD.kLineLength) {
            output += " ";
        }
        return output;
    }
    
    
    /**
     * This method generates strings to print to the LCD on lines 1-6, then
     * outputs those string to the LCD.
     */
    public void printToConsole() {
        String line1;
        if (USE_PID_CONTROL_FOR_ARM) {
            line1 = "Arm" + DESIRED_ARM_MOTOR_POSITION +
                "|" + armPot.getVoltage() + "|" + ARM_PID_PREVIOUS_ERROR;
        } else {
            line1 = "Arm Pot Value: " + armPot.getVoltage();
        }
        
        String line2 = "hotGoal: " + hotGoal;
        
        String line3 = "Auton Stage: " + autonStage; // auto toggling between manual and not manual?
        
        String line4 = "Auton Timer: " + autonTimer.get();
                /*"In w " + 
                DESIRED_INTAKE_MOTOR_SPEED + "h " +
                CURRENT_INTAKE_MOTOR_SPEED;*/
        
        
        String line5 = "Distance[in]: " + distance;
        
        String line6 = "Manual Shooter: " + SHOOTER_MANUAL_CONTROL;

        
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser1, 
                                               1,
                                               padStringForLcd(line1));
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 
                                               1,
                                               padStringForLcd(line2));
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 
                                               1,
                                               padStringForLcd(line3));
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 
                                               1,
                                               padStringForLcd(line4));
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 
                                               1,
                                               padStringForLcd(line5));
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 
                                               1,
                                               padStringForLcd(line6));
        
        DriverStationLCD.getInstance().updateLCD();
        
    }
    
}
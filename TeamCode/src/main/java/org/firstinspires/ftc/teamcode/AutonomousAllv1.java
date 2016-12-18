/**
 * Created by Ishaan on 12/7/2016.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import java.io.File;
import java.util.Date;
import java.util.Locale;

/**
 * Created by Ishaan on 12/7/2016.
 */

@Autonomous(group = "auto", name = "Auto All")
public class AutonomousAllv1 extends LinearVisionOpMode {

    int frameCount = 0;

    // DC motors
    DcMotor lf, lb, rf, rb, shooter, sweeper;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Servo leftBeacon, rightBeacon, shooterGate;
    Servo leftBottomCapBall, rightBottomCapBall;


    ModernRoboticsI2cRangeSensor ultrasonic;
    ModernRoboticsAnalogOpticalDistanceSensor light;
    ElapsedTime timer;

    //states in sequencer
    enum stateMachine {
        start, timeDelay, slideState, getColor, pushBeacon, shooterWheel, triggerGate, stop, pivotRobot
    };

    //Create State Machine Object
    stateMachine state;

    //dataloger
    DataLogger data;

    Date day = new Date();

    //Intialize Counter Variables
    int noOfBeaconsPressed = 0;
    int seqCounter = 0;

    //Create Drive Train Object
    MecanumDrive drive;

    //Holds data for beacon color detection
    String rightOrLeft = "";
    String rightColor = "";

    //Cap Ball Starting Servo Positions
    double leftBottomCapBallStart = 1.0;
    double rightBottomCapBallStart = 0.0;

    // Shooter and shooter gate
    double shooterGateOpen = 0;
    double shooterGateClosed = 0.5;
    double shooterGateOpenTime = 1.0;
    double shooterWheelPower = 0.41;

    //Max Encder Ticks Per Seconds for our Shooter Motor
    int shooterEncoderMax = 3080;

    //Counts Per Inch
    double countsPerInch = 119.4;

    // positions of the beacons' servo
    double leftBeaconRetract = 1;
    double leftBeaconExtend = 0;
    double rightBeaconRetract = 0;
    double rightBeaconExtend = 1;

    // Trigger for light sensor
    double whiteLightTrigger = 0.3;

    // Shooting distance from wall
    double shootingDistance = 50;

    // Speeds with Corresponding Gains
    double highPower = .75;
    double highGain = 0.01;

    double midPower = 0.5;
    double midGain = 0.01;

    double lowPower = 0.3;
    double lowGain = 0.05;

    double allPower = 1;
    double allPowerGain = 0.01;


    //Counters for verifying accuracy for beacon color detection
    int redCount = 0, blueCount = 0, blankCount = 0;

    //Starting delay
    int autoStartDelay = 0;


    public static double endGyro;

    //Beacon Analysis Object
    Beacon.BeaconAnalysis beaconAnalysis;

    String autoProgram;

    //Create Arrays for all 4 Autonomous Programs
    Object[][] sequenceArray;

    Object[][] blueFullArray = {
            // STEP 1. START
            {stateMachine.start},

            //STEP 2. Add a delay in seconds if needed
            {stateMachine.timeDelay, autoStartDelay},

            // Slide State Parameters
            //      Angle -- Angle at which you slide
            //      Power -- power for the motors
            //      Switch Case -- what conditions we are testing (for example less than encoder)
            //      Parameter for the test case
            //      Orientation -- Orientation for the robot
            //      Gain for the angle correction
            // STEP 3. Move at a 45 degree angle until encoders are past -4100 at power 1
            // Robot is at 0 orientation (facing forward) )

            //STEP 3. Move 36 inches at angle of 45 degrees at full power
            {stateMachine.slideState, 45, allPower, 1, 36.0, 0.0, allPowerGain, allPower, lowPower, 1.0},


            // STEP 4. Move towards the wall until the ultrasonic sensor is 25 cm, and then 15 cm; orientation is 0
            {stateMachine.slideState, 90, allPower, 3, midPower, 0.01, 25.0, 0.0, highGain},
            {stateMachine.slideState, 45, allPower, 3, midPower, 0.01, 15.0, 0.0, midGain},

            // STEP 5. Move forward at slow speed until you see light sensor at Light Trigger
            {stateMachine.slideState, 0, lowPower, 6, whiteLightTrigger, 0.0, lowGain},

            //STEP 6. Get in position to read color
            {stateMachine.slideState, 0, midPower, 1, 2.0, 0.0, midGain, midPower, lowPower, 0.00004},

            //STEP 7. Read color
            {stateMachine.getColor},

            //STEP 8. Line up to the beacon
            {stateMachine.slideState, 180, lowPower, 1, 2.0, 0.0, midGain, midPower, lowPower, 0.00004},

            //STEP 9. Extend correct beacon pusher
            {stateMachine.pushBeacon, 1, "blue"}, // 0 is retract, 1 is push

            //STEP 10. Slide to Beacon with Pusher Extended
            {stateMachine.slideState, 90, allPower, 3, midPower, 0.01, 7.0, 0.0, highGain},

            // STEP 11. Start wheel shooter
            {stateMachine.shooterWheel, shooterWheelPower},

            // STEP 12. Back off the wall to the shooting distance from the wall
            {stateMachine.slideState, -90, highPower, 4, shootingDistance, 0.0, highGain},

            // STEP 13. Retract beacon pusher
            {stateMachine.pushBeacon, 0, "blue"}, // retracts beacon

            // STEPS 14-17. Shoot 1st particle, wait 1 sec, Shoot 2nd particle, stop shooter
            {stateMachine.triggerGate, shooterGateOpenTime},

            {stateMachine.shooterWheel, 0.0},
            {stateMachine.slideState, 50, allPower, 3, highPower, 0.01, 15.0, 0.0, highGain},
            {stateMachine.slideState, 0, midPower, 1, 18.0, 0.0, midGain, allPower, allPower, 0.00004},

            //STEP 18. Slide to second white line at low speed
            {stateMachine.slideState, 0, lowPower, 6, whiteLightTrigger, 0.0, lowGain},

            //STEP 19. Get in position to read color
            {stateMachine.slideState, 0, midPower, 1, 2.0, 0.0, midGain, midPower, lowPower, 0.00004},

            //STEP 20. Read color
            {stateMachine.getColor},

            //STEP 21. Line up to the beacon
            {stateMachine.slideState, 180, lowPower, 1, 2.0, 0.0, midGain, midPower, lowPower, 0.00004},

            //STEP 22. Extend correct beacon pusher
            {stateMachine.pushBeacon, 1, "blue"}, // 0 is retract, 1 is push

            //STEP 23. Slide to Beacon with Pusher Extended
            {stateMachine.slideState, 90, allPower, 3, midPower, 0.01, 7.0, 0.0, highGain},

            // STEP 24. Back off wall
            {stateMachine.slideState, -90, highPower, 4, 35.0, 0.0, highGain},

            // STEP 25. Retract beacon pusher
            {stateMachine.pushBeacon, 0, "blue"},

            //STEP 26. Pivot robot to 45 degrees
            {stateMachine.pivotRobot, 30.0, 0.01, 1, highPower, lowPower/3},

            //STEP 27. Drive at full power to cap ball to push it and park
            {stateMachine.slideState, 180, null, 1, 44.0, 45.0, allPowerGain, allPower, allPower, 0.00004},

            //STEP 28. Stop robot
            {stateMachine.stop},
    };

    //Array for full red autonomous
    Object [][] redFullArray = {

            // Slide State Parameters
            //      Angle -- Angle at which you slide
            //      Power -- power for the motors
            //      Switch Case -- what conditions we are testing (for example less than encoder)
            //      Parameter for the test case
            //      Orientation -- Orientation for the robot
            //      Gain for the angle correction
            // STEP 3. Move at a 45 degree angle until encoders are past -4100 at power 1
            // Robot is at 0 orientation (facing forward) )


            // STEP 1. START
            {stateMachine.start},

            //STEP 2. Add a delay in seconds if needed
            {stateMachine.timeDelay, 0},

            //STEP 3. Slide 36 inches at 135 degrees
            {stateMachine.slideState, 135, null, 1, 36.0, 0.0, allPowerGain, allPower, lowPower, 1.0},

            //STEP 4. Move towards wall until ultrasonic reads 25, then 15
            {stateMachine.slideState, 90, allPower, 3, midPower, 0.01, 25.0, 0.0, highGain},
            {stateMachine.slideState, 135, allPower, 3, midPower, 0.01, 15.0, 0.0, midGain},

            //STEP 5. Move forward at slow speed until we see the line
            {stateMachine.slideState, 180, lowPower, 6, whiteLightTrigger, 0.0, lowGain},

            //STEP 6. Get in position to read color
            {stateMachine.slideState, 0, null, 1, 2.0, 0.0, midGain, midPower, lowPower, 0.00004},

            //STEP 7. Read color
            {stateMachine.getColor},

            //STEP 8. Line up to beacon
            {stateMachine.slideState, 180, null, 1, 2.0, 0.0, midGain, midPower, lowPower, 0.00004},

            //STEP 9. Extend correct beacon pusher
            {stateMachine.pushBeacon, 1, "red"}, //0 is retract, 1 is extend

            //STEP 10. Slide towards beacon with beacon pusher extended
            {stateMachine.slideState, 90, allPower, 3, highPower, 0.01, 7.0, 0.0, highGain},

            //STEP 11. Start shooter wheel
            {stateMachine.shooterWheel, shooterWheelPower},

            //STEP 12. Back off the wall to the shooting distance from the wall
            {stateMachine.slideState, 270, highPower, 4, shootingDistance, 0.0, highGain},

            //STEP 13. Retract Beacon Pusher
            {stateMachine.pushBeacon, 0, "red"}, // retracts beacon pusher

            //STEP 14-15. Shoot 1st particle, wait, shoot second particle.
            {stateMachine.triggerGate, shooterGateOpenTime},
            //Set 0 power to shooter
            {stateMachine.shooterWheel, 0.0},

            //STEP 16. Slide at 130 degrees towards the wall
            {stateMachine.slideState, 130, allPower, 3, highPower, 0.01, 15.0, 0.0, highGain},

            //STEP 17. Slide fast for 15 inches towards beacon line
            {stateMachine.slideState, 180, null, 1, 18.0, 0.0, midGain, allPower, allPower, 0.00004},

            //STEP 18. Slide to second white line at slow speed
            {stateMachine.slideState, 180, lowPower, 6, whiteLightTrigger, 0.0, lowGain},

            //STEP 19. Get in position to read color
            {stateMachine.slideState, 0, null, 1, 2.0, 0.0, midGain, midPower, lowPower, 0.00004},

            //STEP 20. Read color
            {stateMachine.getColor},

            //STEP 21. Line up to beacon
            {stateMachine.slideState, 180, null, 1, 2.0, 0.0, midGain, midPower, lowPower, 0.00004},

            //STEP 22. Extend correct beacon pusher
            {stateMachine.pushBeacon, 1, "red"}, //0 is retract, 1 is extend

            //STEP 23. Slide towards beacon with beacon pusher extended
            {stateMachine.slideState, 90, allPower, 3, highPower, 0.01, 7.0, 0.0, highGain},

            //STEP 24. Back off wall
            {stateMachine.slideState, 270, highPower, 4, 35.0, 0.0, highGain},

            //STEP 25. Retract beacon pusher
            {stateMachine.pushBeacon, 0, "red"}, //retracts beacon

            //STEP 26. Pivot robot to 330
            {stateMachine.pivotRobot, 330, 0.01, 1, highPower, lowPower/3},

            //STEP 27. Drive at full power to cap ball to push it and park
            {stateMachine.slideState, 0, null, 1, 44.0, 330.0, allPowerGain, allPower, allPower, 0.00004},

            //STEP 28. Stop robot
            {stateMachine.stop},



            /*
            {stateMachine.slideState, 135, allPower, 1, 2050.0, 0.0, allPowerGain},


            // STEP 4. Move towards the wall until the ultrasonic sensor is 40 cm; orientation is 0
            {stateMachine.slideState, 90, highPower, 3, 45.0, 0.0, highGain},

            // STEP 5. Move forward fast to get near line encoder position -955
            {stateMachine.slideState, 180, highPower, 1, 700.0, 0.0, highGain},

            // STEP 6. Move forward at slow speed until you see light sensor at Light Trigger
            {stateMachine.slideState, 180, lowPower, 6, whiteLightTrigger, 0.0, lowGain},

            // STEP 7. Move to position to get camera image
            {stateMachine.slideState, 0, midPower, 2, -290.0, 0.0, midGain},

            //getColor State has 2 parameter,
            //  1. which is the number of steps to skip if you don't resolve the color
            //  2. Number of seconds to get the image

            // STEP 8. Get the beacon colors
            {stateMachine.getColor, 1, 1},


            // STEP 9. Slide back to the white line
            {stateMachine.slideState, 120, midPower, 6, whiteLightTrigger, 0.0, midGain},

            // pushBeacon is either 1 for extend or 0 for retract must run the getColor state before this.
            //
            // STEP 10. Extend the beacon pusher
            {stateMachine.pushBeacon, 1, "red"}, // 0 is retract, 1 is push

            // STEP 11. Slide back to white line slowly if there was overshoot
            {stateMachine.slideState, 0, lowPower, 6, whiteLightTrigger, 0.0, lowGain},


            // STEP 12. Slide to wall until 12 cm to push the beacon.
            // A new case in slideState to be able to skip if the beacon is not detected
            {stateMachine.slideState, 90, highPower, 7, pushBeaconDistance, 0.0, midGain},

*/
            // STEP 13. Start wheel shooter
           /* {stateMachine.shooterWheel, shooterWheelPower},

            // STEP 14. Back off the wall to the shooting distance from the wall
            {stateMachine.slideState, 270, highPower, 4, shootingDistance, 0.0, highGain},


            // STEP 15. Retract beacon pusher
            {stateMachine.pushBeacon, 0, "red"}, // retracts beacon

            // STEP 16. Move forward to shooting position
            //{stateMachine.slideState, 0, midPower, 2, -290.0, 0.0, midGain},

            // STEPS 17-20. Shoot 1st particle, wait 1 sec, Shoot 2nd particle, stop shooter
            {stateMachine.triggerGate, shooterGateOpenTime},

            {stateMachine.shooterWheel, 0.0},

            // STEP 21 Move forward to second line at speed .75
            {stateMachine.slideState, 180, highPower, 1, 2784.96, 0.0, highGain},

            // STEP 22 Move sideways slowly to 40 cm from wall
            {stateMachine.slideState, 90, highPower, 3, 45.0, 0.0, highGain},

            // STEP 23 Move slowly to line
            {stateMachine.slideState, 180, lowPower, 6, whiteLightTrigger, 0.0, lowGain},

            // STEP 24  Move to the camera position
            {stateMachine.slideState, 0, midPower, 2, -290.0, 0.0, midGain},

            // STEP 25. Get the beacon colors
            {stateMachine.getColor, 1, 1},

            // STEP 26. Slide back to the white line
            {stateMachine.slideState, 120, midPower, 6, whiteLightTrigger, 0.0, midGain},

            // STEP 27. Extend the beac0n pusher
            {stateMachine.pushBeacon, 1, "red"}, // 0 is retract, 1 is push

            // STEP 28. Slide back to white line slowly if there was overshoot
            {stateMachine.slideState, 0, lowPower, 6, whiteLightTrigger, 0.0, lowGain},

            // STEP 29. Slide to wall until 12 cm to push the beacon.
            {stateMachine.slideState, 90, highPower, 7, pushBeaconDistance, 0.0, highGain},

            // STEP 30. Back off wall
            {stateMachine.slideState, 270, highPower, 4, 35.0, 0.0, highGain},

            // STEP 31. Retract beacon pusher
            {stateMachine.pushBeacon, 0, "red"},

            // STEP 32. slide to cap ball
            {stateMachine.slideState, 315, allPower, 5, 3.0, 0.0, allPowerGain},
            {stateMachine.stop},*/
    };

    //Array for slide and shoot program for blue alliance
    Object[][] blueSlideAndShootArray = {
            // STEP 1. START
            {stateMachine.start},

            //STEP 2. Add a delay in seconds if needed
            {stateMachine.timeDelay, 7},

            // Slide State Parameters
            //      Angle -- Angle at which you slide
            //      Power -- power for the motors
            //      Switch Case -- what conditions we are testing (for example less than encoder)
            //      Parameter for the test case
            //      Orientation -- Orientation for the robot
            //      Gain for the angle correction

            //STEP 3. Start shooter
            {stateMachine.shooterWheel, shooterWheelPower},


            //STEP 4. Slide at -45 degrees until -3300 enconder counts at orientation 0.0
            {stateMachine.slideState, -45, null, 1, 40.0, 0.0, allPowerGain, allPower, highPower, 1.0},
            // {stateMachine.slideState, -45, allPower, 2, -1900.0, 0.0, allPowerGain},


            //STEP 5. Slide at 0 degrees until -820 encoder counts at orientation 0.0
            {stateMachine.slideState, 0, null, 1, 12.0, 0.0, midGain, midPower, midPower, 1.0},
            {stateMachine.slideState, 0, null, 1, 12.0, 0.0, lowGain, lowPower, lowPower, 1.0},

            //{stateMachine.slideState, 0, highPower, 2, -700.0, 0.0, highGain},
            //{stateMachine.slideState, 0, lowPower, 2, -700.0, 0.0, lowGain},


            //STEP 6. Shoot
            {stateMachine.triggerGate, shooterGateOpenTime},
            {stateMachine.timeDelay, 2}, // If you want to wait before starting, start here
            {stateMachine.triggerGate, shooterGateOpenTime},
            //STEP 7. Stop shooter
            {stateMachine.shooterWheel, 0.0},

            //STEP 8. Push Cap ball
            {stateMachine.slideState, 300, null, 1, 12.0, 0.0, midGain, midPower, midPower, 1.0},
            //{stateMachine.slideState, 300, highPower, 2, -750.0, 0.0, highGain},

            //STEP 9. Move back to shooting position
            {stateMachine.slideState, 120, null, 1, 12.0, 0.0, midGain, midPower, midPower, 1.0},
            //{stateMachine.slideState, 120, highPower, 1, 750.0, 0.0, highGain},

            //STEP 8. Slide at 0 degrees until -3300 encoder counts to push cap ball
            {stateMachine.slideState, 270, allPower, 5, 2.0, 0.0, allPowerGain},

            //STEP 9. Stop program, move to final state
            {stateMachine.stop},
    };

    //Array for slide and shoot program for red alliance
    Object[][] redSlideAndShootArray = {
            // STEP 1. START
            {stateMachine.start},

            //STEP 2. Add a delay in seconds if needed
            {stateMachine.timeDelay, 5},

            // Slide State Parameters
            //      Angle -- Angle at which you slide
            //      Power -- power for the motors
            //      Switch Case -- what conditions we are testing (for example less than encoder)
            //      Parameter for the test case
            //      Orientation -- Orientation for the robot
            //      Gain for the angle correction

            //STEP 3. Start shooter
            {stateMachine.shooterWheel, shooterWheelPower},

            //STEP 4. Slide at -45 degrees until -3300 enconder counts at orientation 0.0
            {stateMachine.slideState, -135, null, 1, 40.0, 0.0, allPowerGain, allPower, highPower, 1.0},
            //{stateMachine.slideState, -135, allPower, 1, 3800.0, 0.0, allPowerGain},

            //STEP 5. Slide at 0 degrees until -820 encoder counts at orientation 0.0
            {stateMachine.slideState, 180, null, 1, 12.0, 0.0, midGain, midPower, midPower, 1.0},
            {stateMachine.slideState, 180, null, 1, 12.0, 0.0, lowGain, lowPower, lowPower, 1.0},
            //{stateMachine.slideState, 180, highPower, 1, 700.0, 0.0, highGain},
            //{stateMachine.slideState, 180, lowPower, 1, 700.0, 0.0, lowGain},

            //STEP 6. Shoot
            {stateMachine.triggerGate, shooterGateOpenTime},
            {stateMachine.timeDelay, 2}, // If you want to wait before starting, start here
            {stateMachine.triggerGate, shooterGateOpenTime},

            //STEP 7. Stop shooter
            {stateMachine.shooterWheel, 0.0},

            //STEP 8. Push Cap ball
            {stateMachine.slideState, 250, null, 1, 12.0, 0.0, midGain, midPower, midPower, 1.0},
            //{stateMachine.slideState, 250, highPower, 1, 2000.0, 0.0, highGain},

            //STEP 9. Move back to shooting position
            {stateMachine.slideState, 70, null, 1, 12.0, 0.0, midGain, midPower, midPower, 1.0},
            //{stateMachine.slideState, 70, highPower, 2, -2000.0, 0.0, highGain},

            //STEP 8. Slide at 0 degrees until -3300 encoder counts to push cap ball
            {stateMachine.slideState, 270, allPower, 5, 2.0, 0.0, allPowerGain},
            //{stateMachine.slideState, 270, allPower, 5, 1.5, 0.0, allPowerGain},

            //STEP 9. Stop program, move to final state
            {stateMachine.stop},
    };
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.parseDouble(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    public int average4Motors(int a, int b, int c, int d) {
        return (int) (a + b + c + d) / 4;
    }
    public double averageEncoders(){
        double encoderA = (lf.getCurrentPosition() + rb.getCurrentPosition()) / 2;
        double encoderB = (rf.getCurrentPosition() + lb.getCurrentPosition()) / 2;
        double encoderAverage = (Math.abs(encoderA) + Math.abs(encoderB)) / 2;
        return encoderAverage;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        data = new DataLogger(day.toString() + " autonomous data");

        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();
        telemetry.addData("Init", "vision started");
        telemetry.update();


        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.VVAnalysis);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();



        telemetry.addData("Init", "Open CV setup");
        telemetry.update();

        //call motors from hardware map
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");

        leftBeacon = hardwareMap.servo.get("left_beacon");
        rightBeacon = hardwareMap.servo.get("right_beacon");
        shooterGate = hardwareMap.servo.get("shooter_gate");

        leftBottomCapBall = hardwareMap.servo.get("lb_cap_ball");
        rightBottomCapBall = hardwareMap.servo.get("rb_cap_ball");

        leftBeacon.setPosition(leftBeaconRetract);
        rightBeacon.setPosition(rightBeaconRetract);
        shooterGate.setPosition(shooterGateClosed);

        leftBottomCapBall.setPosition(leftBottomCapBallStart);
        rightBottomCapBall.setPosition(rightBottomCapBallStart);

        //make it so when the powers are set to 0, the motors will stop and not move
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse the directions of the right motors
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ultrasonic = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonic");
        light = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "light");

        shooter = hardwareMap.dcMotor.get("shooter");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMaxSpeed(shooterEncoderMax);

        Date day = new Date();
        data = new DataLogger(day.toString() + " light data");
        telemetry.addData("Init", "hardware done");
        telemetry.update();

        timer = new ElapsedTime();
        timer.reset();

        state = stateMachine.start;

        noOfBeaconsPressed = 0;
        seqCounter = 0;

        telemetry.addData("Init", "imu calibrating");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);// Get the calibration data
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

        // Save the calibration data to a file. You can choose whatever file
        // name you wish here, but you'll want to indicate the same file name
        // when you initialize the IMU in an opmode in which it is used. If you
        // have more than one IMU on your robot, you'll of course want to use
        // different configuration file names for each.
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());

        telemetry.addData("Init", "imu calibrated");
        telemetry.update();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        drive = new MecanumDrive(rf, rb, lf, lb, imu, light, ultrasonic, null, telemetry, timer);
        telemetry.addData("Init", "done");
        telemetry.update();
        boolean selected = false;

        //Use gamepad inputs to choose which program will run
        while(!selected){
            if(gamepad1.x){
                telemetry.addData("Program selected", "Blue Full Autonomous");
                selected = true;
                sequenceArray = blueFullArray;
                autoProgram = "blueFull";
            }else if(gamepad1.b){
                telemetry.addData("Program selected", "Red Full Autonomous");
                selected = true;
                sequenceArray = redFullArray;
                autoProgram = "redFull";
            }else if(gamepad1.y){
                telemetry.addData("Program selected", "Blue Slide and Shoot");
                selected = true;
                sequenceArray = blueSlideAndShootArray;
                autoProgram = "blueSlide";
            }else if(gamepad1.a){
                telemetry.addData("Program selected", "Red Slide and Shoot");
                selected = true;
                sequenceArray = redSlideAndShootArray;
                autoProgram = "redSlide";
            }else{
                telemetry.addData("Press X", "Blue Full Autonomous");
                telemetry.addData("Press B", "Red Full Autonomous");
                telemetry.addData("Press Y", "Blue Slide and Shoot");
                telemetry.addData("Press A", "Red Slide and Shoot");
            }
            telemetry.update();
        }
        waitForStart();

        while(opModeIsActive()){
            switch ((stateMachine) sequenceArray[seqCounter][0]) {
                case start:
                    seqCounter++;
                    timer.reset();
                    break;

                //Starting Delay. Waits for time in seconds before starting the autonomous program
                case timeDelay:
                    if (timer.seconds() < (int) sequenceArray[seqCounter][1]) {

                    } else {
                        seqCounter++;
                        timer.reset();
                        drive.resetEncoders();
                    }
                    telemetry.addData("state", "delay");
                    telemetry.addData("time", timer.seconds());
                    break;

                //Reads the color of the right side of the beacon
                case getColor:
                    beaconAnalysis = beacon.getAnalysis();
                    if (beacon.getAnalysis().isRightRed()) { //Cheks to see if red
                        rightColor = "red";
                        redCount ++;

                    } else if (beacon.getAnalysis().isRightBlue()) { //Checks to see if blue
                        rightColor = "blue";
                        blueCount ++;
                    }
                    else{ //Checks to see if no color was found
                        blankCount ++;
                    }


                    if(redCount > 20 || blueCount > 20){ //Verifies that correct color was chosen
                        noOfBeaconsPressed++;

                        telemetry.addData("right color", rightColor);
                        telemetry.addData("blue count", blueCount);
                        telemetry.addData("red count", redCount);
                        telemetry.addData("blank count", blankCount);
                        //Write data to the Excel Spreadsheet
                        data.addField(blueCount);
                        data.addField(redCount);
                        data.addField(blankCount);
                        data.newLine();
                        //Reset counters and timers
                        redCount = 0;
                        blueCount = 0;
                        blankCount = 0;
                        timer.reset();
                        drive.resetEncoders();
                        seqCounter++;
                    } else if (blankCount > 50){
                        noOfBeaconsPressed++;
                        //reset counters
                        redCount = 0;
                        blueCount = 0;
                        blankCount = 0;
                        //Write data to excel spreadsheet
                        data.addField(blueCount);
                        data.addField(redCount);
                        data.addField(blankCount);
                        data.newLine();
                        telemetry.addData("right color", rightColor);
                        telemetry.addData("blue count", blueCount);
                        telemetry.addData("blue count", redCount);
                        telemetry.addData("blue count", blankCount);
                        timer.reset();
                        drive.resetEncoders();
                        seqCounter+=4;
                    }
                    break;

                //Determines which beacon pusher to extend, based on alliance and the color readings
                case pushBeacon:
                    // push for 1. retract for any other value.
                    telemetry.addData("color", rightOrLeft);
                    if ((int) sequenceArray[seqCounter][1] == 1) {
                        if(((String)sequenceArray[seqCounter][2]).equals("red")){
                            if(rightOrLeft.equals("red, blue")){
                                leftBeacon.setPosition(leftBeaconExtend);
                                rightBeacon.setPosition(rightBeaconRetract);
                            }
                            else if(rightOrLeft.equals("blue, red")){
                                rightBeacon.setPosition(rightBeaconExtend);
                                leftBeacon.setPosition(leftBeaconRetract);
                            }else{
                                leftBeacon.setPosition(leftBeaconRetract);
                                rightBeacon.setPosition(rightBeaconRetract);
                            }
                        }
                        else if(((String)sequenceArray[seqCounter][2]).equals("blue")){
                            if(rightColor.equals("blue")){
                                leftBeacon.setPosition(leftBeaconRetract);
                                rightBeacon.setPosition(rightBeaconExtend);
                            }
                            else if(rightColor.equals("red")){
                                rightBeacon.setPosition(rightBeaconRetract);
                                leftBeacon.setPosition(leftBeaconExtend);
                            } else{
                                leftBeacon.setPosition(leftBeaconRetract);
                                rightBeacon.setPosition(rightBeaconRetract);
                            }
                        }
                        else{
                            leftBeacon.setPosition(leftBeaconRetract);
                            rightBeacon.setPosition(rightBeaconRetract);
                        }

                    } else { // argument is not 1. retract both beacons.
                        leftBeacon.setPosition(leftBeaconRetract);
                        rightBeacon.setPosition(rightBeaconRetract);
                    }
                    seqCounter++;
                    timer.reset();
                    break;

                //State for moving the robot. Contains 8 different case for 8 different cases to check while sliding
                case slideState:
                    switch ((int)sequenceArray[seqCounter][3]) {
                        case 1: //goes while encoder count is less than desired. next parameter is the desired encoder count.
                            double encoderAverage = averageEncoders(); //get current position of the encoders
                            double encoderError = ((double)sequenceArray[seqCounter][4] * countsPerInch) - encoderAverage; //encoderError = target encoder distance - current encoder position
                            double encoderSpeed = (encoderError * (double)sequenceArray[seqCounter][9]) + (double)sequenceArray[seqCounter][8]; // Speed = error*gain + minSpeed
                            if(encoderSpeed > (double)sequenceArray[seqCounter][7]){ //If the speed is greater than the maxSpeed, set the speed to the max speed
                                encoderSpeed = (double)sequenceArray[seqCounter][7];
                            }
                            else if(encoderSpeed < (double)sequenceArray[seqCounter][8]){ //If the speed is less than the minSpeed, set the speed to the min speed
                                encoderSpeed = (double)sequenceArray[seqCounter][8];
                            }

                            if(encoderError > 0){ //Move the robot while the encoderError is greater than 0
                                drive.slideAngleIMU((int) sequenceArray[seqCounter][1], encoderSpeed, true, (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]);
                            }
                            else{
                                drive.setPowerAll(0, 0, 0, 0);
                                timer.reset();
                                drive.resetEncoders();
                                seqCounter ++;
                            }

                            break;
                        case 2: //goes while encoder is greater than desired
                            double backEncoder = average4Motors(rf.getCurrentPosition(), rb.getCurrentPosition(), lf.getCurrentPosition(), lb.getCurrentPosition());;
                            if(drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], backEncoder > (double)sequenceArray[seqCounter][4], (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                                telemetry.addData("state", "slide encoder ");
                                telemetry.addData("Encoder average", backEncoder);

                                data.newLine();
                            }
                            else{
                                timer.reset();
                                drive.resetEncoders();
                                seqCounter ++;
                            }
                            break;
                        case 3: //goes while US level is greater than desired
                            //Parameters: Angle, Max Power, Case, Min Power, gain, desired distance, orientation, oGain
                            double currentDistance = ultrasonic.cmUltrasonic(); //get the current ultrasonic value
                            double error = currentDistance - (double)sequenceArray[seqCounter][6]; //error = current distance - desired distance
                            double speed = (double)sequenceArray[seqCounter][4] + (error * (double)sequenceArray[seqCounter][5]); //s[eed to minSpeed + (error * gain)
                            if(speed > (double)sequenceArray[seqCounter][2]){ //if the speed is greater than the maxSpeed, set the speed to the maxSpeed
                                speed = (double)sequenceArray[seqCounter][2];
                            }
                            else if(speed < (double)sequenceArray[seqCounter][4]){ // If the speed is less than the minSpeed, set the speed to the minSpeed
                                speed = (double)sequenceArray[seqCounter][4];
                            }

                            if(error > 0){ //Move the robot while the calculated error is greater than 0. It will stop once the distance has been reached
                                drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), speed, true, (double) sequenceArray[seqCounter][7], (double) sequenceArray[seqCounter][8]);
                                telemetry.addData("current distance", currentDistance);
                                telemetry.addData("error", error);
                                telemetry.addData("state", "ultrasonic");
                            }
                            else{
                                drive.setPowerAll(0, 0, 0, 0);
                                timer.reset();
                                drive.resetEncoders();
                                seqCounter++;
                            }

                            break;

                        case 4: //goes while US level is less than desired
                            if(drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], ultrasonic.cmUltrasonic() < (double)sequenceArray[seqCounter][4] , (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                                telemetry.addData("state", "US Level ");
                                telemetry.addData("ultrasonic", ultrasonic.cmUltrasonic());
                            }
                            else{
                                timer.reset();
                                drive.resetEncoders();
                                seqCounter++;
                            }
                            break;

                        case 5: //goes while timer is less than desired
                            if(drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], timer.seconds() < (double)sequenceArray[seqCounter][4] , (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                                telemetry.addData("state", "time ");
                                telemetry.addData("ultrasonic", ultrasonic.cmUltrasonic());
                            }
                            else{
                                timer.reset();
                                drive.resetEncoders();
                                seqCounter++;
                            }
                            break;

                        case 6: //goes while light value is less than desired
                            if(drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], light.getLightDetected() < (double)sequenceArray[seqCounter][4], (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                                telemetry.addData("state", "light");
                                telemetry.addData("light value", light.getLightDetected());
                            }
                            else{
                                timer.reset();
                                drive.resetEncoders();
                                seqCounter++;
                            }
                            break;
                        case 7: //Moves only if we have a color detected
                            if (rightOrLeft.equals("red, blue")|| rightOrLeft.equals("blue, red")) {
                                if (drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], ultrasonic.cmUltrasonic() > (double) sequenceArray[seqCounter][4], (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1) {
                                    telemetry.addData("state", ultrasonic.cmUltrasonic());
                                } else {
                                    timer.reset();
                                    drive.resetEncoders();
                                    seqCounter++;
                                }
                            } else {
                                timer.reset();
                                drive.resetEncoders();
                                seqCounter++;
                            }
                            break;
                    }
                    break;

                case stop: //Stops robot
                    drive.setPowerAll(0, 0, 0, 0);
                    timer.reset();
                    telemetry.addData("state", "stop");
                    telemetry.addData("US level", ultrasonic.cmUltrasonic());
                    telemetry.addData("right color", rightColor);
                    telemetry.addData("blue count", blueCount);
                    telemetry.addData("red count", redCount);
                    telemetry.addData("blank count", blueCount);
                    break;

                case shooterWheel: //Sets shooter wheel to specific power
                    shooter.setPower((double) sequenceArray[seqCounter][1]);
                    seqCounter++;
                    timer.reset();
                    break;

                case triggerGate: //Releases both particles
                    // Shoot first particle by opening gate.
                    if(timer.seconds() < shooterGateOpenTime)
                        shooterGate.setPosition(shooterGateOpen);
                    else {
                        shooterGate.setPosition(shooterGateClosed);
                        seqCounter++;
                        timer.reset();
                    }
                    break;

                case pivotRobot: //Pivots robot to a desired angle
                    // pivot to angle for a certain time
                    if (timer.seconds()<(int) sequenceArray[seqCounter][3]){
                        drive.pivotToAngleIMU((double) sequenceArray[seqCounter][1], (double) sequenceArray[seqCounter][2], true, (double) sequenceArray[seqCounter][4],(double) sequenceArray[seqCounter][5]);
                        } else
                    {
                        timer.reset();
                        drive.resetEncoders();
                        seqCounter++;
                    }
                    break;
            }
            telemetry.addData("color", rightOrLeft);
            telemetry.update();
            if (hasNewFrame()) {
                //Get the frame
                Mat rgba = getFrameRgba();
                Mat gray = getFrameGray();

                //Discard the current frame to allow for the next one to render
                discardFrame();

                //Do all of your custom frame processing here
                //For this demo, let's just add to a frame counter
                frameCount++;
            }
        }

    }
    @Override
    public void stop(){
        //Write IMU calibration file, so we can read the file in teleop
        super.stop();
        String filename = "lastGyroValue.txt";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        float gyroStop;
        if(autoProgram.equals("blueFull")){
            gyroStop = -imu.getAngularOrientation().firstAngle;
        }else if(autoProgram.equals("redFull")){
            gyroStop = (-imu.getAngularOrientation().firstAngle+180)%360;
        }else if(autoProgram.equals("blueSlide")){
            gyroStop = (-imu.getAngularOrientation().firstAngle-90)%360;
        }else if(autoProgram.equals("redSlide")){
            gyroStop = (-imu.getAngularOrientation().firstAngle-90)%360;
        }else{
            gyroStop = -imu.getAngularOrientation().firstAngle;
        }
        ReadWriteFile.writeFile(file, String.valueOf(gyroStop));
    }

}


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DataLogger;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.opencv.core.Size;

/**
 * Created by Ishaan Oberoi on 11/26/2016.
 */
@Autonomous(name = "test auto methods")
public class AutoBlueNewDrive extends VisionOpMode {
    DcMotor lf, lb, rf, rb;
    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Servo buttonPusherLeft;
    Servo buttonPusherRight;
    CRServo gripper;
    ModernRoboticsI2cRangeSensor ultrasonic;
    ModernRoboticsAnalogOpticalDistanceSensor light;
    ElapsedTime timer;
    ModernRoboticsI2cColorSensor color;
    //states in sequencer
    enum stateMachine {
        slideState, timeDelay, testTelemetry, grip, pivotCapBall, getColor, goToBeacon2Line, transition, retractServos, start, estop, delay, slide45Distance, slide90Ultrasonic, slide0light, waitBeforeGoingBack, slide180Light, pivotTo0Time, slideToBeacon, pushBeacon, checkColor, waitForBeaconPusher, retractBeaconPusher, slideNeg45ToBeacon2, slide30in, testGetToPosition, slide45Cap, pivotToNeg45, slide20US, capBall, stop
    };
    stateMachine state;
    //dataloger
    DataLogger data;
    int delay = 0;
    int beaconPresssed = 0;
    int seqCounter = 0;
    double firstDistance = 42;
    MecanumDrive drive;
    String rightOrLeft;
    Object[][] sequenceArray = new Object[][]{
            {stateMachine.start},
            {stateMachine.start},
            {stateMachine.timeDelay, 2},
            {stateMachine.slideState, 45, 1.0, 2, -3341.953451, 0.0, 0.05},
            //{stateMachine.slideState, 90, .75, 3, 47, 0.0, 0.04},
            // {stateMachine.slideState, 0, .25, 6, .15, 0.0, 0.02},

            //{stateMachine.slideState, 90, .25, 3, 20, 0.0, 0.02},
            {stateMachine.stop}
    };
    @Override
    public void init() {
        super.init();
        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));

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
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

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

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        //call motors from hardware map
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        buttonPusherLeft = hardwareMap.servo.get("left_beacon");
        buttonPusherRight = hardwareMap.servo.get("right_beacon");
        //gripper = hardwareMap.crservo.get("beacon_brace");
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
       // light = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "light");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
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
        timer = new ElapsedTime();

        firstDistance *= 127.5;
        state = stateMachine.start;
        drive = new MecanumDrive(rf, rb, lf, lb, imu, light, ultrasonic, null, telemetry, timer);

        beaconPresssed = 0;
        seqCounter = 0;
    }

    public int average2Motors(int a, int b) {
        return (int) (a + b) / 2;
    }

    public int average4Motors(int a, int b, int c, int d) {
        return (int) (a + b + c + d) / 4;
    }
    public void loop() {
        switch ((stateMachine) sequenceArray[seqCounter][0]) {
            case start:
                seqCounter++;
                telemetry.addData("stae", "start");
                timer.reset();
                break;

            case timeDelay:
                if (timer.seconds() < (int) sequenceArray[seqCounter][1]) {
                    //          telemetry.addData("state", "delay");
                    //        telemetry.addData("time", timer.seconds());
                    //       telemetry.addData("delay", (int) sequenceArray[seqCounter][1]);
                } else {
                    seqCounter++;
                    timer.reset();
                    drive.resetEncoders();
                }
                break;

            case getColor:
                rightOrLeft = beacon.getAnalysis().getColorString();
                if(rightOrLeft.equals("???, ???")||beacon.getAnalysis().getConfidence()<.05){
                    rightOrLeft = beacon.getAnalysis().getColorString();
                    telemetry.addData("color", rightOrLeft);
                    telemetry.addData("Confidence", beacon.getAnalysis().getConfidence());
                }
                else{
                    timer.reset();
                    drive.resetEncoders();
                    seqCounter++;
                }
                break;

            case grip:
                gripper.setPower((double)sequenceArray[seqCounter][1]);
                if (timer.seconds() < (double)sequenceArray[seqCounter][2]) {
                    telemetry.addData("color sensor", color.red());
                    data.addField(color.red());
                    data.addField(color.blue());
                    data.addField(rightOrLeft);
                    data.newLine();
                } else {
                    gripper.setPower(0);
                    timer.reset();
                    drive.resetEncoders();
                    seqCounter++;
                }

                break;

            case pushBeacon:

                telemetry.addData("color", rightOrLeft);




                if (timer.seconds() < (double) sequenceArray[seqCounter][2]) {
                    if (rightOrLeft.equals("red, blue")) {
                        gripper.setPower(1);
                        buttonPusherLeft.setPosition(0);
                    } else if(rightOrLeft.equals("blue, red")) {
                        gripper.setPower(1);
                        buttonPusherRight.setPosition(0);
                    }
                } else {
                    gripper.setPower(0);
                    buttonPusherLeft.setPosition(.9);
                    buttonPusherRight.setPosition(.7);
                    seqCounter++;
                    timer.reset();
                    drive.resetEncoders();
                }


                break;

            case testTelemetry:
                if(timer.seconds()<5){
                    telemetry.addData("state 13", "");
                }
                else{
                    seqCounter++;
                    timer.reset();
                }
                break;

            case slideState:
                //if (drive.slideAngle(((Integer) sequenceArray[counter][1]).doubleValue(), (double) sequenceArray[counter][2], ((Boolean) sequenceArray[counter][3]).booleanValue(), (double) sequenceArray[counter][4], (double) sequenceArray[counter][5]) == 1) {
                switch ((int)sequenceArray[seqCounter][3]) {
                    case 1: //goes while encoder count is less than desired
                        double encoderAverage = average4Motors(rf.getCurrentPosition(), rb.getCurrentPosition(), lf.getCurrentPosition(), lb.getCurrentPosition());
                        if(drive.slideAngleIMU(0.0, (double) sequenceArray[seqCounter][2], encoderAverage < (double)sequenceArray[seqCounter][4], (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                            telemetry.addData("state", "slide encoder ");

                        }
                        else{
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
                        }
                        else{
                            timer.reset();
                            drive.resetEncoders();
                            seqCounter ++;
                        }
                        break;
                    case 3: //goes while US level is greater than desired
                        if(drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], ultrasonic.cmUltrasonic() > (double)sequenceArray[seqCounter][4] , (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                            telemetry.addData("state", "US Level ");
                        }
                        else{
                            timer.reset();
                            drive.resetEncoders();
                            seqCounter++;
                        }
                        break;

                    case 4: //goes while US level is less than desired
                        if(drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], ultrasonic.cmUltrasonic() < (double)sequenceArray[seqCounter][4] , (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                            telemetry.addData("state", "US Level ");
                        }
                        else{
                            timer.reset();
                            drive.resetEncoders();
                            seqCounter++;
                        }
                        break;

                    case 5: //goes while US level is greater than desired
                        if(drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], timer.seconds() < (double)sequenceArray[seqCounter][4] , (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                            telemetry.addData("state", "time ");
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
                }
                break;
            case stop:
                drive.setPowerAll(0, 0, 0, 0);
                telemetry.addData("state", "stop");
        }
    }
}
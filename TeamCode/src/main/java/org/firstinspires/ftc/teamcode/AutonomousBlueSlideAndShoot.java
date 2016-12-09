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
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import java.io.File;
import java.util.Date;
import java.util.Locale;

import static android.R.attr.name;

/**
 * Created by Jyoti on 12/8/2016.
 */
@Autonomous(name = "Slide and Shoot Blue", group = "Autonomous")
public class AutonomousBlueSlideAndShoot extends LinearVisionOpMode{

    // DC motors
    DcMotor lf, lb, rf, rb, shooter, sweeper;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Servo leftBeacon, rightBeacon, shooterGate;

    ModernRoboticsI2cRangeSensor ultrasonic;
    ModernRoboticsAnalogOpticalDistanceSensor light;
    ElapsedTime timer;

    //states in sequencer
    enum stateMachine {
        start, timeDelay, slideState, getColor, pushBeacon, shooterWheel, triggerGate, stop, pivotRobot
    };

    stateMachine state;

    //dataloger
    DataLogger data;

    Date day = new Date();

    int noOfBeaconsPressed = 0;
    int seqCounter = 0;

    MecanumDrive drive;
    String rightOrLeft;

    // Shooter and shooter gate
    double shooterGateOpen = 0;
    double shooterGateClosed = 0.5;
    double shooterGateOpenTime = 1.0;
    double shooterWheelPower = 0.31;

    // positions of the beacons' servo
    double leftBeaconRetract = 0;
    double leftBeaconExtend = 1;
    double rightBeaconRetract = 1;
    double rightBeaconExtend = 0;

    // Trigger for light sensor
    double whiteLightTrigger = 0.3;

    // Shooting distance from wall
    double shootingDistance = 50;

    // Speeds

    double highPower = .75;
    double highGain = 0.01;

    double midPower = 0.5;
    double midGain = 0.01;

    double lowPower = 0.3;
    double lowGain = 0.05;

    double allPower = 1;
    double allPowerGain = 0.01;

    //We want to push the beacon when our robot is 10 cm away from the wall
    double pushBeaconDistance = 8;

    public static double endGyro;

    Beacon.BeaconAnalysis beaconAnalysis;
    double imageConfidence;

    Object[][] sequenceArray = new Object[][]{
            // STEP 1. START
            {stateMachine.start},

            //STEP 2. Add a delay in seconds if needed
            {stateMachine.timeDelay, 0},

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
            {stateMachine.slideState, -45, allPower, 2, -1900.0, 0.0, allPowerGain},



            //STEP 5. Slide at 0 degrees until -820 encoder counts at orientation 0.0
            {stateMachine.slideState, 0, highPower, 2, -1624.0, 0.0, highGain},



            //STEP 6. Shoot
            {stateMachine.triggerGate, shooterGateOpenTime},
            {stateMachine.timeDelay, 2}, // If you want to wait before starting, start here
            {stateMachine.triggerGate, shooterGateOpenTime},
            //STEP 7. Stop shooter
            {stateMachine.shooterWheel, 0.0},

            //STEP 8. Push Cap ball
            {stateMachine.slideState, 30, highPower, 2, -2000.0, 0.0, highGain},

            //STEP 9. Move back to shooting position
            {stateMachine.slideState, 210, highPower, 1, 2000.0, 0.0, highGain},
            //STOP for testing
            {stateMachine.stop},

            //STEP 8. Slide at 0 degrees until -3300 encoder counts to push cap ball
            {stateMachine.slideState, -90, allPower, 2, -3300.0, 0.0, allPowerGain},

            //STEP 9. Stop program, move to final state
            {stateMachine.stop},
    };

    {
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.parseDouble(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    public int average4Motors(int a, int b, int c, int d) {
        return (int) (a + b + c + d) / 4;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        data = new DataLogger(day.toString() + " autonomous data");

        //Wait for vision to initialize - this should be the first thing you do
        waitForVisionStart();
        telemetry.addData("Init", "vision started");
        telemetry.update();

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
        enableExtension(VisionOpMode.Extensions.BEACON);         //Beacon detection
        enableExtension(VisionOpMode.Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(VisionOpMode.Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

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

        leftBeacon.setPosition(leftBeaconRetract);
        rightBeacon.setPosition(rightBeaconRetract);
        shooterGate.setPosition(shooterGateClosed);

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
        shooter.setMaxSpeed(6000);

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

        waitForStart();

        while(opModeIsActive()){
            switch ((stateMachine) sequenceArray[seqCounter][0]) {
                case start:
                    seqCounter++;
                    timer.reset();
                    break;

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

                case getColor:
                    beaconAnalysis = beacon.getAnalysis();
                    rightOrLeft = beaconAnalysis.getColorString();
                    imageConfidence = beaconAnalysis.getConfidence();

                    if(rightOrLeft.equals("???, ???")|| imageConfidence <.05){
                        telemetry.addData("color", rightOrLeft);
                        telemetry.addData("Confidence", imageConfidence);

                        // If didn't get color or got with low confidence,
                        // keep checking for 1 second. If more than 1s has elapsed
                        // and haven't been able to determine colors, then skip # of states
                        // passed as argument.
                        if(timer.seconds() > 1){
                            rightOrLeft = "???, ???";
                            telemetry.addData("skip color", "");
                            seqCounter += (int) sequenceArray[seqCounter][1];
                            noOfBeaconsPressed++;
                            timer.reset();
                            break;
                        }
                    }
                    else{
                        noOfBeaconsPressed++;
                        telemetry.addData("goodImage ",rightOrLeft);
                        telemetry.addData("goodImage ", imageConfidence);
                        timer.reset();
                        drive.resetEncoders();
                        seqCounter++;
                    }
                    break;

                case pushBeacon:
                    // push for 1. retract for any other value.
                    telemetry.addData("color", rightOrLeft);
                    if ((int) sequenceArray[seqCounter][1] == 1) {
                        if (rightOrLeft.equals("red, blue")) {
                            // Since this is blue auto, we have to press the blue.
                            leftBeacon.setPosition(leftBeaconExtend); // extend left beacon
                            rightBeacon.setPosition(rightBeaconRetract); // retract right beacon because it is inverted
                            // just for safety
                        } else if(rightOrLeft.equals("blue, red")) {
                            rightBeacon.setPosition(rightBeaconExtend); // extend right beacon
                            leftBeacon.setPosition(leftBeaconRetract); // retract left beacon. for safety
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

                case slideState:
                    //if (drive.slideAngle(((Integer) sequenceArray[counter][1]).doubleValue(), (double) sequenceArray[counter][2], ((Boolean) sequenceArray[counter][3]).booleanValue(), (double) sequenceArray[counter][4], (double) sequenceArray[counter][5]) == 1) {
                    switch ((int)sequenceArray[seqCounter][3]) {
                        case 1: //goes while encoder count is less than desired. next parameter is the desired encoder count.
                            double encoderAverage = average4Motors(rf.getCurrentPosition(), rb.getCurrentPosition(), lf.getCurrentPosition(), lb.getCurrentPosition());
                            if(drive.slideAngleIMU((int) sequenceArray[seqCounter][1], (double) sequenceArray[seqCounter][2], encoderAverage < (double)sequenceArray[seqCounter][4], (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                                telemetry.addData("state", encoderAverage);

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
                                data.addField(rf.getCurrentPosition());
                                data.addField(rb.getCurrentPosition());
                                data.addField(lf.getCurrentPosition());
                                data.addField(lb.getCurrentPosition());
                                data.newLine();
                            }
                            else{
                                timer.reset();
                                drive.resetEncoders();
                                seqCounter ++;
                            }
                            break;
                        case 3: //goes while US level is greater than desired
                            if(drive.slideAngleIMU(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], ultrasonic.cmUltrasonic() > (double)sequenceArray[seqCounter][4] , (double) sequenceArray[seqCounter][5], (double) sequenceArray[seqCounter][6]) == 1){
                                telemetry.addData("state", ultrasonic.cmUltrasonic());
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
                        case 7: //goes while US level is greater than desired only if the
                            //red blue is detected
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

                case stop:
                    drive.setPowerAll(0, 0, 0, 0);
                    timer.reset();
                    telemetry.addData("state", "stop");
                    telemetry.addData("US level", ultrasonic.cmUltrasonic());
                    telemetry.addData("color", rightOrLeft);
                    break;

                case shooterWheel:
                    shooter.setPower((double) sequenceArray[seqCounter][1]);
                    seqCounter++;
                    timer.reset();
                    break;

                case triggerGate:
                    // Shoot first particle by opening gate.
                    if(timer.seconds() < shooterGateOpenTime)
                        shooterGate.setPosition(shooterGateOpen);
                    else {
                        shooterGate.setPosition(shooterGateClosed);
                        seqCounter++;
                        timer.reset();
                    }
                    break;

                case pivotRobot:
                    // pivot to angle for a certain time
                    if (timer.seconds()<(double) sequenceArray[seqCounter][3]){
                        drive.pivotToAngleIMU((double) sequenceArray[seqCounter][1], (double) sequenceArray[seqCounter][2], true, (double) sequenceArray[seqCounter][4],(double) sequenceArray[seqCounter][5]);
                    } else
                    {
                        timer.reset();
                        drive.resetEncoders();
                        seqCounter++;
                    }
                    break;
            }
            telemetry.update();
        }

    }
    @Override
    public void stop(){
        super.stop();
        endGyro = -imu.getAngularOrientation().firstAngle;
    }

}

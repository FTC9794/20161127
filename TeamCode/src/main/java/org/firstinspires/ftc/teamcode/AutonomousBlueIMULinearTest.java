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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import java.io.File;

/**
 * Created by Ishaan Oberoi on 12/1/2016.
 */
@Autonomous(group = "auto", name = "linear Auto Blue test")
public class AutonomousBlueIMULinearTest extends LinearVisionOpMode {
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
        slideState, timeDelay, testTelemetry, grip, pivotCapBall, getColor, goToBeacon2Line, transition, retractServos, start, estop, slide45Distance, slide90Ultrasonic, slide0light, waitBeforeGoingBack, slide180Light, pivotTo0Time, slideToBeacon, pushBeacon, checkColor, waitForBeaconPusher, retractBeaconPusher, slideNeg45ToBeacon2, slide30in, testGetToPosition, slide45Cap, pivotToNeg45, slide20US, capBall, stop
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
            {stateMachine.timeDelay, 2},
            {stateMachine.slideState, 45, 1.0, 1, 341.953451, 0.0, 0.05},
            {stateMachine.stop, 90, .75, 3, 47.0, 0.0, 0.05},
            {stateMachine.slideState, 0, 0.25, 6, 0.15, 0.0, 0.005},
            {stateMachine.getColor},
            {stateMachine.slideState, 100, 0.5, 3, 15.0, 0.0, 0.005},
            {stateMachine.slideState, 0, 0.15, 6, 0.15, 0.0, 0.005},
            {stateMachine.slideState, 180, 0.15, 6, 0.15, 0.0, 0.005},
            {stateMachine.slideState, 90, 0.17, 3, .0, 0.0, 0.05},
            {stateMachine.grip, 1.0, 1.5},
            {stateMachine.pushBeacon, 0, 1.0},
            {stateMachine.grip, -1.0, 1.5},
            {stateMachine.slideState, 315, 0.45, 4, 43.0, 0.0, 0.007},
            {stateMachine.slideState, 0, 0.35, 6, 0.15, 0.0, 0.008},
            {stateMachine.getColor},
            {stateMachine.slideState, 100, 0.5, 3, 15.0, 0.0, 0.005},
            {stateMachine.slideState, 0, 0.15, 6, 0.15, 0.0, 0.005},
            {stateMachine.slideState, 180, 0.15, 6, 0.17, 0.0, 0.005},
            {stateMachine.slideState, 90, 0.22, 3, 8.0, 0.0, 0.05},
            {stateMachine.grip, 1.0, 1.5},
            {stateMachine.pushBeacon, 0, 1.0},
            {stateMachine.grip, -1.0, 1.5},
            {stateMachine.stop}

    };
    public int average2Motors(int a, int b) {
        return (int) (a + b) / 2;
    }

    public int average4Motors(int a, int b, int c, int d) {
        return (int) (a + b + c + d) / 4;
    }
    @Override
    public void runOpMode() throws InterruptedException {
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
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

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
        light = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "light");

        telemetry.addData("Init", "hardware done");
        telemetry.update();


        timer = new ElapsedTime();
        timer.reset();
        firstDistance *= 127.5;
        state = stateMachine.start;

        beaconPresssed = 0;
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
        timer.reset();
        while(opModeIsActive()){
            if(drive.slideAngleIMU(45, .5, timer.seconds()<1, 0.0, 0.05)==1){

            }else{
                drive.setPowerAll(0, 0, 0, 0);
            }
            telemetry.update();
        }
    }
}

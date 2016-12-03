package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.DataLogger;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.opencv.core.Size;

import java.io.File;

/**
 * Created by Ishaan Oberoi on 11/26/2016.
 */
//@Autonomous(name = "test auto methods")
public class AutoBlueNewDrive extends OpMode {
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

    int counter = 0;
    @Override
    public void init() {
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



        timer = new ElapsedTime();
        timer.reset();
        firstDistance *= 127.5;
        state = stateMachine.start;
        drive = new MecanumDrive(rf, rb, lf, lb, imu, light, ultrasonic, null, telemetry, timer);

        beaconPresssed = 0;
        seqCounter = 0;
    }
    @Override
    public void init_loop(){
        if (counter == 0){
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
            counter++;
        }else{
            telemetry.addData("Done with init", "press play");
        }
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
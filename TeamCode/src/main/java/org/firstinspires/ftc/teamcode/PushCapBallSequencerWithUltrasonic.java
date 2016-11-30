package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ishaan Oberoi on 11/26/2016.
 */
@Autonomous(name = "Autonomous 2a")
public class PushCapBallSequencerWithUltrasonic extends OpMode {
    DcMotor lf, lb, rf, rb;
    ElapsedTime timer;
    //states in sequencer
    enum stateMachine {
        slideStateNoGyro, slideState, timeDelay, testTelemetry, grip, pivotCapBall, /*getColor,*/ goToBeacon2Line, transition, retractServos, start, estop, delay, slide45Distance, slide90Ultrasonic, slide0light, waitBeforeGoingBack, slide180Light, pivotTo0Time, slideToBeacon, pushBeacon, checkColor, waitForBeaconPusher, retractBeaconPusher, slideNeg45ToBeacon2, slide30in, testGetToPosition, slide45Cap, pivotToNeg45, slide20US, capBall, stop
    };
    stateMachine state;
    Servo rightBeacon;
    Servo leftBeacon;
    Servo shooterGate;
    MecanumDrive drive;
    ModernRoboticsI2cRangeSensor ultrasonic;
    int seqCounter = 0;
    Object[][] sequenceArray = new Object[][]{
            {stateMachine.start},
            {stateMachine.slideStateNoGyro, 0, 1.0, 7, 2.0},
            {stateMachine.stop}
    };
    @Override
    public void init() {
        //call motors from hardware map
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        leftBeacon = hardwareMap.servo.get("left_beacon");
        rightBeacon = hardwareMap.servo.get("right_beacon");
        shooterGate = hardwareMap.servo.get("shooter_gate");
        //gripper = hardwareMap.crservo.get("beacon_brace");
        //make it so when the powers are set to 0, the motors will stop and not move
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBeacon.setPosition(0);
        rightBeacon.setPosition(1);
        shooterGate.setPosition(.5);
        ultrasonic = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonic");

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
       // light = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "light");

        timer = new ElapsedTime();
        state = stateMachine.start;
        drive = new MecanumDrive(rf, rb, lf, lb, null, null, null, null, telemetry, null);
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
/*
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
*/
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
                        break;/*
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
/*
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
                        }*/
                }
                break;

            case slideStateNoGyro:
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
                        break;/*
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
                        }*/
                    case 7:
                        if(drive.slide(((Integer) sequenceArray[seqCounter][1]).doubleValue(), (double) sequenceArray[seqCounter][2], timer.seconds()<(double)sequenceArray[seqCounter][4]) == 1){
                            telemetry.addData("state", "slide encoder ");
                        }
                        else{
                            timer.reset();
                            drive.resetEncoders();
                            seqCounter ++;
                        }
                        break;
                }
                break;
            case stop:
                drive.setPowerAll(0, 0, 0, 0);
                telemetry.addData("state", "stop");
        }
    }
}
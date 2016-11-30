package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ishaan Oberoi on 11/29/2016.
 */
@Autonomous(group = "mecanum", name = "autonomous 1")
public class PushCapBall extends OpMode {
    DcMotor lf, lb, rf, rb;
    ElapsedTime timer;
    MecanumDrive drive;
    enum stateMachine{
        start, move, stop
    }
    stateMachine state;
    @Override
    public void init() {
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        timer = new ElapsedTime();
        drive = new MecanumDrive(rf, rb, lf, lb, null, null, null, null, telemetry, null);
        state = stateMachine.start;
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
    }

    @Override
    public void loop() {
        switch(state){
            case start:
                timer.reset();
                state = stateMachine.move;
                break;
            case move:
                if(drive.slide(0, 1, timer.seconds()<2)==1){
                    telemetry.addData("state: ", "move");
                }else{
                    state = stateMachine.stop;
                }
                break;
            case stop:
                drive.setPowerAll(0, 0, 0, 0);
                telemetry.addData("state: ", "stop");
                break;
        }
    }
}

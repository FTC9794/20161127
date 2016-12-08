package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.AutonomousBlueIMULinear;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.io.File;

/**
 * Created by Ishaan Oberoi on 11/29/2016.
 */
//@TeleOp(group = "Mecanum Drive", name = "TeleOp 3")
public class SuperMecIMU extends OpMode {
    DcMotor rf;
    DcMotor rb;
    DcMotor lf;
    DcMotor lb;
    DcMotor shooter;
    DcMotor sweeper;
    Servo leftBeacon;
    Servo rightBeacon;
    Servo shooterGate;
    MecanumDrive drive;
    ElapsedTime beaconTimer;
    ElapsedTime shooterTime;
    ElapsedTime shooterToggleTimer;
    ElapsedTime harvestToggleTimer;
    double speed;
    double angle;
    double IMUAngle;
    double pivotSpeed;
    BNO055IMU imu;
    Orientation q;
    boolean shooterToggle;
    boolean harvestToggle;
    double offset;
    @Override
    public void init() {
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        shooter = hardwareMap.dcMotor.get("shooter");
        sweeper = hardwareMap.dcMotor.get("sweeper");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper.setDirection(DcMotorSimple.Direction.REVERSE);

        drive = new MecanumDrive(rf, rb, lf, lb, imu, null, null, null, telemetry, null);

        leftBeacon = hardwareMap.servo.get("left_beacon");
        rightBeacon = hardwareMap.servo.get("right_beacon");
        shooterGate = hardwareMap.servo.get("shooter_gate");

        leftBeacon.setPosition(0);
        rightBeacon.setPosition(1);
        shooterGate.setPosition(.5);

        shooterToggle = false;
        harvestToggle = false;

        shooterTime = new ElapsedTime();
        beaconTimer = new ElapsedTime();
        shooterToggleTimer = new ElapsedTime();        harvestToggleTimer = new ElapsedTime();

        // We are expecting the IMU to be attached to an I2C port on a Core Device Interface Module and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Get the calibration data
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

        // Save the calibration data to a file. You can choose whatever file
        // name you wish here, but you'll want to indicate the same file name
        // when you initialize the IMU in an opmode in which it is used. If you
        // have more than one IMU on your robot, you'll of course want to use
        // different configuration file names for each.
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        offset = AutonomousBlueIMULinear.endGyro;

    }

    @Override
    public void loop() {
        angle = drive.joystickToAngle(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        speed = drive.returnRadius(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        q = imu.getAngularOrientation();
        IMUAngle = q.firstAngle;
        angle += IMUAngle + offset;
        pivotSpeed = -gamepad1.right_stick_x;
        telemetry.addData("Angle: ", angle);
        telemetry.addData("Speed: ", speed);
        telemetry.addData("pivot speed: ", pivotSpeed);
        if(gamepad1.left_trigger>.1){
            speed*=.5;
        }
        drive.pivotSlide(angle, speed, true, pivotSpeed);
        /*
        if(gamepad1.b){
            beaconTimer.reset();
            leftBeacon.setPosition(0);
            rightBeacon.setPosition(0);
        }
        if(beaconTimer.seconds()>1){
            leftBeacon.setPosition(1);
            rightBeacon.setPosition(1);
        }*/
        if(shooterToggleTimer.seconds()>1){
            if(gamepad2.b){
                if(shooterToggle){
                    shooter.setPower(0);
                    shooterToggle = false;
                }else{
                    shooter.setPower(.4);
                    shooterToggle = true;
                }
                shooterToggleTimer.reset();
            }

        }

        if(gamepad2.x){
            shooterGate.setPosition(0);
            shooterTime.reset();
        }
        if(shooterTime.milliseconds()>75){
            shooterGate.setPosition(.5);
        }

        if(harvestToggleTimer.seconds()>.5){
            if(gamepad2.y){
                if(harvestToggle){
                    sweeper.setPower(0);
                    harvestToggle = false;

                }else{
                    sweeper.setPower(1);
                    harvestToggle = true;
                }
                harvestToggleTimer.reset();
            }

        }
        if(gamepad2.a){
            sweeper.setPower(-1);
        }

    }
}

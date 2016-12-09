package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.io.File;

/**
 * Created by Ishaan Oberoi on 12/1/2016.
 */

@TeleOp(group = "Mecanum Drive", name = "TeleOp Red")
@Disabled
public class LinearTeleOp3Red extends LinearOpMode{
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
    double shootingSpeed = .325;
    @Override
    public void runOpMode() throws InterruptedException {
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        shooter = hardwareMap.dcMotor.get("shooter");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMaxSpeed(6000);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeper.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("init", "hardware done");
        telemetry.update();
        drive = new MecanumDrive(rf, rb, lf, lb, imu, null, null, null, telemetry, null);
        telemetry.addData("init", "Drivetrain init");
        telemetry.update();
        leftBeacon = hardwareMap.servo.get("left_beacon");
        rightBeacon = hardwareMap.servo.get("right_beacon");
        shooterGate = hardwareMap.servo.get("shooter_gate");

        leftBeacon.setPosition(0.02);
        rightBeacon.setPosition(.98);
        shooterGate.setPosition(.5);

        shooterToggle = false;
        harvestToggle = false;

        shooterTime = new ElapsedTime();
        shooterToggleTimer = new ElapsedTime();
        harvestToggleTimer = new ElapsedTime();
        telemetry.addData("init", "IMU starting");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
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
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        offset = AutonomousRedIMULinear.endGyro;
        telemetry.addData("init", "done");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                drive.pivotToAngleIMU(90 + offset, .005, true, .5, .1);
            } else if (gamepad1.dpad_down) {
                drive.pivotToAngleIMU(270 + offset, .005, true, .5, .1);
            } else if (gamepad1.dpad_left) {
                drive.pivotToAngleIMU(0 + offset, .005, true, .5, .1);
            } else if (gamepad1.dpad_right) {
                drive.pivotToAngleIMU(180 + offset, .005, true, .5, .1);
            } else {
                angle = drive.joystickToAngle(gamepad1.right_stick_x, -gamepad1.right_stick_y);
                speed = drive.returnRadius(gamepad1.right_stick_x, -gamepad1.right_stick_y);
                q = imu.getAngularOrientation();
                IMUAngle = q.firstAngle;
                angle += IMUAngle + offset;
                pivotSpeed = -gamepad1.left_stick_x;
                telemetry.addData("offset", offset);
                telemetry.addData("Angle: ", angle);
                telemetry.addData("Speed: ", speed);
                telemetry.addData("pivot speed: ", pivotSpeed);
                telemetry.addData("gamepad1 left x", gamepad1.right_stick_x);
                telemetry.addData("gamepad1 left y", gamepad1.right_stick_y);

                if (gamepad1.left_trigger > .1) {
                    speed *= .5;
                }
                drive.pivotSlide(angle, speed, true, pivotSpeed);
            }

            if (shooterToggleTimer.seconds() > 1) {
                if (gamepad2.b) {
                    if (shooterToggle) {
                        shooter.setPower(0);
                        shooterToggle = false;
                    } else {
                        shooter.setPower(shootingSpeed);
                        shooterToggle = true;
                    }
                    shooterToggleTimer.reset();
                }

            }

            if (gamepad2.x) {
                shooterGate.setPosition(0);
                shooterTime.reset();
            }
            if (shooterTime.milliseconds() > 75) {
                shooterGate.setPosition(.5);
            }

            if (harvestToggleTimer.seconds() > .5) {
                if (gamepad2.y) {
                    if (harvestToggle) {
                        sweeper.setPower(0);
                        harvestToggle = false;

                    } else {
                        sweeper.setPower(1);
                        harvestToggle = true;
                    }
                    harvestToggleTimer.reset();
                }

            }
            if (gamepad2.a) {
                sweeper.setPower(-1);
            }

            telemetry.update();
        }
    }
}

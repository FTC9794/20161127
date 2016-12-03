package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;

/**
 * Created by Ishaan Oberoi on 11/27/2016.
 */
//@Autonomous(name = "state machine test", group = "simple auto")
public class TestAutoStateMachine extends OpMode {
    DcMotor rf, rb, lf, lb;
    enum StateMachine {
        start, move, moveback, stop
    }
    StateMachine state;
    ElapsedTime timer;
    ModernRoboticsI2cColorSensor color;
    BNO055IMU imu;
    @Override
    public void init() {
        color = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color");
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        state = StateMachine.start;
        timer = new ElapsedTime();
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
    }

    @Override
    public void loop() {
        switch(state) {
            case start:
                timer.reset();
                state = StateMachine.move;
                break;
            case move:
                if (timer.seconds() < 1) {
                    rf.setPower(1);
                    rb.setPower(1);
                    lf.setPower(1);
                    lb.setPower(1);
                } else {
                    timer.reset();
                    state = StateMachine.moveback;
                }

                break;
            case moveback:
                if (timer.seconds() < 1) {
                    rf.setPower(-1);
                    rb.setPower(-1);
                    lf.setPower(-1);
                    lb.setPower(-1);
                } else {
                    state = StateMachine.stop;
                }
                break;
            case stop:
                rf.setPower(0);
                rb.setPower(0);
                lf.setPower(0);
                lb.setPower(0);
        }
    }
}

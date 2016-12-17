package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Created by Ishaan Oberoi on 10/19/2016.
 */

public class MecanumDrive {

    private DcMotor rf;
    private DcMotor rb;
    private DcMotor lf;
    private DcMotor lb;
    private ModernRoboticsI2cCompassSensor compass;
    private ModernRoboticsAnalogOpticalDistanceSensor light;
    private ModernRoboticsI2cRangeSensor ultrasonic;
    private ModernRoboticsI2cGyro gyro;
    private Telemetry telemetry;
    private ElapsedTime pivotTime;
    private ElapsedTime moveTime;

    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    //create variables
    private double currentAngle;
    private double newSpeed;
    private double angleDifference;
    private double wallReading;
    private double vertical;
    private double horizontal;
    private double angleChange;
    boolean notVisible = true;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables ftc;
    private VuforiaTrackable gears;
    private VuforiaTrackable tools;
    private VuforiaTrackable wheels;
    private VuforiaTrackable legos;
    private List<VuforiaTrackable> allTrackables;
    private VuforiaLocalizer.Parameters parameters;
    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    private double currentX;
    private double currentY;
    private double vuforiaAngle;
    private double xDifference;
    private double yDifference;
    private double pivotdifference;
    private double pitch;
    private double roll;
    private double yaw;


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    /*
    Inputs:         The four motors
                    The compass sensor
                    The ODS sensor
                    Ultrasonic sensor
                    Gyro sensor
                    Telemetry
                    pivotTime: Object of ElapsedTime used for checking the time elapsed. Used in the pivotAngle method.
    Outputs:        None
    Description:    Constructor for Mecanum drive object
     */
    public MecanumDrive(DcMotor rf, DcMotor rb, DcMotor lf, DcMotor lb, BNO055IMU imu, ModernRoboticsAnalogOpticalDistanceSensor light, ModernRoboticsI2cRangeSensor ultrasonic, ModernRoboticsI2cGyro gyro, Telemetry telemetry, ElapsedTime pivotTime) {
        this.rf = rf;
        this.rb = rb;
        this.lf = lf;
        this.lb = lb;
        this.imu = imu;
        this.light = light;
        this.ultrasonic = ultrasonic;
        this.gyro = gyro;
        this.telemetry = telemetry;
        this.pivotTime = pivotTime;

    }

    /*
    Inputs:         Angle: Clockwise is positive; it can go negative
                    Gain: How much the robot will change the power per degree as it approaches the desired angle (between 0 and 1)
                    Time: How long the robot will take to get to the desired angle
                    Max Speed: Double between 0 and 1
                    Min Speed: Double between 0 and 1
                    Active: Boolean expression of whether the op mode is active or not
    Outputs:        Return of Active - 0 or 1 - which determines whether the op mode is done or not
    Description:    This method pivots the robot to a desired angle. Because of the gain, the robot speed changes.
                    Max speed and speed limit the range of speed. The power used is the gain times the difference
                    of the current angle vs the desired (0-doesn't move, 1- moves at full force)
     */
    public int pivotToAngle(double angle, double gain, double time, double maxSpeed, double minSpeed) {
        if (gyroCheck() == 0) {
            setPowerAll(0, 0, 0, 0);
        }
        //set speeds until time is up
        if (pivotTime.time() < time) {
            //measure the gyro sensor
            //get gyro sensor value
            currentAngle = gyro.getIntegratedZValue();

            //determine the new speed
            angleDifference = currentAngle + angle;
            newSpeed = -angleDifference * gain;

            //set limits
            if (newSpeed > maxSpeed) {
                newSpeed = maxSpeed;
            } else if (newSpeed < -maxSpeed) {
                newSpeed = -maxSpeed;
            }
            if (newSpeed < minSpeed && newSpeed > 0) {
                newSpeed = minSpeed;
            } else if (newSpeed > -minSpeed && newSpeed < 0) {
                newSpeed = -minSpeed;
            }

            //send back data about what the robot is doing

            setPowerAll(newSpeed, newSpeed, -newSpeed, -newSpeed);
            telemetry.addData("gyro angle", gyro.getIntegratedZValue());
            telemetry.addData("angle diff", angleDifference);
            return 1;
        } else {
            return 0;
        }
    }
    public int pivotToAngleIMU(double desiredAngle, double gain, boolean endCase, double maxSpeed, double minSpeed) {
        //set speeds until time is up
        if (endCase) {
            //measure the gyro sensor
            //get gyro sensor value
            Orientation o = imu.getAngularOrientation();
            double currentAngle = -o.firstAngle;
            double difference = desiredAngle - currentAngle;
            if(difference>180){
                difference-=360;
            }else if(difference<-180){
                difference+=360;
            }

            double magSpeed = Math.abs(difference * gain);
            telemetry.addData("Mag speed", magSpeed);
            if(magSpeed > maxSpeed){
                magSpeed = maxSpeed;
            }
            if(magSpeed<minSpeed){
                magSpeed = minSpeed;
            }
            if(difference<0){
                rf.setPower(magSpeed);
                rb.setPower(magSpeed);
                lf.setPower(-magSpeed);
                lb.setPower(-magSpeed);
            }else{
                rf.setPower(-magSpeed);
                rb.setPower(-magSpeed);
                lf.setPower(magSpeed);
                lb.setPower(magSpeed);
            }
/*
            //determine the new speed
            angleDifference = currentAngle - angle;
            newSpeed = -angleDifference * gain;
            //set limits
            if (newSpeed > maxSpeed) {
                newSpeed = maxSpeed;
            } else if (newSpeed < -maxSpeed) {
                newSpeed = -maxSpeed;
            }
            if (newSpeed < minSpeed && newSpeed > 0) {
                newSpeed = minSpeed;
            } else if (newSpeed > -minSpeed && newSpeed < 0) {
                newSpeed = -minSpeed;
            }
*/
            //send back data about what the robot is doing


            telemetry.addData("Angle", currentAngle);
            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("angle difference", currentAngle-desiredAngle);
            return 1;
        } else {
            return 0;
        }
    }
/*
    public int pivotToAngleIMU(double desiredAngle, double gain, boolean endCase, double maxSpeed, double minSpeed) {
        if (endCase) {
            //measure the gyro sensor
            //get gyro sensor value
            Orientation o = imu.getAngularOrientation();
            double currentAngle = -o.firstAngle;
            double difference = desiredAngle - currentAngle;
            double magSpeed = 0.0;

            double turnDirection = 1.0; // 1 is clockwise; -1 is counterclockwise

            if (difference >0) {
                // Desired angle is greater than current, turn clockwise
                turnDirection = 1.0;
            }

            // if you need to turn more than 180 degrees, turn the opposite way
            // set the magnitude of the speed to be proportional to the difference
            // if you are more than 180 degrees away, you need to subtract the difference from 360
            if (Math.abs(difference)>180){
                turnDirection = -turnDirection;
                magSpeed = (360-(Math.abs(difference)) * gain) ;
            } else {
                magSpeed = Math.abs(difference) * gain;
            }


            // never go above the max speed or below min speed

            if(magSpeed > maxSpeed){
                magSpeed = maxSpeed;
            }
            if(magSpeed<minSpeed){
                magSpeed = minSpeed;
            }

            rf.setPower(magSpeed*turnDirection);
            rb.setPower(magSpeed*turnDirection);
            lf.setPower(-magSpeed*turnDirection);
            lb.setPower(-magSpeed*turnDirection);


            //send back data about what the robot is doing


            telemetry.addData("Angle", currentAngle);
            telemetry.addData("Mag speed", magSpeed);
            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("angle difference", currentAngle-desiredAngle);
            return 1;
        } else {
            return 0;
        }
    }
*/
    /*
    Inputs:         Speeds: Four doubles from -1 to 1 representing the speed of each motor (RF, RB, LF, LB)
                    Desired Light: The amount of reflected light the robot needs to follow (0 to 1)
                    Gain: How much the robot will change the power per .01 light intensity off the desired light (0 to 1)
                    leftOrRight: String "left" or "right" determining whether you follow the left or right side of the line
                    endCase: Is 0 when the robot will stop line following
    Outputs:        Return of endCase: 0 makes the robot stop following the line
    Description:    This method follows a line
     */
    public int followLine(double rfSpeed, double rbSpeed, double lfSpeed, double lbSpeed, double desiredLight, double gain, String leftOrRight, boolean endCase) {
        if (endCase) {
            angleChange = (light.getLightDetected() - desiredLight) * gain * 100;
            if (leftOrRight.equals("left")) {
                rf.setPower(rfSpeed + angleChange);
                rb.setPower(rbSpeed + angleChange);
                lf.setPower(lfSpeed - angleChange);
                lb.setPower(lbSpeed - angleChange);
            } else {
                rf.setPower(rfSpeed - angleChange);
                rb.setPower(rbSpeed - angleChange);
                lf.setPower(lfSpeed + angleChange);
                lb.setPower(lbSpeed + angleChange);
            }
            return 1;
        } else {
            setPowerAll(0, 0, 0, 0);
            return 0;
        }
    }

    /*
    Inputs:         Speeds: Four doubles from -1 to 1 representing the speed of each motor (RF, RB, LF, LB)
                    Light: How much light you would like your robot to see (int between 0 - dark and 256 - white light)
                    Sentinel which controls the execution of the loop (0 or 1)
    Outputs:        Return 0 makes the robot stop and 1 keeps it going
    Description:    This method will move the robot until a certain light value has been detected
     */
    public int goTillLight(double rfSpeed, double rbSpeed, double lfSpeed, double lbSpeed, int light) {
        if (this.light.getLightDetected() < light) {
            setPowerAll(rfSpeed, rbSpeed, lfSpeed, lbSpeed);
            return 1;
        } else {
            setPowerAll(0, 0, 0, 0);
            return 0;
        }
    }

    /*
    Inputs:         Sentinel which controls the execution of the loop (0 or 1)
    Outputs:        Returns 0 if gyro is unplugged or 1 if it is working
    Description:    Checks if gyro is working
     */
    public int gyroCheck() {
        int value = gyro.getHeading();
        if (value == 361) {
            return 0;
        } else {
            return 1;
        }
    }

    /*
    Inputs:         Four doubles from -1 to 1 representing the speed of each motor (RF, RB, LF, LB)
    Outputs:        none
    Description:    Sets power to all of the motors
     */
    public void setPowerAll(double rfSpeed, double rbSpeed, double lfSpeed, double lbSpeed) {
        rf.setPower(rfSpeed);
        rb.setPower(rbSpeed);
        lf.setPower(lfSpeed);
        lb.setPower(lbSpeed);
    }

    /*
    Inputs:         Four doubles from -1 to 1 representing the speed of each motor (RF, RB, LF, LB)
                    Desired Distance: Int Distance from the wall in CM read by the ultrasonic sensors
    Outputs:        Returns 1 to keep moving, or 0 to stop
    Description:    This method sets power to all of the motors until ultrasonic sensor sees a certain value
                    If no wall is visible to the ultrasonic sensor, its reading is 256
     */
    public int getToDistance(double rfSpeed, double rbSpeed, double lfSpeed, double lbSpeed, int desiredDistance) {

        //get the average sensor value
        wallReading = ultrasonic.cmUltrasonic();


        if (wallReading <= 5) {
            //bad ultrasonic reading
            setPowerAll(0, 0, 0, 0);
            return 0;
        } else {

            //move the distance
            if (desiredDistance < ultrasonic.cmUltrasonic()) {
                setPowerAll(rfSpeed, rbSpeed, lfSpeed, lbSpeed);
                return 1;
            } else if (desiredDistance > ultrasonic.cmUltrasonic()) {
                setPowerAll(-rfSpeed, -rbSpeed, -lfSpeed, -lbSpeed);
                return 1;
            } else {
                setPowerAll(0, 0, 0, 0);
                return 0;
            }
        }

    }

    /*
    Inputs:         Four doubles from -1 to 1 representing the speed of each motor (RF, RB, LF, LB)
                    Time: Desired time in seconds you want to move
    Outputs:        Returns 1 to keep moving, or 0 to stop
    Description:    This method sets power to all of the motors until ultrasonic sensor sees a certain value
                    This method moves the robot for time
     */
    public int moveForTime(double rfSpeed, double rbSpeed, double lfSpeed, double lbSpeed, double time) {
        if (moveTime.seconds() < time) {
            setPowerAll(rfSpeed, rbSpeed, lfSpeed, lbSpeed);
            return 1;
        } else {
            setPowerAll(rfSpeed, rbSpeed, lfSpeed, lbSpeed);
            return 0;
        }
    }

    /*
    Inputs:         Angle: Double for Desired Angle at which to slide the robot (in the frame of reference of the robot)
                    Speed: Power to apply to all the motors, double between -1 and 1
                    Boolean Condition:  any case for which the robot does the action (case being sensor values etc.)
    Outputs:        Returns 1 to keep moving, or 0 to stop
    Description: This method slides the robot at a certain angle at a certain speed



    The parameters are angle which is a double in the frame of reference of the robot
    The robot will slide at the angle
    Speed is a double between -1 and 1
    Condition is a boolean which is when the method will return 0 and stop doing anything

     */
    public int slide(double angle, double speed, boolean condition) {

        //return the new X and Y values using the angle needed and the speed the robot was
        //traveling at
        horizontal = round2D(calculateX(angle, speed));
        vertical = round2D(calculateY(angle, speed));
        telemetry.addData("vertical", vertical);
        //determine the powers using the new X and Y values and the other joystick to pivot
        if (condition) {
            rf.setPower((vertical - horizontal) * .5);
            rb.setPower((vertical + horizontal) * .5);
            lf.setPower((vertical + horizontal) * .5);
            lb.setPower((vertical - horizontal) * .5);
            return 1;
        } else {
            setPowerAll(0, 0, 0, 0);
            return 0;
        }

    }

    /*
    Pivot slide slides the robot at an angle while allowing it to pivot a certain amount
    The parameters are angle which is a double in the frame of reference of the robot
    speed which is a double between -1 and 1
    the condition is a boolean value which is when the program will stop and return 0
    the pivotAmount is a double between -1 and 1 which is the speed at which it will pivot
    Difference between this method and slideAngle is you can set the power you are pivoting at
    for smooth turns
    Method mostly for teleop
     */
    public double averageEncoder(){
        int motors = 4;
        DcMotor[] motorList = {rf, rb, lf, lb};
        double total = 0;
        for(DcMotor motor:motorList){
            if(motor.getPower()==0){
                motors--;
            }else{
                total += motor.getCurrentPosition()/motor.getPower();
            }
        }
        return total/motors;
    }
    public int pivotSlide(double angle, double speed, boolean condition, double pivotAmount) {

        //return the new X and Y values using the angle needed and the speed the robot was
        //traveling at
        horizontal = round2D(calculateX(angle, speed));
        vertical = round2D(calculateY(angle, speed));

        //determine the powers using the new X and Y values and the other joystick to pivot
        if (condition) {
            rf.setPower((vertical - horizontal) + pivotAmount * .5);
            rb.setPower((vertical + horizontal) + pivotAmount * .5);
            lf.setPower((vertical + horizontal) - pivotAmount * .5);
            lb.setPower((vertical - horizontal) - pivotAmount * .5);
            return 1;
        } else {
            setPowerAll(0, 0, 0, 0);
            return 0;
        }

    }

    /*
    Slide angle is a method that makes the robot slide at an angle while keeping the same pivot angle so the robot will always face the same direction
    The parameters are slideDirection in the frame of reference of the robot
    The speed as a double between -1 and 1
    the condition as a boolean which is when the program will end and return 0 when calling this command you may want to use conditions such as
        Insert Examples
    the orientation which is a double that is the direction the robot will face in the frame of reference of the gyro calibration position
    the oGain which is a double between 0 and 1 which is how much gain for proportional feedback to correct for orientation
     */
    public int slideAngle(double slideDirection, double speed, boolean condition, double orientation, double oGain) {
        // get the horizonal and vertical components of the robot speeds
        // the horizonal and vertical are used to set the power of the motor

        horizontal = round2D(calculateX(slideDirection, speed));
        vertical = round2D(calculateY(slideDirection, speed));

        // Get the gyro angle (WHAT ARE THE VALID RANGES?)
        int gyroAngle = gyro.getIntegratedZValue();

        // Correct for the orientation of the robot relative to the desired direction
        // orientation is the desired angle want to slide
        // correction is the error times the gain.

        double pivotCorrection = -((orientation - gyroAngle) * oGain);

        if (condition) {
            rf.setPower(((vertical - horizontal) - pivotCorrection) * .5);
            rb.setPower(((vertical + horizontal) - pivotCorrection) * .5);
            lf.setPower(((vertical + horizontal) + pivotCorrection) * .5);
            lb.setPower(((vertical - horizontal) + pivotCorrection) * .5);
            return 1;
        } else {
            setPowerAll(0, 0, 0, 0);
            return 0;
        }
    }




    double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.parseDouble(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public int slideAngleIMU(double slideDirection, double speed, boolean condition, double orientation, double oGain) {

        // get the horizonal and vertical components of the robot speeds
        // the horizonal and vertical are used to set the power of the motor
        horizontal = round2D(calculateX(slideDirection, speed));
        vertical = round2D(calculateY(slideDirection, speed));

        Orientation o = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double ox = formatAngle(o.angleUnit, o.firstAngle);
        double gyroAngle = ox;

        // What is the range of the gyroAngle?
        double pivotCorrection = -((orientation - gyroAngle) * oGain);
        //determine the powers using the new X and Y values and the other joystick to pivot
        if (condition) {
            rf.setPower(((vertical - horizontal) - pivotCorrection) * .5);
            rb.setPower(((vertical + horizontal) - pivotCorrection) * .5);
            lf.setPower(((vertical + horizontal) + pivotCorrection) * .5);
            lb.setPower(((vertical - horizontal) + pivotCorrection) * .5);
            telemetry.addData("gyro angle", gyroAngle);
            return 1;
        } else {
            setPowerAll(0, 0, 0, 0);
            return 0;
        }
    }




    // returns how far the joystick is pushed in distance from the center
    // inputs are the X value and Y joystick value
    public double returnRadius(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    // takes the X and Y value of the joystick and determines
    // the angle that the joystick is at relative to the center of the joystcik
    // returns in degrees
    public double joystickToAngle(double x, double y) {
        return Math.atan2(x, y) * (180 / Math.PI);
    }

    //returns X vector value using angle and speed
    public double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    //returns the Y vector value using angle and speed
    public double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    //rounds the input to 2 decimal places
    public double round2D(double input) {
        input *= 100;
        input = Math.round(input);
        return input / 100;
    }

    public void resetEncoders(){
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

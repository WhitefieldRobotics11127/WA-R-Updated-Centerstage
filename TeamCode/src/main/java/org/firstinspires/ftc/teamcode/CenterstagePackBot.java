/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Whitefield Robotics - FTC 2023-2024 Centerstage
 * This code defines the hardware on our robot.
 */

public class CenterstagePackBot {

    public WebcamName webcamName = null;

    /* Public OpMode members. */
    public DcMotor dcMotor1 = null;
    public DcMotor dcMotor2 = null;
    public DcMotor dcMotor3 = null;
    public DcMotor dcMotor4 = null; //four drive motors
    //NEW
    public DcMotor dcMotor5 = null; //intake roller motor
    public DcMotor dcMotor6 = null; //slide/string movement
    public DcMotor dcMotor7 = null; //bucket articulation 1
    //public DcMotor dcMotor8 = null; //bucket articulation 2

    //PAST public DcMotor dcMotor5 = null; //slide articulation
    //PAST public DcMotor dcMotor6 = null; //string movement
    //PAST public DcMotor dcMotor7 = null; //grabber motor

    public VoltageSensor vs;

    // public RevBlinkinLedDriver blinkin = null;

    public DistanceSensor frontDistSensor, leftDistSensor, rightDistSensor;

    //Not deleted to demonstrate how color sensors work
    //public NormalizedColorSensor colorSensor1;

    public DigitalChannel led1R;
    public DigitalChannel led1G;
    public DigitalChannel led2R;
    public DigitalChannel led2G;
    public DigitalChannel led3R;
    public DigitalChannel led3G;
    public DigitalChannel led4R;
    public DigitalChannel led4G;

    public RevBlinkinLedDriver blinkinLedDriver;

    public DigitalChannel touchSensor;

//    public AnalogInput pot = null;
//    public double potLowerVoltage = 0.607, potUpperVoltage = 1.575;
//    private double targetPos = potLowerVoltage;
//    private double wgCoeff = 0.3; // 1/3.3; 3.3 is the max voltage of the potentiometer

//not deleted to demonstrate CR Servos
//    public CRServo intakeServoBR = null;

    //NEW
    public Servo leftBucket = null;
    public Servo rightBucket = null;
    //PAST public Servo leftClaw = null;
    //PAST public Servo rightClaw = null;
    //public Servo launcher = null;

    public BNO055IMU imu;
    Orientation angles; //not sure if we need this
    Acceleration gravity; //not sure if we need this


    //We should be able to just change the wheel diameter to switch out physical wheels
    //Odometry things
    public static final double     COUNTS_PER_MOTOR_REV    = 8192 ;
    public static final double     WHEEL_DIAMETER_INCHES   = 2.75 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     COUNTS_PER_LIFT_INCH         = (537.7) / (1.75 * 3.1415);

//NEW
    public static final double leftBucketClosed = .19;
    public static final double leftBucketOpen = .52;
    public static final double rightBucketOpen = .35;
    public static final double rightBucketClosed = 0.83;


    //PAST
    /*
    public static final double leftClawOpen = .15;
    public static final double leftClawClosed = .42;
    public static final double rightClawClosed = .35;
    public static final double rightClawOpen = 0.57;

     */



    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();



    /* TODO: Make Constructor Params include the bot, opmode, and hwmap and remove such params from the methods
     "opmode" param may be able to be "this", idk abt hwmap, but i think it's in here somewhere
     already. */
    /* Constructor */
    public CenterstagePackBot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;


        RevBlinkinLedDriver.BlinkinPattern pattern;
        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        // Define and Initialize Motors
        dcMotor1 = hwMap.get(DcMotor.class, "motor_1");
        dcMotor2 = hwMap.get(DcMotor.class, "motor_2");
        dcMotor3 = hwMap.get(DcMotor.class, "motor_3");
        dcMotor4 = hwMap.get(DcMotor.class, "motor_4");
        //NEW
        dcMotor5 = hwMap.get(DcMotor.class, "motor_intake");
        dcMotor6 = hwMap.get(DcMotor.class, "motor_slide");
        dcMotor7 = hwMap.get(DcMotor.class, "motor_bucket");
        //dcMotor8 = hwMap.get(DcMotor.class, "motor_bucket_2");

        //PAST dcMotor5 = hwMap.get(DcMotor.class, "motor_articulate");
        //PAST dcMotor6 = hwMap.get(DcMotor.class, "motor_string");
        //PAST dcMotor7 = hwMap.get(DcMotor.class, "motor_grabber");

        // This is what lets us be an omnidirectional bot, changed from previous years to allow us
        // to use vertical motors

        dcMotor1.setDirection(DcMotor.Direction.REVERSE);
        dcMotor6.setDirection(DcMotor.Direction.REVERSE);
        //dcMotor2.setDirection(DcMotor.Direction.REVERSE);
        //dcMotor3.setDirection(DcMotor.Direction.REVERSE);
        //dcMotor4.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        dcMotor1.setPower(0);
        dcMotor2.setPower(0);
        dcMotor3.setPower(0);
        dcMotor4.setPower(0);
        dcMotor5.setPower(0);
        dcMotor6.setPower(0);
        dcMotor7.setPower(0);

//        pot = hwMap.get(AnalogInput.class, "pot");

        //Set motors to run with/without encoders
        //Motors 1, 2 are left-front, right-front
        //Motor 3 is horizontal

        dcMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //NEW
        //dcMotor8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        dcMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor6.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor7.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //NEW
        //dcMotor8.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize installed servos.
        //NEW
        leftBucket = hwMap.get(Servo.class, "left_bucket");
        rightBucket = hwMap.get(Servo.class, "right_bucket");

        //PAST leftClaw = hwMap.get(Servo.class, "leftClaw");
        //PAST rightClaw = hwMap.get(Servo.class, "rightClaw");
        //launcher = hwMap.get(Servo.class, "launcher");

        // blinkin = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        //colorSensor1 = hwMap.get(NormalizedColorSensor.class, "colorSensor1");

        /*
        led1R = hwMap.get(DigitalChannel.class, "l1r");
        led1G = hwMap.get(DigitalChannel.class, "l1g");
        led2R = hwMap.get(DigitalChannel.class, "l2r");
        led2G = hwMap.get(DigitalChannel.class, "l2g");
        led3R = hwMap.get(DigitalChannel.class, "l3r");
        led3G = hwMap.get(DigitalChannel.class, "l3g");
        led4R = hwMap.get(DigitalChannel.class, "l4r");
        led4G = hwMap.get(DigitalChannel.class, "l4g");
        */

        /*
        led1R.setMode(DigitalChannel.Mode.OUTPUT);
        led1G.setMode(DigitalChannel.Mode.OUTPUT);
        led2R.setMode(DigitalChannel.Mode.OUTPUT);
        led2G.setMode(DigitalChannel.Mode.OUTPUT);
        led3R.setMode(DigitalChannel.Mode.OUTPUT);
        led3G.setMode(DigitalChannel.Mode.OUTPUT);
        led4R.setMode(DigitalChannel.Mode.OUTPUT);
        led4G.setMode(DigitalChannel.Mode.OUTPUT);
         */

//        not deleted for demonstrative purposes
//        intakeServoFL = hwMap.get(CRServo.class, "intakeServoFL");

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //may not need this
        gravity = imu.getGravity(); //may not need this

        webcamName = hwMap.get(WebcamName.class, "Webcam 1");
    }

    public boolean closeTo(double val, double target, double error) {
        return (val >= target - error && val < target + error);
    }

    public double getDistance(String sensor) {
        switch (sensor) {
            case "Front":
                return frontDistSensor.getDistance(DistanceUnit.INCH);
            case "Left":
                return leftDistSensor.getDistance(DistanceUnit.INCH);
            case "Right":
                return rightDistSensor.getDistance(DistanceUnit.INCH);
            default:
                System.out.println("JD probably wrote this code");
                return 0;
        }
    }

    /*
    public int getLiftPos() {
        return dcMotor7.getCurrentPosition();
    }
    */

    //NEW
    public void openLeftBucket() {leftBucket.setPosition(leftBucketOpen);}
    //PAST public void openLeftClaw(){ leftClaw.setPosition(leftClawOpen);}

    //NEW
    public void openRightBucket() {rightBucket.setPosition(rightBucketOpen);}
    //PAST public void openRightClaw(){ rightClaw.setPosition(rightClawOpen); }

    //NEW
    public void closeBucket(LinearOpMode opMode){
        leftBucket.setPosition(leftBucketClosed);
        opMode.sleep(150);
        rightBucket.setPosition(rightBucketClosed);
    }

    public void openBucket(LinearOpMode opMode){
        leftBucket.setPosition(leftBucketOpen);
        opMode.sleep(150);
        rightBucket.setPosition(rightBucketOpen);
    }

    //PAST
    /*
    public void openClaw(LinearOpMode opMode){
        leftClaw.setPosition(leftClawOpen);
        opMode.sleep(300);
        rightClaw.setPosition(rightClawOpen);
    }
    //PAST
    /*
    public void closeClaw(LinearOpMode opMode){
        leftClaw.setPosition(leftClawClosed);
        opMode.sleep(300);
        rightClaw.setPosition(rightClawClosed);
    }

     */

    public void moveEntireBucketOut(LinearOpMode opMode, double targetCt, double speed){
        int posCurrent = (dcMotor7.getCurrentPosition());
        while (!opMode.isStopRequested() && posCurrent < targetCt){
            dcMotor7.setPower(speed);
            posCurrent = (dcMotor7.getCurrentPosition());
        }
        dcMotor7.setPower(0);
    }

    public void moveEntireBucketIn(LinearOpMode opMode, double targetCt, double speed){
        int posCurrent = (dcMotor7.getCurrentPosition());
        while (!opMode.isStopRequested() && posCurrent > targetCt){
            dcMotor7.setPower(-speed);
            posCurrent = (dcMotor7.getCurrentPosition());
        }
        dcMotor7.setPower(0);
    }

    //NEW
    public void moveBucketOut(LinearOpMode opMode, double targetCt, double speed){
        int posCurrent = (dcMotor7.getCurrentPosition());
        while (!opMode.isStopRequested() && posCurrent < targetCt){
            dcMotor7.setPower(speed);
            posCurrent = (dcMotor7.getCurrentPosition());
        }
        dcMotor7.setPower(0);
    }

    public void moveBucketIn(LinearOpMode opMode, double targetCt, double speed){
        int posCurrent = (dcMotor7.getCurrentPosition());
        while (!opMode.isStopRequested() && posCurrent > targetCt){
            dcMotor7.setPower(-speed);
            posCurrent = (dcMotor7.getCurrentPosition());
        }
        dcMotor7.setPower(0);
    }

    //PAST
    /*
    public void moveGrabberOut(LinearOpMode opMode, double targetCt, double speed){
        int posCurrent = (dcMotor7.getCurrentPosition());
        while (!opMode.isStopRequested() && posCurrent < targetCt){
            dcMotor7.setPower(speed);
            posCurrent = (dcMotor7.getCurrentPosition());
        }
        dcMotor7.setPower(0);
    }

    public void moveGrabberIn(LinearOpMode opMode, double targetCt, double speed){
        int posCurrent = (dcMotor7.getCurrentPosition());
        while (!opMode.isStopRequested() && posCurrent > targetCt){
            dcMotor7.setPower(-speed);
            posCurrent = (dcMotor7.getCurrentPosition());
        }
        dcMotor7.setPower(0);
    }
     */

    public void moveLiftUp(LinearOpMode opMode, double targetCt, double speed) {
        int posCurrent = -(dcMotor6.getCurrentPosition());
        // Converts the vertical distance to diagonal distance using trig.
//        double targetCt = inches * COUNTS_PER_LIFT_INCH /.982; //Math is incorrect. Counts are easy.
        while (!opMode.isStopRequested() && posCurrent < targetCt) {
            dcMotor6.setPower(speed);
            posCurrent = -(dcMotor6.getCurrentPosition());
            opMode.telemetry.addData("Lift motor encoder count", -dcMotor6.getCurrentPosition());
            opMode.telemetry.update();
        }
        dcMotor6.setPower(0);
    }


    public void moveLiftDown(LinearOpMode opMode, double targetCt, double speed) {
        int posCurrent = -(dcMotor6.getCurrentPosition());
        // Converts the vertical distance to diagonal distance using trig.
//        double targetCt = inches * COUNTS_PER_LIFT_INCH /.982; //see above comment
        while (!opMode.isStopRequested() && posCurrent > targetCt) {
            dcMotor6.setPower(-speed);
            posCurrent = -(dcMotor6.getCurrentPosition());
        }
        dcMotor6.setPower(0);
    }
    
    public void driveToDist(LinearOpMode opMode, String sensor, double inchDist, double driveSpeed) {
        double targetHeading = getHeading();
        double correctionCoefficient = 0.06;
        double deltaHeading = getHeading() - targetHeading;
        switch (sensor) {
            case "Front":
                while (getDistance(sensor) < inchDist) {
                    driveWithCorrection("Backward", deltaHeading, correctionCoefficient, driveSpeed);
                    deltaHeading = getHeading() - targetHeading;
//                    advancedEncoderDrive(opMode, 1, "Backward", driveSpeed);
                }
                driveStop();
                while (getDistance(sensor) > inchDist) {
                    driveWithCorrection("Forward", deltaHeading, correctionCoefficient, driveSpeed);
                    deltaHeading = getHeading() - targetHeading;
//                    advancedEncoderDrive(opMode, 1, "Forward", driveSpeed);
                }
                driveStop();
                break;
            case "Left":
                while (getDistance(sensor) < inchDist) {
                    driveWithCorrection("Right", -deltaHeading, correctionCoefficient, driveSpeed);
                    deltaHeading = getHeading() - targetHeading;
//                    advancedEncoderDrive(opMode, 1, "Right", driveSpeed);
                }
                driveStop();
                while (getDistance(sensor) > inchDist) {
                    driveWithCorrection("Left", -deltaHeading, correctionCoefficient, driveSpeed);
                    deltaHeading = getHeading() - targetHeading;
//                    advancedEncoderDrive(opMode, 1, "Left", driveSpeed);
                }
                driveStop();
                break;
            case "Right":
                while (getDistance(sensor) < inchDist) {
                    driveWithCorrection("Left", -deltaHeading, correctionCoefficient, driveSpeed);
                    deltaHeading = getHeading() - targetHeading;
//                    advancedEncoderDrive(opMode, 1, "Left", driveSpeed);
                }
                driveStop();
                while (getDistance(sensor) > inchDist) {
                    driveWithCorrection("Right", -deltaHeading, correctionCoefficient, driveSpeed);
                    deltaHeading = getHeading() - targetHeading;
//                    advancedEncoderDrive(opMode, 1, "Right", driveSpeed);
                }
                driveStop();
                break;
            default:
                System.out.println("JD probably wrote this code");
                break;
        }
    }
    
    /** Averages and then returns the two straight odometers */
    public double getStraightEncoderCount() {
        return (dcMotor1.getCurrentPosition() + dcMotor2.getCurrentPosition()) / 2;
    }

    /** Returns  the two straight odometers*/
    public double getHorizontalEncoderCount() {
        return -dcMotor3.getCurrentPosition();
    }

    /** This may change from year to year, which is why its a method */
    public double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /** @param inchDist must be in inches
     * @param dir possible directions: "Forward", "Backward", "Left", "Right" */
    public void encoderDrive(LinearOpMode opMode, double inchDist, String dir, double driveSpeed) {
        //add rampSpeed in instead of driveSpeed
        double targetCt = inchDist * COUNTS_PER_INCH *.9;

        double initalStraightCt = getStraightEncoderCount();
        double initailStrafeCt = getHorizontalEncoderCount();

        double currStraightCt = getStraightEncoderCount() - initalStraightCt;
        double currStrafeCt = getHorizontalEncoderCount() - initailStrafeCt;

        if (dir.equals("Forward")) {
            while (!opMode.isStopRequested() && currStraightCt < targetCt) {
                driveForward(driveSpeed);
                currStraightCt = getStraightEncoderCount() - initalStraightCt;
//                opMode.telemetry.addData("TargetCt", targetCt);
//                opMode.telemetry.addData("Current Straight Ct", currStraightCt);
//                opMode.telemetry.update();
            }
            driveStop();
        } else if (dir.equals(("Backward"))) {
            while (!opMode.isStopRequested() && currStraightCt > -targetCt) {
                driveBackward(driveSpeed);
                currStraightCt = getStraightEncoderCount() - initalStraightCt;
            }
            driveStop();
        } else if (dir.equals(("Left"))) {
            while (!opMode.isStopRequested() && currStrafeCt < targetCt) {
                strafeLeft(driveSpeed);
                currStrafeCt = getHorizontalEncoderCount() - initailStrafeCt;

            }
            driveStop();
        } else if (dir.equals(("Right"))) {
            while (!opMode.isStopRequested() && currStrafeCt > -targetCt) {
                strafeRight(driveSpeed);
                currStrafeCt = getHorizontalEncoderCount() - initailStrafeCt;

            }
            driveStop();
        }
    }


    /** A more advanced version of encoderDrive, with heading hold to stay straight when driving.
     * @param distance must be in inches
     * @param direction possible directions: "Forward", "Backward", "Left", "Right"
     */
    public void advancedEncoderDrive(LinearOpMode opMode, double distance, String direction, double speed) {
        double targetCt = distance * COUNTS_PER_INCH * (-.242 * speed + .905);
        //double targetCt = distance * COUNTS_PER_INCH * (-.23 * speed + .471); // * (-.647 * speed + 1.048);
        /*
            y = mx + b where m = -.647 and b = 1.048
            To recalibrate:
            Comment out the part in parentheses. Run the bot at 30% speed (x = 0.3) to a certain distance and record the drift (in inches),
            then run the bot at 70% speed (x = 0.7) to the same distance and record the drift (in
            inches). Your y value for each will be the target distance divided by the actual distance
            it went. (actually I'm not sure about that last sentence)
            Power is x and Drift is y (in inches). Use these to find the slope and y-intercept.
        */



        double targetHeading = getHeading();
        double currentHeading = getHeading();
        double deltaHeading = currentHeading - targetHeading; // If I could replace `delta` with `∆`, I would.

        double initialStraightCt = getStraightEncoderCount();
        double initialStrafeCt = getHorizontalEncoderCount();

        double currStraightCt = getStraightEncoderCount() - initialStraightCt;
        double currStrafeCt = getHorizontalEncoderCount() - initialStrafeCt;

        double correctionCoefficient = 0.06;
        double strafeCoefficient = 0.06;

        if (direction.equals("Forward")) {
            while (!opMode.isStopRequested() && currStraightCt < targetCt) {
                driveWithCorrection(direction, deltaHeading, correctionCoefficient, speed); //While not at target, drive
//                opMode.telemetry.addData("TargetCt", targetCt);
//                opMode.telemetry.addData("Current Straight Ct", currStraightCt);
//                opMode.telemetry.update();
                //refresh variables
                currentHeading = getHeading();
                opMode.telemetry.addData("Straight Encoder Count", getStraightEncoderCount());
                opMode.telemetry.update();
                // Changed from: deltaHeading = currentHeading - targetHeading; to line below
                deltaHeading = targetHeading - currentHeading;
                currStraightCt = getStraightEncoderCount() - initialStraightCt;
                currStrafeCt = getHorizontalEncoderCount() - initialStrafeCt;
            }
            driveStop();
        } else if (direction.equals("Backward")) {
            while (!opMode.isStopRequested() && currStraightCt > -targetCt) {
                driveWithCorrection(direction, deltaHeading, correctionCoefficient, speed); //While not at target, drive

                //refresh variables
                opMode.telemetry.addData("Straight Encoder Count", getStraightEncoderCount());
                opMode.telemetry.update();
                currentHeading = getHeading();
                //Changed from: deltaHeading = currentHeading - targetHeading; to line below
                deltaHeading = targetHeading - currentHeading;
                currStraightCt = getStraightEncoderCount() - initialStraightCt;
                currStrafeCt = getHorizontalEncoderCount() - initialStrafeCt;
            }
            driveStop();
        } else if (direction.equals("Left")) {
            correctionCoefficient = strafeCoefficient;
            while (!opMode.isStopRequested() && currStrafeCt < targetCt) { // TODO: Check to see if `targetCt` still needs to be negative
                driveWithCorrection(direction, -deltaHeading, correctionCoefficient, speed); //While not at target, drive

                //refresh variables
                currentHeading = getHeading();
                //Changed from: deltaHeading = currentHeading - targetHeading; to line below
                deltaHeading = targetHeading - currentHeading;
                currStraightCt = getStraightEncoderCount() - initialStraightCt;
                currStrafeCt = getHorizontalEncoderCount() - initialStrafeCt;
                opMode.telemetry.addData("Horizontal Encoder Count", getHorizontalEncoderCount());
                opMode.telemetry.update();
            }
            driveStop();
        } else if (direction.equals("Right")) {
            correctionCoefficient = strafeCoefficient;
            while (!opMode.isStopRequested() && currStrafeCt > -targetCt) { // TODO: Check to see if `targetCt` still needs to be negative
                driveWithCorrection(direction, -deltaHeading, correctionCoefficient, speed); //While not at target, drive

                //refresh variables
                currentHeading = getHeading();
                //Changed from: deltaHeading = currentHeading - targetHeading; to line below
                deltaHeading = targetHeading - currentHeading;
                currStraightCt = getStraightEncoderCount() - initialStraightCt;
                currStrafeCt = getHorizontalEncoderCount() - initialStrafeCt;
                opMode.telemetry.addData("Horizontal Encoder Count", getHorizontalEncoderCount());
                opMode.telemetry.update();
            }
            driveStop();
        }
    }

    /** Helper method for `advancedEncoderDrive` */
    private void driveWithCorrection(String dir, double headingDelta, double correctionCoefficient, double driveSpeed) {

        //if veering left, set leftCorrect to ∆heading * (the coeff), otherwise, no left correction, set to 0
        double leftCorrect = (headingDelta > 0) ? (Math.abs(headingDelta) * correctionCoefficient) : 0;
        //if veering right, set rightCorrect to ∆heading * (the coeff), otherwise, no right correction, set to 0
        double rightCorrect = (headingDelta < 0) ? (Math.abs(headingDelta) * correctionCoefficient) : 0;

        // TODO: Check signs on all cases, may need to reverse strafe direction
        switch (dir) {
            case "Forward":  // motors 1/3 are the left, 2/4 are right
                driveSpeed *= -1; //set all motors to go the correct direction
                dcMotor1.setPower(driveSpeed - leftCorrect);
                dcMotor2.setPower(driveSpeed - rightCorrect);
                dcMotor3.setPower(driveSpeed - leftCorrect);
                dcMotor4.setPower(driveSpeed - rightCorrect);
                break;
            case "Backward": // motors 2/4 are the left, 1/3 are right
                dcMotor1.setPower(driveSpeed + rightCorrect);
                dcMotor2.setPower(driveSpeed + leftCorrect);
                dcMotor3.setPower(driveSpeed + rightCorrect);
                dcMotor4.setPower(driveSpeed + leftCorrect);
                break;
            case "Left": // motors 3/4 are left, 1/2 are right
                dcMotor1.setPower(driveSpeed + leftCorrect);
                dcMotor2.setPower(-driveSpeed - leftCorrect);
                dcMotor3.setPower(-driveSpeed - rightCorrect);
                dcMotor4.setPower(driveSpeed + rightCorrect);
                break;
            case "Right": //motors 1/2 are left, 3/4 are right
                dcMotor1.setPower(-driveSpeed - rightCorrect);
                dcMotor2.setPower(driveSpeed + rightCorrect);
                dcMotor3.setPower(driveSpeed + leftCorrect);
                dcMotor4.setPower(-driveSpeed - leftCorrect);
                break;
        }
    }

    public void driveForward(double driveSpeed) {
        dcMotor1.setPower(-driveSpeed);
        dcMotor2.setPower(-driveSpeed);
        dcMotor3.setPower(-driveSpeed);
        dcMotor4.setPower(-driveSpeed);
    }

    public void driveStop() {
        dcMotor1.setPower(0);
        dcMotor2.setPower(0);
        dcMotor3.setPower(0);
        dcMotor4.setPower(0);
    }

    public void driveBackward(double driveSpeed) {
        dcMotor1.setPower(driveSpeed);
        dcMotor2.setPower(driveSpeed);
        dcMotor3.setPower(driveSpeed);
        dcMotor4.setPower(driveSpeed);
    }

    public void strafeRight(double driveSpeed) {
        dcMotor1.setPower(-driveSpeed);
        dcMotor2.setPower(driveSpeed);
        dcMotor3.setPower(driveSpeed);
        dcMotor4.setPower(-driveSpeed);
    }

    public void strafeLeft(double driveSpeed) {
        dcMotor1.setPower(driveSpeed);
        dcMotor2.setPower(-driveSpeed);
        dcMotor3.setPower(-driveSpeed);
        dcMotor4.setPower(driveSpeed);
    }

    public void powerStrafeRight(double driveSpeed, double pwr) {
        dcMotor1.setPower(driveSpeed * 1.75 * pwr);
        dcMotor2.setPower(-driveSpeed * 1.75 * pwr);
        dcMotor3.setPower(-driveSpeed * 1.75);
        dcMotor4.setPower(driveSpeed * 1.75);

    }

    public void powerStrafeLeft(double driveSpeed, double pwr) {
        dcMotor1.setPower(-driveSpeed * 1.75 * pwr);
        dcMotor2.setPower(driveSpeed * 1.75 * pwr);
        dcMotor3.setPower(driveSpeed * 1.75);
        dcMotor4.setPower(-driveSpeed * 1.75);

    }

    public void rotateCCW(double driveSpeed) {
        dcMotor1.setPower(-driveSpeed);
        dcMotor2.setPower(driveSpeed);
        dcMotor3.setPower(-driveSpeed);
        dcMotor4.setPower(driveSpeed);

    }
    public void rotateCW(double driveSpeed) {
        dcMotor1.setPower(driveSpeed);
        dcMotor2.setPower(-driveSpeed);
        dcMotor3.setPower(driveSpeed);
        dcMotor4.setPower(-driveSpeed);

    }

}
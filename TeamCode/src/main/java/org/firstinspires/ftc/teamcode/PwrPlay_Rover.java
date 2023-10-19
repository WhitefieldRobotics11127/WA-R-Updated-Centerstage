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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

//import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;


/**
 Whitefield Robotics Power Play TeleOp Code
 */

@TeleOp(name="PwrPlay_Rover", group="Packbot")
//@Disabled
public class PwrPlay_Rover extends OpMode {

    /* Declare OpMode members. */
    PowerPlayPackBot robot = new PowerPlayPackBot();

    //variables
    double heading;

    double forward, rotate, strafe, direction = 1; //direction: 1 is normal, -1 is reversed
    double gear = .5;

    double currStrafeCt, currStraightCt;

    double front_left, front_right, rear_left, rear_right;

//    double wg_left = robot.wg_left_closed, wg_right = robot.wg_right_closed;

    double intake_motor;
    double conveyor_motor;

    boolean sprinting = false;

    double rotisseriePos = PowerPlayPackBot.rotisserieOpen;
    double chickenPos = PowerPlayPackBot.rotisserieClosed;

    double defLiftPwr = 0;
    double liftHoldPower = .15;
    double lifty_speed = 1;
    boolean isHolding = true;

    boolean canLift = false;

    RevBlinkinLedDriver.BlinkinPattern defPattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;

    private final static int GAMEPAD_LOCKOUT = 300;
    Deadline gamepadRateLimit;
    private int currentLiftPos;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        //robot.resetEncoders(robot);

        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        telemetry.addData(">", "Shall we play a game?");
    }

    @Override
    public void init_loop() {
    }


    @Override
    public void loop() {
//        heading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        /** GAMEPAD 1 */

        telemetry.addData("Encoder value: ", robot.dcMotor5.getCurrentPosition());
        telemetry.update();

        if (gamepad1.a)
            gear = .7;
        if (gamepad1.b)
            gear = 1;
        if (gamepad1.x)
            gear = .5;
        if (gamepad1.y)
            gear = .25;

        if (gamepadRateLimit.hasExpired() && gamepad1.left_stick_button && !sprinting) {
            sprinting = true;
            gear = 0.90;
            gamepadRateLimit.reset();
        } else if (gamepadRateLimit.hasExpired() && gamepad1.left_stick_button && sprinting) {
            sprinting = false;
            gear = 0.50;
            gamepadRateLimit.reset();
        }

        forward = gear * gamepad1.left_stick_y;
        strafe = gear * -gamepad1.left_stick_x;
        rotate = -gear * gamepad1.right_stick_x;
        //rotate = -gear * -gamepad1.right_stick_x;

        front_left = direction * forward + rotate + (direction * strafe);
        front_right = direction * forward - rotate - (direction * strafe);
        rear_left = direction * forward + rotate - (direction * strafe);
        rear_right = direction * forward - rotate + (direction * strafe);


        //thunder = struck;


        //Changes if the robot "front" is in the front or the back of the bot
        if (gamepad1.left_bumper)
            direction = -1;
        if (gamepad1.right_bumper)
            direction = 1;

        /* if (direction == -1) { // Code for direction indicator lights
            robot.led1R.setState(false);
            robot.led1G.setState(true);
            robot.led2R.setState(false);
            robot.led2G.setState(true);
            robot.led3R.setState(true);
            robot.led3G.setState(false);
            robot.led4R.setState(true);
            robot.led4G.setState(false);
        } else if (direction == 1) {
            robot.led1R.setState(true);
            robot.led1G.setState(false);
            robot.led2R.setState(true);
            robot.led2G.setState(false);
            robot.led3R.setState(false);
            robot.led3G.setState(true);
            robot.led4R.setState(false);
            robot.led4G.setState(true);
        }
        */

        /** GAMEPAD 2 */

        robot.dcMotor5.setPower(-gamepad2.left_stick_y);
        //robot.dcMotor6.setPower(gamepad2.right_stick_y);

        //Bumpers open/close the 2 sides simultaneously

        if (gamepad2.right_bumper){
            robot.rotisserie.setPosition(PowerPlayPackBot.rotisserieOpen);
            robot.chicken.setPosition(PowerPlayPackBot.chickenOpen);
        }

        if (gamepad2.left_bumper){
            robot.rotisserie.setPosition(PowerPlayPackBot.rotisserieClosed);
            robot.chicken.setPosition(PowerPlayPackBot.chickenClosed);
        }


        if (gamepad2.back){ //retract the odometer wheels
            robot.parmesan.setPosition(1);
            robot.fowl.setPosition(1);     //numbers are here for placeholders right now
        }

        if (gamepad2.start){ //set the odometer wheels back on the ground
            robot.parmesan.setPosition(0);
            robot.fowl.setPosition(0);     //numbers are here for placeholders right now
        }

        /*
        These are extra controls for the driver to have greater flexibility when
        maneuvering the grabber, or in case one side of the grabber fails for some reason:
            - x opens the left side
            - y closes the left side
            - b opens the right side
            - a closes the right side
        */

        if(gamepad2.x){
            robot.rotisserie.setPosition(PowerPlayPackBot.rotisserieOpen);
        }
        if(gamepad2.y){
            robot.rotisserie.setPosition(PowerPlayPackBot.rotisserieClosed);
        }
        if(gamepad2.b){
            robot.chicken.setPosition(PowerPlayPackBot.chickenOpen);
        }
        if(gamepad2.a){
            robot.chicken.setPosition(PowerPlayPackBot.chickenClosed);
        }

        /*

//        if (gamepad2.left_stick_button)
//            lifty_speed = .3;
//            isHolding = false;
//        if (gamepad2.right_stick_button)
//            lifty_speed = 1;

/*
        if (rotisseriePos == 0.0) {
            robot.dcMotor5.setPower(-gamepad2.right_stick_y);
            robot.dcMotor6.setPower(-gamepad2.right_stick_y);
        }
        if (rotisseriePos > 0.0)
            robot.dcMotor7.setPower(-gamepad2.left_stick_y);
        robot.spinnyHorse.setPower(gamepad2.right_stick_x);



        if (gamepad2.left_bumper) {
//            robot.rotisserie.setPosition(0);
            rotisseriePos = 0.0;
        } else if (gamepad2.y) {
//            robot.rotisserie.setPosition(0.5);
            rotisseriePos = 0.5;
        } else if (gamepad2.right_bumper){
//            robot.rotisserie.setPosition(1);
            rotisseriePos = 1.0;
        }
        robot.rotisserie.setPosition(rotisseriePos); */
 /*       if (Math.abs(gamepad2.left_trigger) > 0.85) {
            robot.flywheels(1.0);
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        } else if (Math.abs(gamepad2.left_trigger) > 0.01) {
            robot.flywheels(gamepad2.left_trigger);
        } else {
            robot.flywheels(0.0);
            robot.blinkin.setPattern(defPattern);
        }
    */
//        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
//            conveyor_motor = gamepad2.left_stick_y;
//        }

//        if (gamepad2.dpad_up)
//            targetLiftCt = FreightFrenzyPackBot.topLevelHeight;
//        else if (gamepad2.dpad_down)
//            targetLiftCt = FreightFrenzyPackBot.bottomLevelHeight;
//
//        currentLiftPos = robot.getLiftPos();
//        if (currentLiftPos > targetLiftCt*0.97 && currentLiftPos < targetLiftCt*1.03)
//            robot.dcMotor7.setPower(0.0);
//        else if (rotisseriePos > 0.0 && currentLiftPos < targetLiftCt)
//            robot.dcMotor7.setPower(0.4);
//        else if (rotisseriePos > 0.0 && currentLiftPos > targetLiftCt)
//            robot.dcMotor7.setPower(-0.2);
//        else
//            robot.dcMotor7.setPower(0.0);

        // Blinkin for later
        /*
        if (gamepad2.dpad_up)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        if (gamepad2.dpad_down)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        if (gamepad2.dpad_left)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        if (gamepad2.dpad_right)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
        */

        /** TELEMETRY */

        telemetry.addData("Sprinting", sprinting);

//        currStraightCt = robot.dcMotor1.getCurrentPosition();
//        currStraightCt /= robot.COUNTS_PER_INCH;
//        currStrafeCt = robot.dcMotor3.getCurrentPosition();
//        currStrafeCt /= robot.COUNTS_PER_INCH;
//
        // telemetry.addData("Lift Odometer", robot.dcMotor7.getCurrentPosition());
        // telemetry.addData("Lift Odometer - Vertical Inch", robot.dcMotor7.getCurrentPosition() / FreightFrenzyPackBot.COUNTS_PER_LIFT_INCH * .982);
//        telemetry.addData("Straight 2 Odometer", "" + currStraightCt + " " + robot.dcMotor2.getCurrentPosition());
//        telemetry.addData("Strafe Odometer", "" + currStrafeCt + " " + robot.dcMotor3.getCurrentPosition());
//
//        telemetry.addData("Straight Odo Method", robot.getStraightEncoderCount());
//        telemetry.addData("Horizontal Odo Method", robot.getHorizontalEncoderCount());
//
//        telemetry.addData("Heading", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle); //Usually what we call "heading"
//        telemetry.addData("IMU Angle 2", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
//        telemetry.addData("IMU Angle 3", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);

//        telemetry.addData("wg_left", wg_left);
//        telemetry.addData("wg_right", wg_right);

//        telemetry.addData("Pot Voltage", robot.pot.getVoltage());

        telemetry.update();

        //The conveyor motor has been changed to be the motor that lifts up the wobble goal
//        if (canLift) conveyor_motor = -gamepad2.left_stick_y * lifty_speed; //(gamepad2.left_stick_y * lifty_speed) - defLiftPwr;

        robot.dcMotor1.setPower(front_left);
        robot.dcMotor2.setPower(front_right);
        robot.dcMotor3.setPower(rear_left);
        robot.dcMotor4.setPower(rear_right);

//        robot.wgArmMovementHandler();


//        robot.wg_left.setPosition(wg_left);
//        robot.wg_right.setPosition(wg_right);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        }




}

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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

//import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;


/**
 Whitefield Robotics Centerstage TeleOp Code
 */

@TeleOp(name="Centerstage_Rover", group="Packbot")
//@Disabled
public class CenterstageRover extends OpMode {

    /* Declare OpMode members. */
    CenterstagePackBot robot = new CenterstagePackBot();

    //variables
    double heading;

    double forward, strafe, direction = 1; //direction: 1 is normal, -1 is reversed
    double rotate = 1;
    double gear = .5;

    double currStrafeCt, currStraightCt;

    double front_left, front_right, rear_left, rear_right;

    boolean sprinting = false;

    //double leftClawPos = CenterstagePackBot.leftClawOpen;
    //double rightClawPos = CenterstagePackBot.rightClawOpen;

    double defLiftPwr = 0;
    double liftHoldPower = .15;
    double lifty_speed = 1;
    boolean isHolding = true;

    RevBlinkinLedDriver.BlinkinPattern defPattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;

    private final static int GAMEPAD_LOCKOUT = 300;
    Deadline gamepadRateLimit;
    private int currentLiftPos;

    boolean leftClawClosed, rightClawClosed;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

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
            gear = .75;
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
        rotate = gear * gamepad1.right_stick_x;
        //rotate = -gear * -gamepad1.right_stick_x;

        front_left = direction * forward + rotate + (direction * strafe);
        front_right = direction * forward - rotate - (direction * strafe);
        rear_left = direction * forward + rotate - (direction * strafe);
        rear_right = direction * forward - rotate + (direction * strafe);

        /*
        if (gamepad1.back)
            robot.launcher.setPosition(0);
        if (gamepad1.start)
            robot.launcher.setPosition(1);

*/

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

        //PAST telemetry.addData("Grabber encoder value: ", robot.dcMotor7.getCurrentPosition());
        //PAST telemetry.update();
        telemetry.addData("Articulation encoder value: ", robot.dcMotor5.getCurrentPosition());
        telemetry.update();
        telemetry.addData("Slide encoder value: ", robot.dcMotor6.getCurrentPosition());
        telemetry.update();

        //Left stick pushes the slide up/down
        robot.dcMotor6.setPower(-gamepad2.right_stick_y);

        //Right trigger articulates the entire slide back/forth, was originally on right stick
        robot.dcMotor5.setPower(gamepad2.left_trigger);
        robot.dcMotor5.setPower(-gamepad2.right_trigger);

        //Right stick moves the grabber back and forth (not opening it)
            robot.dcMotor7.setPower(-(gamepad2.left_stick_y) * 0.3);
            if (gamepad2.dpad_down)
                robot.dcMotor7.setPower(0);
            //robot.dcMotor8.setPower(-(gamepad2.left_stick_y) * 0.3);

        if (gamepad2.dpad_left)
            robot.leftBucket.setPosition(CenterstagePackBot.leftBucketOpen);
        if (gamepad2.dpad_right)
            robot.leftBucket.setPosition(CenterstagePackBot.leftBucketClosed);
        if (gamepad2.x)
            robot.rightBucket.setPosition(CenterstagePackBot.rightBucketClosed);
        if (gamepad2.b)
            robot.rightBucket.setPosition(CenterstagePackBot.rightBucketOpen);

        if (gamepad2.a){

        }
        if (gamepad2.y){
            while (!robot.touchSensor.getState()){
                robot.dcMotor6.setPower(-0.5);
            }
            if (robot.touchSensor.getState())
                robot.dcMotor6.setPower(0);
        }


        /*
        if (gamepad2.a){
            robot.dcMotor7.setTargetPosition(-50);
            robot.dcMotor7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dcMotor7.setPower(0.25);
            while (robot.dcMotor7.isBusy())
                telemetry.update();
            robot.dcMotor7.setPower(0);

            robot.dcMotor7.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.dcMotor8.setTargetPosition(0);
        }

        if (gamepad2.y){
            robot.dcMotor7.setTargetPosition(655); //Bucket drop needs to go here
            robot.dcMotor7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dcMotor7.setPower(0.25);
            while (robot.dcMotor7.isBusy())
                telemetry.update();
            robot.dcMotor7.setPower(0);
        }
         */

        //PAST
        /*
        if (gamepad2.dpad_left)
            robot.leftClaw.setPosition(CenterstagePackBot.leftClawOpen);
        if (gamepad2.dpad_right)
            robot.leftClaw.setPosition(CenterstagePackBot.leftClawClosed);
        if (gamepad2.x)
            robot.rightClaw.setPosition(CenterstagePackBot.rightClawClosed);
        if (gamepad2.b)
            robot.rightClaw.setPosition(CenterstagePackBot.rightClawOpen);
        */

        /*
        if (gamepad2.back){ //retract the odometer wheels
            robot.parmesan.setPosition(1);
            robot.fowl.setPosition(1);     //numbers are here for placeholders right now
        }

        if (gamepad2.start){ //set the odometer wheels back on the ground
            robot.parmesan.setPosition(0);
            robot.fowl.setPosition(0);     //numbers are here for placeholders right now
        }

/*
        // Blinkin for later

        */
        if (gamepad1.dpad_up)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        if (gamepad1.dpad_down)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        if (gamepad1.dpad_left)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        if (gamepad1.dpad_right)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);

        if (gamepad2.right_bumper)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        if (gamepad2.left_bumper)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);


        /** TELEMETRY */

        telemetry.addData("Sprinting", sprinting);

//        currStraightCt = robot.dcMotor1.getCurrentPosition();
//        currStraightCt /= robot.COUNTS_PER_INCH;
//        currStrafeCt = robot.dcMotor3.getCurrentPosition();
//        currStrafeCt /= robot.COUNTS_PER_INCH;
//
//        telemetry.addData("Pot Voltage", robot.pot.getVoltage());

        telemetry.update();

        robot.dcMotor1.setPower(front_left);
        robot.dcMotor2.setPower(front_right);
        robot.dcMotor3.setPower(rear_left);
        robot.dcMotor4.setPower(rear_right);

    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        }
}
/*


 */
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

/*This is the header for a typical opmode. The type is at the front, either @TeleOp or
* @Autonomous. The name is what shows up during opmode selection on the driver hub, and the group
* is the other opmodes it shows up next to during opmode selection on the driver hub. If
* @Disabled is commented out, then the opmode will show up on the driver hub.
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
    double gear = .5; //speed

    double currStrafeCt, currStraightCt;

    double front_left, front_right, rear_left, rear_right;

    boolean sprinting = false;

    double lifty_speed = 1;

    RevBlinkinLedDriver.BlinkinPattern defPattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;

    private final static int GAMEPAD_LOCKOUT = 300;
    Deadline gamepadRateLimit;
    private int currentLiftPos;

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
        //This code won't change much over the years

        telemetry.addData("Encoder value: ", robot.dcMotor5.getCurrentPosition());
        telemetry.update();

        //Changes the speed of the robot
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

        //Drive code, doesn't need to be changed
        forward = gear * gamepad1.left_stick_y;
        strafe = gear * -gamepad1.left_stick_x;
        rotate = gear * gamepad1.right_stick_x;
        //rotate = -gear * -gamepad1.right_stick_x;

        front_left = direction * forward + rotate + (direction * strafe);
        front_right = direction * forward - rotate - (direction * strafe);
        rear_left = direction * forward + rotate - (direction * strafe);
        rear_right = direction * forward - rotate + (direction * strafe);


        if (gamepad1.start) //reset & hold
            robot.launcher.setPosition(0);
        if (gamepad1.back) //release
            robot.launcher.setPosition(1);

        robot.dcMotor7.setPower(gamepad1.right_trigger);
        robot.dcMotor7.setPower(-gamepad1.left_trigger);

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
        telemetry.addData("Slide encoder value: ", robot.dcMotor6.getCurrentPosition());
        telemetry.update();

        //Right stick pushes the slide up/down
        robot.dcMotor6.setPower(gamepad2.right_stick_y);

        //Left stick powers the intake (in and out)
        robot.dcMotor5.setPower(gamepad2.left_stick_y * 0.775);

        robot.dcMotor8.setPower(gamepad2.right_trigger * 0.4);
        robot.dcMotor8.setPower(gamepad2.left_trigger * -0.7);

        //Left stick moves the hang slide up and down
        //robot.dcMotor7.setPower(-(gamepad2.left_stick_y));
        //robot.dcMotor8.setPower(-(gamepad2.left_stick_y) * 0.3);

        //lb moves entire bucket in and out (toggle), rb opens and closes bucket (toggle)
        if (gamepad2.left_bumper){
            robot.rotisseriePlace();
        }

        if (gamepad2.right_bumper){
            robot.rotisserieReturn();
        }

        if (gamepad2.b)
            robot.openBucket();
        if (gamepad2.x){
            robot.closeBucket();
        }

        // this is a devious comment

        //When driver 2 presses y, the bucket will go into the retracted position & the slide
        //will move down based on the touch sensor (limit switch)
        /*
        if (gamepad2.y){
            robot.rotisseriePlace(); //rotisserie directions are backwards rn but I don't feel like fixing them
            while (!(robot.touchSensor.isPressed())){
                robot.dcMotor6.setPower(-0.5);
            }
            robot.dcMotor6.setPower(0);
        }

         */

        if (gamepad2.start){
            robot.retractPurpleArm(); //sets it up to hold the pixel
        }
        if (gamepad2.back){
            robot.deployPurpleArm();
        }

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
        if (gamepad2.dpad_left)
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
// This is my Comment :) //

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
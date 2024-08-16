/* Copyright (c) 2019 FIRST. All rights reserved.
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

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


/** PSUEDOCODE:
 * Scanning the spikes method/placing the purple pixel (not dependent on side):
 *  - Detect position of prop using OpenCv from starting position (no need for movement)
 *  - Rotate the correct direction and place the pixel
 *
 * Move to the backdrop (for each side):
 *  - Move to a certain spot where we can rotate, rotate to pick up the yellow pixel, rotate back
 *    and continue on
 *  - Rotate to the correct place based on prop position
 *  - Move the same way each time to the backdrop (use dist sensors and encoders)
 *
 * Place on the backdrop (for each side):
 *  - Move to the correct position based on prop position
 *  - Move slide up a certain amount
 *  - Release pixel
 *  - Retract
 *
 *  Park (for each side)
 *   - Move left or right to get out of the way of alliance partner
 *
 * All OpModes we want to write:
 *  - Scan, place purple pixel, park (blue and red side)
 *  - Scan, place purple pixel, place yellow pixel, park (blue and red side)
 *  - Scan, place yellow pixel, place purple pixel, park (blue and red side)
 *  - Scan, place purple pixel, place yellow pixel, place white pixel, park (blue and red side)
 *  - Scan, place yellow pixel, place purple pixel, place white pixel, park (blue and red side)
 */


public class CenterstageAuto {

    private LinearOpMode myOpMode;       // Access to the OpMode object
    private CenterstagePackBot myRobot;        // Access to the Robot hardware
    private HardwareMap myHardwareMap;
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor


    // Class Members
    private OpenGLMatrix location = null;

    private final int rows = 640;
    private final int cols = 480;

    //OpenCV variables
    private String position = "";
    public OpenCvWebcam webcam;
    public TSEDeterminationPipeline pipeline; //BEWARE THE PIPELINE
    public TSEDeterminationPipelineWithSide pipelineOther;
    public TSEDeterminationPipeline.TSEPosition savedAnalysis =
            TSEDeterminationPipeline.TSEPosition.LEFT;
    public TSEDeterminationPipelineWithSide.TSEPosition savedAnalysis1 =
            TSEDeterminationPipelineWithSide.TSEPosition.LEFT;

    //Encoder constants for the bucket
    final double bucketStart = 0;
    public static double bucketDeploy = 1010;
    public static double bucketDrop = 655; //bucket drop also needs to be changed in the bucket position
                                            //function in Centerstage Rover

    //Encoder constants for the entire lift. The amount of encoder counts the slide motor will rotate before
    //stopping
    final double floorHeight = 1705;
    double backboardHeight = 1695;

    //Encoder constants to extend the lift
    final double baseHeight = 0;
    double backboardLevel = 0;

    //Constructor
    public CenterstageAuto(LinearOpMode theOpMode, CenterstagePackBot theRobot, HardwareMap theHwMap) {
        myOpMode = theOpMode;
        myRobot = theRobot;
        myHardwareMap = theHwMap;
    }

    public double getHeading() {
        return myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /*
    This method is the method that just places the purple pixel, should be the same for all 4 sides.
    Make sure you keep every autonomous program within a method in the main autonomous class and then
    call that method from each OpMode class, which you will directly run from the Driver Hub.
    TODO: put this in all other autonomous methods
    */
    public void placePurplePixel(String color){
        double driveSpeed = 0.35;
        double rotateSpeed = 0.3;
        double liftSpeed = 0.3;
        int sleepTime = 300;
        String markerPos = "Left";

        if (color.equals("Red")) {
            myOpMode.telemetry.addData("Realtime analysis", scanPropCV("Red"));
            myOpMode.telemetry.update();
            markerPos = scanSavedRed();
        }
        else if (color.equals("Blue")) {
            myOpMode.telemetry.addData("Realtime analysis", scanPropCV("Blue"));
            myOpMode.telemetry.update();
            markerPos = scanSavedBlue();
        }
        myOpMode.telemetry.addData("Marker Pos", markerPos);
        myOpMode.telemetry.update();

        myRobot.retractPurpleArm();
        myOpMode.sleep(sleepTime);

        myRobot.openBucket();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 30, "Forward", driveSpeed); //drives to middle tape
        myOpMode.sleep(sleepTime);

        if (markerPos.equals("Left")){
            while (myRobot.getHeading() < 90)
                myRobot.rotateCW(rotateSpeed);
            myRobot.driveStop();
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 2, "Backward", driveSpeed);
            myOpMode.sleep(sleepTime);
            while (myRobot.getHeading() > 0)
                myRobot.rotateCCW(rotateSpeed);
            myRobot.driveStop();
        }
        else if (markerPos.equals("Middle")){
            myOpMode.sleep(sleepTime);
        }
        else {
            while (myRobot.getHeading() > -90)
                myRobot.rotateCCW(rotateSpeed);
            myRobot.driveStop();
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 2, "Backward", driveSpeed);
            myOpMode.sleep(sleepTime);
            while (myRobot.getHeading() < 0)
                myRobot.rotateCW(rotateSpeed);
            myRobot.driveStop();
        }
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 10, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);
    }

    /*
    PSEUDOCODE FOR BOT 3.0
    Place purple pixel (should work for all sides)
        - Read
        - Drive forwards
        - Rotate based on location
        - Drop the pixel
        - Rotate towards backboard (left on blue or right on red)
    Front stage side:
        - Drive left (red) or right (blue) towards center of the field (not too far to go into other side)
        - Drive forwards and either:
            - Parks and drops yellow pixel or
            - Goes to place on board and then parks in the triangle
    Backstage side:
        - Drive right (red) or left (blue) to clear the spikes
        - Drive forwards
        - Place on board
        - Park in the corner
    */

    //This places the purple pixel on the front side and parks in the triangle while dropping the yellow
    public void frontStagePurplePark(String color, String side){
        double driveSpeed = 0.35;
        double rotateSpeed = 0.3;
        double liftSpeed = 0.3;
        int sleepTime = 300;
        String markerPos = "Left";

        if (color.equals("Red")){
            myRobot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        }
        if (color.equals("Blue")){
            myRobot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        }

        placePurplePixel(color);

        myRobot.advancedEncoderDrive(myOpMode, 15, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 48, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (color.equals("Red")){
            while (myRobot.getHeading() > 90)
                myRobot.rotateCCW(rotateSpeed);
        }
        if (color.equals("Blue")){
            while (myRobot.getHeading() < 90)
                myRobot.rotateCW(rotateSpeed);
        }
        myRobot.driveStop();

        myRobot.advancedEncoderDrive(myOpMode, 72, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.rotisseriePlace();
        myOpMode.sleep(sleepTime);

        myRobot.openBucket();
        myOpMode.sleep(sleepTime);
    }

    //This places the purple pixel on the front side, places the yellow pixel, and parks in the triangle
    public void frontStagePurpleYellow(String color, String side){
        double driveSpeed = 0.35;
        double rotateSpeed = 0.3;
        double liftSpeed = 0.3;
        int sleepTime = 300;
        String markerPos = "Left";

        if (color.equals("Red")){
            myRobot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        }
        if (color.equals("Blue")){
            myRobot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        }

        //TSE detection might need to change
        if (color.equals("Red")) {
            myOpMode.telemetry.addData("Realtime analysis", scanPropCV("Red"));
            myOpMode.telemetry.update();
            markerPos = scanSavedRed();
        }
        else if (color.equals("Blue")) {
            myOpMode.telemetry.addData("Realtime analysis", scanPropCV("Blue"));
            myOpMode.telemetry.update();
            markerPos = scanSavedBlue();
        }
        myOpMode.telemetry.addData("Marker Pos", markerPos);
        myOpMode.telemetry.update();

        placePurplePixel(color);

        myRobot.advancedEncoderDrive(myOpMode, 15, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 48, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (color.equals("Red")){
            while (myRobot.getHeading() > 90)
                myRobot.rotateCCW(rotateSpeed);
        }
        if (color.equals("Blue")){
            while (myRobot.getHeading() < 90)
                myRobot.rotateCW(rotateSpeed);
        }
        myRobot.driveStop();

        myRobot.advancedEncoderDrive(myOpMode, 72, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (markerPos.equals("Left"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 10 : 40, color.equals("Red") ? "Right" : "Left", driveSpeed);
        if (markerPos.equals("Middle"))
            myRobot.advancedEncoderDrive(myOpMode, 21.5, color.equals("Red") ? "Right" : "Left", driveSpeed);
        if (markerPos.equals("Right"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 28.5 : 10, color.equals("Red") ? "Right" : "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.rotisseriePlace();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 2, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.openBucket();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 2, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (markerPos.equals("Left"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 10 : 40, color.equals("Red") ? "Left" : "Right", driveSpeed);
        else if (markerPos.equals("Middle"))
            myRobot.advancedEncoderDrive(myOpMode, 21.5, color.equals("Red") ? "Left" : "Right", driveSpeed);
        else if (markerPos.equals("Right"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 28.5 : 10, color.equals("Red") ? "Left" : "Right", driveSpeed);
        myOpMode.sleep(sleepTime);
    }

    //This places the purple pixel on the back side, places the yellow pixel, and parks in the corner
    public void backStagePurpleYellow(String color, String side){
        double driveSpeed = 0.35;
        double rotateSpeed = 0.3;
        double liftSpeed = 0.3;
        int sleepTime = 299;
        String markerPos = "Left";

        if (color.equals("Red")){
            myRobot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        }
        if (color.equals("Blue")){
            myRobot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        }

        //TSE detection might need to change
        if (color.equals("Red")) {
            myOpMode.telemetry.addData("Realtime analysis", scanPropCV("Red"));
            myOpMode.telemetry.update();
            markerPos = scanSavedRed();
        }
        else if (color.equals("Blue")) {
            myOpMode.telemetry.addData("Realtime analysis", scanPropCV("Blue"));
            myOpMode.telemetry.update();
            markerPos = scanSavedBlue();
        }
        myOpMode.telemetry.addData("Marker Pos", markerPos);
        myOpMode.telemetry.update();

        //Place purple pixel method starts here

        placePurplePixel(color);

        if (color.equals("Red")){
            while (myRobot.getHeading() > 90)
                myRobot.rotateCCW(rotateSpeed);
        }
        if (color.equals("Blue")){
            while (myRobot.getHeading() < 90)
                myRobot.rotateCW(rotateSpeed);
        }
        myRobot.driveStop();

        myRobot.dcMotor8.setPower(0.2);
        myOpMode.sleep(150);
        myRobot.dcMotor8.setPower(0);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 29, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (markerPos.equals("Left"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 40 : 10, color.equals("Red") ? "Left" : "Right", driveSpeed);
        if (markerPos.equals("Middle"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 31 : 21.5, color.equals("Red") ? "Left" : "Right", driveSpeed);
        if (markerPos.equals("Right"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 22 : 28.5, color.equals("Red") ? "Left" : "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.rotisseriePlace();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 2, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.openBucket();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 2, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (markerPos.equals("Left"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 30 : 27, color.equals("Red") ? "Right" : "Left", driveSpeed);
        else if (markerPos.equals("Middle"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 22 : 30, color.equals("Red") ? "Right" : "Left", driveSpeed);
        else if (markerPos.equals("Right"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 12 : 39, color.equals("Red") ? "Right" : "Left", driveSpeed);
        myOpMode.sleep(sleepTime);
    }

    public void stateFrontStage(){
        double driveSpeed = 0.35;
        double rotateSpeed = 0.3;
        double liftSpeed = 0.3;
        int sleepTime = 300;
        String markerPos = "Left";

        myRobot.retractPurpleArm();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 30, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 2, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);
    }

    public void stateBackStage(String color){
        double driveSpeed = 0.35;
        double rotateSpeed = 0.3;
        double liftSpeed = 0.3;
        int sleepTime = 300;
        String markerPos = "Left";

        myRobot.retractPurpleArm();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 48, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 2, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (color.equals("Red")){
            while (myRobot.getHeading() > 90)
                myRobot.rotateCCW(rotateSpeed);
        }
        if (color.equals("Blue")){
            while (myRobot.getHeading() < 90)
                myRobot.rotateCW(rotateSpeed);
        }
        myRobot.driveStop();

        myRobot.rotisserieReturn();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 40, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.openBucket();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);
    }

    //auto from bot 2.0
    public void placePurpleYellow(String color, String side) {
        //The robot must be facing intake forwards at the start
        double driveSpeed = 0.35;
        double rotateSpeed = 0.3;
        double liftSpeed = 0.3;
        int sleepTime = 300;
        String markerPos = "Left";

        if (color.equals("Red")){
            myRobot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
        }

        if (color.equals("Blue")){
            myRobot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        }

        //myRobot.closeBucket(myOpMode);
        myOpMode.sleep(sleepTime);

        //myRobot.moveEntireBucketIn(myOpMode, 100, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, 100, liftSpeed);
        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            myOpMode.telemetry.addData("Realtime analysis", scanPropCV("Red"));
            myOpMode.telemetry.update();
            markerPos = scanSavedRed();
        }
        else if (color.equals("Blue")) {
            myOpMode.telemetry.addData("Realtime analysis", scanPropCV("Blue"));
            myOpMode.telemetry.update();
            markerPos = scanSavedBlue();
        }
        myOpMode.telemetry.addData("Marker Pos", markerPos);
        myOpMode.telemetry.update();

        /*
        myRobot.advancedEncoderDrive(myOpMode, .1, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, .5, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);
        */

        myRobot.advancedEncoderDrive(myOpMode, 21.4, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftDown(myOpMode, 100, liftSpeed);
        myOpMode.sleep(sleepTime);
        //myRobot.moveEntireBucketOut(myOpMode, 100, liftSpeed);
        myOpMode.sleep(sleepTime);
        if (markerPos.equals("Left")){
            while (myRobot.getHeading() < 58)
                myRobot.rotateCW(rotateSpeed);
            myRobot.driveStop();
           // myRobot.openRightBucket(); //maybe move the bucket back up?
            myOpMode.sleep(sleepTime);
            while (myRobot.getHeading() > 5)
                myRobot.rotateCCW(rotateSpeed);
            myRobot.driveStop();
        }
        else if (markerPos.equals("Middle")) {
        //    myRobot.openRightBucket();
        }
        else { //when markerPos.equals("Right") or we can't/don't detect the prop
            while (myRobot.getHeading() > -49)
                myRobot.rotateCCW(rotateSpeed);
            myRobot.driveStop();
        //    myRobot.openRightBucket();
            myOpMode.sleep(sleepTime);
            while (myRobot.getHeading() < -2)
                myRobot.rotateCW(rotateSpeed);
            myRobot.driveStop();
            myOpMode.sleep(sleepTime);
        }
        myOpMode.sleep(sleepTime);
       // myRobot.closeBucket(myOpMode);
        myOpMode.sleep(sleepTime);
        //myRobot.moveBucketIn(myOpMode, bucketDrop, liftSpeed);
        //myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 15, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 29, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

       // myRobot.openRightBucket();
        myOpMode.sleep(sleepTime);

        /*
        if (color.equals("Red")) {
            while (getHeading() > -90)
                myRobot.rotateCCW(rotateSpeed);
        }
        if (color.equals("Blue")) {
            while (getHeading() < 91.5)
                myRobot.rotateCW(rotateSpeed);
        }

        myRobot.advancedEncoderDrive(myOpMode, 29, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (markerPos.equals("Left"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 40 : 10, color.equals("Red") ? "Left" : "Right", driveSpeed);
        if (markerPos.equals("Middle"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 31 : 21.5, color.equals("Red") ? "Left" : "Right", driveSpeed);
        if (markerPos.equals("Right"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 22 : 28.5, color.equals("Red") ? "Left" : "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveEntireLiftOut(myOpMode, backboardHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 10, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.openRightBucket();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 2, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (markerPos.equals("Left"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 30 : 27, color.equals("Red") ? "Right" : "Left", driveSpeed);
        else if (markerPos.equals("Middle"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 22 : 30, color.equals("Red") ? "Right" : "Left", driveSpeed);
        else if (markerPos.equals("Right"))
            myRobot.advancedEncoderDrive(myOpMode, color.equals("Red") ? 12 : 39, color.equals("Red") ? "Right" : "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        /*
        myRobot.moveBucketIn(myOpMode, bucketDeploy + bucketDrop, liftSpeed);
        myOpMode.sleep(sleepTime);
        myRobot.moveEntireLiftIn(myOpMode, backboardHeight, liftSpeed);
        myOpMode.sleep(sleepTime);
        */
    }

    public void initCV(String color) {
        // Documentation omitted for brevity - please see ocvWebcamExample.java for documentation
        HardwareMap hardwareMap = myHardwareMap;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = myHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myHardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(myRobot.webcamName, cameraMonitorViewId);
        if (color.equals("Red")){
            pipeline = new TSEDeterminationPipeline();
            webcam.setPipeline(pipeline);
        }
        else if (color.equals("Blue")) {
            pipelineOther = new TSEDeterminationPipelineWithSide();
            webcam.setPipeline(pipelineOther);
        }
        else{
            pipeline = new TSEDeterminationPipeline();
            webcam.setPipeline(pipeline);
        }

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                //This will be called if the camera could not be opened
            }
        });
    }

    public void ocvTelemetry() {
        myOpMode.telemetry.addData("Frame Count", webcam.getFrameCount());
        myOpMode.telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        myOpMode.telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        myOpMode.telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        myOpMode.telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        myOpMode.telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        myOpMode.telemetry.update();
    }

    public void shutdownCV() {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    //Takes in the color of our alliance and analyzes where the team prop is.
    public void snapshotCV(String color) {
        if (color.equals("Red"))
            savedAnalysis = pipeline.getAnalysis();
        else if (color.equals("Blue"))
            savedAnalysis1 = pipelineOther.getAnalysis();
        else
            savedAnalysis = pipeline.getAnalysis();
    }

    /*
    This is broken into its own method in case we want to
    save the snapshot and analyze it at different times.
     */

    public String scanSavedBlue(){
        switch (savedAnalysis1){
            case LEFT: {
                return "Left";
            } case RIGHT: {
                return "Right";
            } case CENTER: {
                return "Middle";
            }
        }
        return "Left";
    }

    public String scanSavedRed() {
//        myOpMode.telemetry.addData("CV Analysis", savedAnalysis);
//        myOpMode.telemetry.update();
        switch (savedAnalysis) {
            case LEFT: {
                return "Left";
            } case RIGHT: {
                return "Right";
            } case CENTER: {
                return "Middle";
            }
        }
        return "Left";
    }

    public String scanPropCV(String color) {
        snapshotCV(color);
        if (color.equals("Red"))
            return scanSavedRed();
        if (color.equals("Blue"))
            return scanSavedBlue();
        return scanSavedRed();
    }


    /**
     * TO READ IN INIT: we will have to replace the waitForStart() method with the waiting
     * for start loop found in ocv AutoInitDetectExample.java, and most of the rest of the
     * code will remain the same.
     *
     * However, actually, it seems like we can still use waitForStart() and just grab a snapshot
     * at the very beginning of our auto method and still be fine? idk why they call it in the
     * loop and then call it again after usr presses start. This means we don't have to rewrite
     * the opModes, beyond calling initCV().
     *
     */

    /**
     * What follows is the OpenCV Pipeline for processing the images from the webcam
     * and extracting the position of our prop. This code, and the rest of the OpenCV
     * code, was originally adapted from skystone example code found at
     * https://github.com/OpenFTC/EasyOpenCV.
     *
     * I really don't know why the pipeline class exists inside another class, but this
     * seems to be the convention.
     *
     * The method getAnalysis() will return an enum of the prop's position
     */

    public static class TSEDeterminationPipeline extends OpenCvPipeline
    {
        //An enum to define the prop position
        public enum TSEPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        //Color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         * TODO: Check these points w/ our camera position and redefine them based on prop location
         */
        //every .45 mm is 1 pixel (will depend on monitor size)
        //center of the pixel bucket lines up with the center black line
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(97,230);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(325,230);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(550,230);
        static final int REGION_WIDTH = 51;
        static final int REGION_HEIGHT = 51;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        //Working variables
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile TSEPosition position = TSEPosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1); //TODO: Change coi to 1 to change to Cr?
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {

            /*
             * Overview of what we're doing (Skystone):
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * current editor's note: We will probably be using Cr to detect our
             * Home-Depot-Orange TSE.
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            //Get the Cb channel of the input frame after conversion to YCrCb
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //Find the max of the three averages
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = TSEPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = TSEPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = TSEPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        //Call this from the OpMode thread to obtain the latest analysis
        public TSEPosition getAnalysis() {return position;}
    }

    public static class TSEDeterminationPipelineWithSide extends OpenCvPipeline {

        public enum TSEPosition {
            LEFT,
            CENTER,
            RIGHT
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         * TODO: Check these points w/ our camera position and redefine them based on prop location
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(115, 200);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(270, 200);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(437, 200);
        static final int REGION_WIDTH = 51;
        static final int REGION_HEIGHT = 51;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb, region2_Cb, region3_Cb; //TODO: May need to change to Cr instead of Cb to better detect TSE
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile TSEDeterminationPipelineWithSide.TSEPosition position = TSEDeterminationPipelineWithSide.TSEPosition.LEFT;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1); //TODO: Change coi to 1 to change to Cr?
        }

        @Override
        public void init(Mat firstFrame) {

            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            int minOneTwo = Math.min(avg1, avg2);
            int min = Math.min(minOneTwo, avg3);

            if (min == avg1) // Was it from region 1?
            {
                position = TSEDeterminationPipelineWithSide.TSEPosition.LEFT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (min == avg2) // Was it from region 2?
            {
                position = TSEDeterminationPipelineWithSide.TSEPosition.CENTER; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (min == avg3) // Was it from region 3?
            {
                position = TSEDeterminationPipelineWithSide.TSEPosition.RIGHT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            return input;
        }

        public TSEDeterminationPipelineWithSide.TSEPosition getAnalysis() {
            return position;
        }
    }}
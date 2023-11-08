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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


/** PSUEDOCODE:
 *
 *
 */

public class PowerPlayAuto {

    private LinearOpMode myOpMode;       // Access to the OpMode object
    private PowerPlayPackBot myRobot;        // Access to the Robot hardware
    private HardwareMap myHardwareMap;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    boolean PHONE_IS_PORTRAIT = false;
    String VUFORIA_KEY =
            "Adg0Y9v/////AAABmergYcIrxEPZmeeflCjLz7pIqlTKWre7SqTXe94Qzd8Mdv2CWJzL6Dl8jsSNRH7XEzhINohMrWO0MY4Z0Sm9UYg/lPNTYbfXQSXmTAG2623GHGCogvStqInHKbTqICTPgNJYbe4iGRmcJmvCN1oo+N0+0KzZRaCTHXjqZbUPo430TQNUYELOmdMx5+uT2O1jTx75XZ2MMGcanX0aSMXpzE47V6PMAtXAD11h1CaNB4/dYE+CgkqWTEN/PBKvTYJdCMhNUH6PuENY8q6wkv5aTql0q5ZFNamLN6Vl1PxSgiChwBFMZC53ASMbo606s4TzFgcAD3+AiUZHFk2LmYx088Xj5XkvW1s1DN9KhDR4EYn1";

    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor


    // Class Members
    private OpenGLMatrix location = null;
    //private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;

    private final int rows = 640;
    private final int cols = 480;

    private String position = "";
//    OpenCvCamera webcam;

    private double t, tInit;

    public PowerPlayAuto(LinearOpMode theOpMode, PowerPlayPackBot theRobot, HardwareMap theHwMap) {
        myOpMode = theOpMode;
        myRobot = theRobot;
        myHardwareMap = theHwMap;
    }
    /**
     * Initialize the Vuforia localization engine. Must be done to init TFOD.
     */
    /*
    public void initVuforia() {
        HardwareMap hardwareMap = myHardwareMap;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
            /*
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    /*
    public void initTfod() {
        HardwareMap hardwareMap = myHardwareMap;
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 1080;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.3, 10.0 / 3.0);
        }
    }
*/

    public void shutdownTFOD() {
        tfod.shutdown();
    }


    public double getHeading() {
        return myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void liftTest(){
        double liftSpeed = .3;
        int sleepTime = 5000;

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.groundHeight, liftSpeed);
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.coneStack, liftSpeed);
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.lowHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        // middle height and rest height need to be fixed, do later
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.middleHeight, liftSpeed);
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.restHeight, liftSpeed);
        myOpMode.sleep(sleepTime);
    }

    public void openGrabber(){
        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieOpen);
        myRobot.chicken.setPosition(PowerPlayPackBot.chickenOpen);
    }

    public void closeGrabber(){
        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieClosed);
        myRobot.chicken.setPosition(PowerPlayPackBot.chickenClosed);
    }

    public String scan(String side){
        // Scanner scan = new Scanner(System.in);

        float gain = 5;
        final float[] hsvValues = new float[3];
        String result = "";

        if (side.equals("right")){
            // Turns the light on if it's not on already.
            if (myRobot.colorSensor1 instanceof SwitchableLight) {
                ((SwitchableLight) myRobot.colorSensor1).enableLight(true);
            }

            myRobot.colorSensor1.setGain(gain);

            // Actually gets the colors from the sensor
            NormalizedRGBA colors = myRobot.colorSensor1.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);

            if (colors.red > colors.blue && colors.red > colors.green){
                result = "Red";
            }
            else if (colors.blue > colors.red && colors.blue > colors.green){
                result = "Blue";
            }
            else if (colors.green > colors.red && colors.green > colors.blue){
                result = "Green";
            }
            else
                result = "Green";
        }
        else if (side.equals("left")){
            // Turns the light on if it's not on already.
            if (myRobot.colorSensor2 instanceof SwitchableLight) {
                ((SwitchableLight) myRobot.colorSensor2).enableLight(true);
            }

            myRobot.colorSensor2.setGain(gain);

            // Actually gets the colors from the sensor
            NormalizedRGBA colors = myRobot.colorSensor2.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);

            if (colors.red > colors.blue && colors.red > colors.green){
                result = "Red";
            }
            else if (colors.blue > colors.red && colors.blue > colors.green){
                result = "Blue";
            }
            else if (colors.green > colors.red && colors.green > colors.blue){
                result = "Green";
            }
            else
                result = "Red";
        }
        return result;
    }

    public void scanMedParkLeft(){
        double driveSpeed = 0.4;
        double liftSpeed = 0.7;
        int sleepTime = 450;
        String side = "left";

        myRobot.parmesan.setPosition(0);
        myRobot.fowl.setPosition(0);

        closeGrabber();
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.middleHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 44.65, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        String result = scan(side);

        myRobot.advancedEncoderDrive(myOpMode, 40, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 41, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.middleHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 84, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.topHeight, liftSpeed);

        myRobot.advancedEncoderDrive(myOpMode, 1, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        openGrabber();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1.5, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftDown(myOpMode, PowerPlayPackBot.coneStack, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24.5, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        // Pick up another cone and place it on the high
        /*

        myRobot.advancedEncoderDrive(myOpMode, 44, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.coneStack, liftSpeed);
        myOpMode.sleep(sleepTime);

        closeGrabber();
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.lowHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 4, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftDown(myOpMode, PowerPlayPackBot.restHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 40, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        while (getHeading() < 180) {
            myRobot.rotateCCW(.4);
        }

        myRobot.advancedEncoderDrive(myOpMode, 11.5, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.topHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 6, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        openGrabber();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 6, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftDown(myOpMode, PowerPlayPackBot.restHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 11.5, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);
        */

        // Park - will change direction if we add the high junction

        if (result.equals("Red")) {
            myRobot.advancedEncoderDrive(myOpMode, 93.5, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Blue")){
            myRobot.advancedEncoderDrive(myOpMode, 40, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Green")){
            myOpMode.sleep(sleepTime);
        }
    }

    public void scanMedParkRight(){
        double driveSpeed = 0.4;
        double liftSpeed = 0.7;
        int sleepTime = 450;
        String side = "right";

        myRobot.parmesan.setPosition(0);
        myRobot.fowl.setPosition(0);

        closeGrabber();
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.middleHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 44.65, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        String result = scan(side);

        myRobot.advancedEncoderDrive(myOpMode, 40, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 42, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.middleHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 92, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.topHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        //myRobot.advancedEncoderDrive(myOpMode, 0.25, "Forward", driveSpeed);
        //myOpMode.sleep(sleepTime);

        openGrabber();
        myOpMode.sleep(sleepTime);

        //Move backward a little to clear the junction
        myRobot.advancedEncoderDrive(myOpMode, 0.25, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftDown(myOpMode, PowerPlayPackBot.coneStack, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24.5, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        // Pick up another cone and place it on the high
        // This might end up being in a loop
        /*
        myRobot.advancedEncoderDrive(myOpMode, 44, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        *loop header
        while (updatedConeStack > 0)
        {
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.coneStack, liftSpeed);
        myOpMode.sleep(sleepTime);

        closeGrabber();
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.lowHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 7, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftDown(myOpMode, PowerPlayPackBot.restHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 43, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        while (getHeading() < 180) {
            myRobot.rotateCW(.4);
        }

        myRobot.advancedEncoderDrive(myOpMode, 11.5, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.topHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 3, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        openGrabber();
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 3, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.moveLiftDown(myOpMode, PowerPlayPackBot.restHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 11.5, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        -below added for loop-

        while (getHeading() > 0) {
            myRobot.rotateCW(-.4);
        }

        myRobot.advancedEncoderDrive(myOpMode, 43, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        updatedCoinStack -= 200;

        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.coneStack, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 7, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);
        }
        */

        // Park - will change direction if we add placing on the high junction

        if (result.equals("Red")) {
            //myRobot.advancedEncoderDrive(myOpMode, 1, "Forward", driveSpeed);
            //myOpMode.sleep(sleepTime);
        }
        if (result.equals("Blue")){
            myRobot.advancedEncoderDrive(myOpMode, 42, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Green")){
            myRobot.advancedEncoderDrive(myOpMode, 99, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
    }

    public void scanGroundParkRight(){
        /* Move forward enough to read the sleeve
           Scan the sleeve
           Output data about the sleeve
           Drop the cone
           Maybe need to sleep a few milliseconds before and after scan?
           Move depending on the sleeve
        */
        double driveSpeed = 0.4;
        double liftSpeed = 0.3;
        int sleepTime = 450;
        String side = "right";

        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieClosed);
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.groundHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 19, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        String result = scan(side);

        myRobot.advancedEncoderDrive(myOpMode, 3.1, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        //Drop on ground junction
        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieOpen);
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftDown(myOpMode, PowerPlayPackBot.restHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (result.equals("Red")) {
            myRobot.advancedEncoderDrive(myOpMode, 10, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Backward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 25, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Blue")){
            myRobot.advancedEncoderDrive(myOpMode, 12, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Green")){
            myRobot.advancedEncoderDrive(myOpMode, 11, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 23, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 26, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
    }

    public void scanGroundParkLeft(){
        /* Move forward enough to read the sleeve
           Scan the sleeve
           Output data about the sleeve
           Drop the cone
           Maybe need to sleep a few milliseconds before and after scan?
           Move depending on the sleeve
        */
        double driveSpeed = 0.4;
        int sleepTime = 500;
        double liftSpeed = 0.3;
        String side = "left";

        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieClosed);
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.groundHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 18.5, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);


        String result = scan(side);

        myRobot.advancedEncoderDrive(myOpMode, 3, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1.5, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        //Drop on ground junction - need to finish
        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieOpen);
        myOpMode.sleep(sleepTime);
        myRobot.moveLiftDown(myOpMode, PowerPlayPackBot.restHeight, liftSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1.5, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (result.equals("Red")) {
            myRobot.advancedEncoderDrive(myOpMode, 10, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 23, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 25, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Blue")){
            myRobot.advancedEncoderDrive(myOpMode, 11, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Green")){
            myRobot.advancedEncoderDrive(myOpMode, 10, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Backward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 25, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
    }

    // Drives left and then forward
    public void parkNoSignal() {
        double driveSpeed = 0.4;
        int sleepTime = 500;
        double liftSpeed = 0.3;

        myOpMode.sleep(sleepTime);
        // myRobot.moveLiftUp(myOpMode, PowerPlayPackBot.groundHeight, liftSpeed);
        // myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Left", driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Forward", driveSpeed);

        myOpMode.sleep(sleepTime);
    }

    public void scanPark(){
        double driveSpeed = 0.4;
        int sleepTime = 500;
        String side = "right";

        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieClosed);
        myRobot.chicken.setPosition(PowerPlayPackBot.chickenClosed);

        myRobot.advancedEncoderDrive(myOpMode, 19, "Left", driveSpeed);
        myOpMode.sleep(sleepTime);

        String result = scan(side);

        if (result.equals("Red")) {
            myRobot.advancedEncoderDrive(myOpMode, 18, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 24, "Backward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 25, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Blue")){
            myRobot.advancedEncoderDrive(myOpMode, 12, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
        if (result.equals("Green")){
            myRobot.advancedEncoderDrive(myOpMode, 18, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 23, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 26, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
        }
    }
}


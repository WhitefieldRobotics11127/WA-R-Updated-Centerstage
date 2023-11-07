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
    //private VuforiaTrackables   targets = this.vuforia.loadTrackablesFromAsset("Skystone");        // List of active targets
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    //private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    boolean PHONE_IS_PORTRAIT = false;
    //String VUFORIA_KEY =
    //        "Adg0Y9v/////AAABmergYcIrxEPZmeeflCjLz7pIqlTKWre7SqTXe94Qzd8Mdv2CWJzL6Dl8jsSNRH7XEzhINohMrWO0MY4Z0Sm9UYg/lPNTYbfXQSXmTAG2623GHGCogvStqInHKbTqICTPgNJYbe4iGRmcJmvCN1oo+N0+0KzZRaCTHXjqZbUPo430TQNUYELOmdMx5+uT2O1jTx75XZ2MMGcanX0aSMXpzE47V6PMAtXAD11h1CaNB4/dYE+CgkqWTEN/PBKvTYJdCMhNUH6PuENY8q6wkv5aTql0q5ZFNamLN6Vl1PxSgiChwBFMZC53ASMbo606s4TzFgcAD3+AiUZHFk2LmYx088Xj5XkvW1s1DN9KhDR4EYn1";

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
    public OpenCvWebcam webcam;
    public TSEDeterminationPipeline pipeline;
    public TSEDeterminationPipelineWithSide pipelineOther;
    public TSEDeterminationPipeline.TSEPosition savedAnalysis =
            TSEDeterminationPipeline.TSEPosition.LEFT;
    public TSEDeterminationPipelineWithSide.TSEPosition savedAnalysis1 =
            TSEDeterminationPipelineWithSide.TSEPosition.LEFT;

    private double t, tInit;

    public CenterstageAuto(LinearOpMode theOpMode, CenterstagePackBot theRobot, HardwareMap theHwMap) {
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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

    public void shutdownTFOD() {
        tfod.shutdown();
    }
*/

    public double getHeading() {
        return myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /*
    public void openGrabber(){
        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieOpen);
        myRobot.chicken.setPosition(PowerPlayPackBot.chickenOpen);
    }

    public void closeGrabber(){
        myRobot.rotisserie.setPosition(PowerPlayPackBot.rotisserieClosed);
        myRobot.chicken.setPosition(PowerPlayPackBot.chickenClosed);
    }

     */

    public void placeParkColorSensor(String color, String side){
        //The base method to place a purple pixel on the spike it's supposed to be on and park,
        //using color sensors
        //THE ROBOT MUST BE FACING FORWARD AT START
        double driveSpeed = 0.4;
        double rotateSpeed = 0.4;
        int sleepTime = 300;
        String markerPos = "Middle";

        /*
        myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE THAT PUTS THE COLOR SENSORS IN LINE
            WITH THE PROP LINES, "Forward", driveSpeed);
        myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE RIGHT, "Right", driveSpeed);
        if (scan(side, color) == true){
            markerPos = "Right";
            String driveDirection = "Left";
            myOpMode.telemetry.addData("Marker Pos", markerPos);
            myOpMode.telemetry.update();
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE RIGHT ON SPIKE, "Right", driveSpeed);
            WHICHEVERSERVO.OPENLEFT/RIGHTGRABBER();
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO CENTER, "Left", driveSpeed);
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE FORWARD, "Forward", driveSpeed);
        }
        else {
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE LEFT, "Left", driveSpeed);
            if (scan(side, color) == true){
                markerPos = "Left";
                String driveDirection = "Right";
                myOpMode.telemetry.addData("Marker Pos", markerPos);
                myOpMode.telemetry.update();
                myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE LEFT ON SPIKE, "Left", driveSpeed);
                WHICHEVERSERVO.OPENLEFT/RIGHTGRABBER();
                myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO CENTER, "Right", driveSpeed);
                myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE FORWARD, "Forward", driveSpeed);
            }
        }
        if (markerPos.equals("Middle"));{
            myOpMode.telemetry.addData("Marker Pos", markerPos);
            myOpMode.telemetry.update();
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO CENTER PERHAPS DEPENDENT ON DRIVEDIRECTION, driveDirection, driveSpeed);
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE FORWARD, "Forward", driveSpeed);
            WHICHEVERSERVO.OPENLEFT/RIGHTGRABBER();
            myRobot.advancedEncoderDrive(myOpMode, SOME REMAINING DISTANCE FORWARD, "Forward", driveSpeed);
        }

        if (side.equals("Left")){
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO PARK BASED ON COLOR EXPRESSION LIKE TO THE RIGHT, (color.equals("Red") ? "Right" : "Left"), driveSpeed);
            myOpMode.sleep(sleepTime);
            }
        else {
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO PARK BASED ON COLOR EXPRESSION LIKE TO THE LEFT, (color.equals("Red") ? "Right" : "Left"), driveSpeed);
            myOpMode.sleep(sleepTime);
        }
         */
    }

    public void park(String color){
        double driveSpeed = 0.4;
        double rotateSpeed = 0.4;
        int sleepTime = 300;
        myRobot.leftClaw.setPosition(CenterstagePackBot.leftClawClosed);
        myOpMode.sleep(sleepTime);
        myRobot.rightClaw.setPosition(CenterstagePackBot.rightClawClosed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 1, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);
        if (color.equals("Blue"))
            myRobot.advancedEncoderDrive(myOpMode, 48, "Left", driveSpeed);
        if (color.equals("Red"))
            myRobot.advancedEncoderDrive(myOpMode, 48, "Right", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.leftClaw.setPosition(CenterstagePackBot.leftClawOpen);
        myOpMode.sleep(sleepTime);
        myRobot.rightClaw.setPosition(CenterstagePackBot.rightClawOpen);
        myOpMode.sleep(sleepTime);
    }

    public void placePark(String color, String side){
        //The base method to place a purple pixel on the spike it's supposed to be on and park
        // THE ROBOT MUST BE FACING BACKWARDS AT START
        double driveSpeed = 0.4;
        double rotateSpeed = 0.4;
        int sleepTime = 300;
        String markerPos = "Left";

        if (color.equals("Red"))
            markerPos = scanSavedRed();
        else if (color.equals("Blue"))
            markerPos = scanSavedBlue();
        myOpMode.telemetry.addData("Marker Pos", markerPos);
        myOpMode.telemetry.update();

        /*
        myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE THAT CLEARS THE WALL, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);
        while (getHeading() < 180){
            myRobot.rotateCCW(rotateSpeed);
        }
        myOpMode.sleep(sleepTime);
        myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE AT THE FIRST 2 SPIKES LEVEL, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (markerPos.equals("Right")){
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO HOVER OVER THE SPIKE/AVOID THE METAL BARS, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
            WHICHEVERSERVO.OPENLEFT/RIGHTGRABBER();
            myOpMode.sleep(sleepTime);
        }
        else if (markerPos.equals("Middle")){
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
            WHICHEVERSERVO.OPENLEFT/RIGHTGRABBER();
            myOpMode.sleep(sleepTime);
        }
        else { //when markerPos.equals("Left") or we can't/don't detect the prop
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO HOVER OVER THE SPIKE/AVOID THE METAL BARS, "Left", driveSpeed);
            myOpMode.sleep(sleepTime);
            WHICHEVERSERVO.OPENLEFT/RIGHTGRABBER();
            myOpMode.sleep(sleepTime);
        }

        if (!markerPos.equals("Middle")){
            if (markerPos.equals("Right")){
                myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE, "Left", driveSpeed);
                myOpMode.sleep(sleepTime);
                myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE, "Forward", driveSpeed);
                myOpMode.sleep(sleepTime);
            }
            if (markerPos.equals("Left")){
                myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE, "Right", driveSpeed);
                myOpMode.sleep(sleepTime);
                myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE, "Forward", driveSpeed);
                myOpMode.sleep(sleepTime);
            }
        else{
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
            }
        }

        if (side.equals("Left")){
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO PARK BASED ON BOOLEAN STATEMENT AT RIGHT, (color.equals("Red") ? "Right" : "Left"), driveSpeed);
            myOpMode.sleep(sleepTime);
            }
        else {
            myRobot.advancedEncoderDrive(myOpMode, SOME DISTANCE TO PARK BASED ON BOOLEAN STATEMENT AT RIGHT, (color.equals("Red") ? "Right" : "Left"), driveSpeed);
            myOpMode.sleep(sleepTime);
        }
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
     * and extracting the position of our TSE. This code, and the rest of the OpenCV
     * code, was adapted from skystone example code found at https://github.com/OpenFTC/EasyOpenCV.
     *
     * I really don't know why the pipeline class exists inside another class, but this
     * seems to be the convention.
     *
     * The method getAnalysis() will return an enum of the TSE's position
     */

    public static class TSEDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum TSEPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         * TODO: Check these points w/ our camera position and redefine them based on prop location
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90,200);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(300,200);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(490,200);
        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 50;

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

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb; //TODO: May need to change to Cr instead of Cb to better detect TSE
        //Mat region1_Cr, region2_Cr, region3_Cr;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        //Mat Cr = new Mat();
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

        /*
        void inputToCr(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }
        */

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

            //region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            //region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
            //region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {

            /*
             * Overview of what we're doing:
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

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
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


            /*
             * Find the max of the 3 averages
             */
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

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public TSEPosition getAnalysis()
        {
            return position;
        }
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90, 200);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(300, 200);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(490, 200);
        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 50;

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
        //Mat region1_Cr, region2_Cr, region3_Cr;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        //Mat Cr = new Mat();
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

            //region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            //region2_Cr = Cr.submat(new Rect(region2_pointA, region2_pointB));
            //region3_Cr = Cr.submat(new Rect(region3_pointA, region3_pointB));
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


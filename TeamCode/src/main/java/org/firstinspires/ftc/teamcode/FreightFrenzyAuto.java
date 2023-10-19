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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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

import java.util.List;


/** PSUEDOCODE:
 * Start on the Storage Unit side
 *
 * Scan Barcode
 *
 * Strafe right 24in
 * Drive forward 24in (dist will change once we have more details about bot design)
 * [Place Block]
 *
 * Drive backward 24in (")
 * Strafe left 48in
 *
 * Spin carousel
 *
 * Drive forward 24in to Park
 *
 *
 *
 * Start on the Warehouse side
 *
 * Scan Barcode
 *
 * Strafe left 24in
 * Drive forward 24in (")
 * [Place Block]
 *
 * Drive backward 24in (")
 * Rotate CW 90˚
 * Strafe right a little
 * Drive forward 48in to park in Warehouse
 *
 *
 */


public class FreightFrenzyAuto {

    private LinearOpMode        myOpMode;       // Access to the OpMode object
    private FreightFrenzyPackBot myRobot;        // Access to the Robot hardware
    private HardwareMap         myHardwareMap;
    //private VuforiaTrackables   targets = this.vuforia.loadTrackablesFromAsset("Skystone");        // List of active targets
    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    //private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    boolean PHONE_IS_PORTRAIT = false;
    // String VUFORIA_KEY =
            // "Adg0Y9v/////AAABmergYcIrxEPZmeeflCjLz7pIqlTKWre7SqTXe94Qzd8Mdv2CWJzL6Dl8jsSNRH7XEzhINohMrWO0MY4Z0Sm9UYg/lPNTYbfXQSXmTAG2623GHGCogvStqInHKbTqICTPgNJYbe4iGRmcJmvCN1oo+N0+0KzZRaCTHXjqZbUPo430TQNUYELOmdMx5+uT2O1jTx75XZ2MMGcanX0aSMXpzE47V6PMAtXAD11h1CaNB4/dYE+CgkqWTEN/PBKvTYJdCMhNUH6PuENY8q6wkv5aTql0q5ZFNamLN6Vl1PxSgiChwBFMZC53ASMbo606s4TzFgcAD3+AiUZHFk2LmYx088Xj5XkvW1s1DN9KhDR4EYn1";

    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    int test = 0;

    // Class Members
    private OpenGLMatrix location = null;
    //private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;

    private final int rows = 640;
    private final int cols = 480;

    private String position = "";
//    OpenCvCamera webcam;

    private double t, tInit;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private final double lineToHubDist = 15; //inches
    private final double lineToCarouselDist = 12; //inches

    //OpenCV Fields
    public OpenCvWebcam webcam;
    public TSEDeterminationPipeline pipeline;
    public TSEDeterminationPipeline.TSEPosition savedAnalysis =
            TSEDeterminationPipeline.TSEPosition.LEFT;
    
    
    public FreightFrenzyAuto(LinearOpMode theOpMode, FreightFrenzyPackBot theRobot, HardwareMap theHwMap) {
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
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.3, 10.0/3.0);
        }
    }
    */

    public void shutdownTFOD() {
        tfod.shutdown();
    }
    
    public String scanBarcode() {
        double recogX = -1;
        String recogLabel = "";
        double screenMidX = 400; //the middle

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            int timeout = 0;
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            while ((updatedRecognitions == null || updatedRecognitions.size() < 1) && timeout < 5000000) {
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (!recognition.getLabel().equals("Marker"))
                            updatedRecognitions.remove(recognition);

                    }
//                    if (updatedRecognitions.size() < 1)
//                        updatedRecognitions = null;
                }
                timeout++;
            }
            myOpMode.telemetry.addData("timeout", timeout);
            myOpMode.telemetry.update();

            if (updatedRecognitions != null) {
//                myOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals("Marker")) {
                        recogX = (recognition.getLeft() + recognition.getRight()) / 2.0;
//                        recogLabel = recognition.getLabel();
                    }

//                    recogX = (recognition.getLeft() + recognition.getRight()) / 2.0;
//                    recogLabel = recognition.getLabel();
//                    myOpMode.telemetry.addData("label", label);
                    myOpMode.telemetry.addData("x", recognition.getLeft());
                    myOpMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    /*
                    myOpMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    myOpMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                     */
                }
                myOpMode.telemetry.update();
            } else {
//                shutdownTFOD();
                return "Left"; //if its the one the bot can't see. Runs if timeout maxes out
            }
        } else {
            myOpMode.telemetry.addData("TFOD NOT INITIALIZED", "");
            myOpMode.telemetry.update();
        }
//        shutdownTFOD();
        if (recogX > 0 && recogX < screenMidX) { //if screen left
            return "Right";
        } else if (recogX > screenMidX) { //if screen right
            return "Middle";
        } else {
            return "Left";
        }
    }

    public void dropBlock(String pos) {
        double liftSpeed = 0.3;

        if (pos.equals("Left") || pos.equals("Bottom")) {
            myRobot.moveLiftUp(myOpMode, 0.0, 0.2);
        }
        else if (pos.equals("Middle")) {
            myRobot.rotisserie.setPosition(FreightFrenzyPackBot.rotisserieMid);
            myOpMode.sleep(1500);
            myRobot.moveLiftUp(myOpMode, FreightFrenzyPackBot.middleLevelHeight, liftSpeed);
        }
        else if (pos.equals("Right") || pos.equals("Top")) {
            myRobot.rotisserie.setPosition(FreightFrenzyPackBot.rotisserieMid);
            myOpMode.sleep(1500);
            myRobot.moveLiftUp(myOpMode, FreightFrenzyPackBot.topLevelHeight, liftSpeed);
        }
        myRobot.rotisserie.setPosition(FreightFrenzyPackBot.rotisserieOut);
        myOpMode.sleep(2000);

        myRobot.rotisserie.setPosition(pos.equals("Left") ? FreightFrenzyPackBot.rotisserieIn : FreightFrenzyPackBot.rotisserieMid);
        myOpMode.sleep(1000);

        myRobot.moveLiftDown(myOpMode, FreightFrenzyPackBot.bottomLevelHeight, liftSpeed);

        if (!pos.equals("Left")) myRobot.rotisserie.setPosition(FreightFrenzyPackBot.rotisserieIn);
    }

    public double getHeading() {
        return myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void duckParkDuck(String color) { //Front of bot is the Intake/Duck Spinner. Front starts facing the wall.
        double driveSpeed = 0.4;
        int sleepTime = 500;

        myRobot.advancedEncoderDrive(myOpMode, 24, "Backward", driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, (color.equals("Red") ? "Right" : "Left"), driveSpeed);

        myOpMode.sleep(sleepTime);

//        myRobot.advancedEncoderDrive(myOpMode, lineToCarouselDist, "Forward", driveSpeed);
        myRobot.driveToDist(myOpMode, "Front", 9, driveSpeed/2);

        myOpMode.sleep(sleepTime);

        myRobot.spinDuck(myOpMode, color);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, lineToCarouselDist + 12, "Backward", driveSpeed);
    }

    public void duckParkWarehouse(String color) { //Front Starts facing wall
        double driveSpeed = 0.4;
        double rotateSpeed = 0.4;
        int sleepTime = 500;

        myRobot.advancedEncoderDrive(myOpMode, 24, "Backward", driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, (color.equals("Red") ? "Right" : "Left"), driveSpeed);

        myOpMode.sleep(sleepTime);

//        myRobot.advancedEncoderDrive(myOpMode, lineToCarouselDist, "Forward", driveSpeed);
        myRobot.driveToDist(myOpMode, "Front", 9, driveSpeed/2);

        myOpMode.sleep(sleepTime);

        myRobot.spinDuck(myOpMode, color);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, lineToCarouselDist, "Backward", driveSpeed);

        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            while (getHeading() < 90) {
                myRobot.rotateCCW(rotateSpeed);
            }
        } else if (color.equals("Blue")){
            while (getHeading() > -90) {
                myRobot.rotateCW(rotateSpeed);
            }
        }
        myRobot.driveStop();

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 72, "Forward", driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 15, (color.equals("Red") ? "Right" : "Left"), driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Forward", driveSpeed);
    }

    public void parkWarehouse(String color) { //Front (Intake) Starts facing Warehouse
        double driveSpeed = 0.4;
        int sleepTime = 500;

//        myRobot.advancedEncoderDrive(myOpMode, 3, (color.equals("Red") ? "Right" : "Left"), driveSpeed);

//        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 24, "Forward", driveSpeed);
    }

    public void shipDuckParkDuck(String color) { //Front of bot is the Intake/Duck Spinner. Front starts facing the wall.
        double driveSpeed = 0.4;
        double rotateSpeed = 0.4;
        int sleepTime = 400;

        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Red") ? 13 : 12.5), "Backward", driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.rotisserie.setPosition(FreightFrenzyPackBot.rotisserieOut);

        myOpMode.sleep(4000);

        myRobot.rotisserie.setPosition(FreightFrenzyPackBot.rotisserieIn);

//        myOpMode.sleep(sleepTime);

//        myRobot.advancedEncoderDrive(myOpMode, 3, "Forward", driveSpeed); //unsigned
/*
        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            while (getHeading() > -85) {
                myRobot.rotateCCW(rotateSpeed);
            }
        } else {
            while (getHeading() < 85) {
                myRobot.rotateCW(rotateSpeed);
            }
        }
        myRobot.driveStop();

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 42, "Forward", driveSpeed);

        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            while (getHeading() < 0) {
                myRobot.rotateCW(rotateSpeed);
            }
        } else {
            while (getHeading() > 0) {
                myRobot.rotateCCW(rotateSpeed);
            }
        }
        myRobot.driveStop();
*/
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 3, "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 50, (color.equals("Red") ? "Right" : "Left"), driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.driveToDist(myOpMode, (color.equals("Red") ? "Right" : "Left"), (color.equals("Red") ? 2 : 3.5), driveSpeed/2);
        myOpMode.sleep(sleepTime);

//        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Red") ? 13.5 : 8.5), "Forward", driveSpeed/2);
        myRobot.driveToDist(myOpMode, "Front", (color.equals("Red") ? 6.77 : 7.5), driveSpeed/2);

        myOpMode.sleep(sleepTime);

        myRobot.spinDuck(myOpMode, color);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Red") ? 15 : 13), "Backward", driveSpeed);
    }

    public void readShipDuckParkDuck(String color) { //Front of bot is the Intake/Duck Spinner. Front starts facing the wall.
        double driveSpeed = 0.4;
        double rotateSpeed = 0.4;
        int sleepTime = 300;
        String markerPos = "Left";

//        myOpMode.sleep(2000); //give TFOD time to init
        markerPos = scanSaved();
        myOpMode.telemetry.addData("Marker Pos", markerPos);
        myOpMode.telemetry.update();

        myRobot.advancedEncoderDrive(myOpMode, 25, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (color.equals("Blue")) {
            while (getHeading() < 60) {
                myRobot.rotateCW(rotateSpeed);
            }
        } else {
            while (getHeading() > -45) {
                myRobot.rotateCCW(rotateSpeed);
            }
        }
        myRobot.driveStop();
        myOpMode.sleep(sleepTime);

//        myRobot.advancedEncoderDrive(myOpMode, 2, "Backward", driveSpeed);
//
//        myOpMode.sleep(sleepTime);
//
//        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Red") ? 19.5 : 30), (color.equals("Red") ? "Left" : "Right"), driveSpeed);
//
//        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Blue") ? 6.77 : 1), "Backward", driveSpeed*.75);

        myOpMode.sleep(sleepTime);

        dropBlock(markerPos);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Blue") ? 7 : 2), "Forward", driveSpeed);
        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            while (getHeading() < -15) {
                myRobot.rotateCW(rotateSpeed);
            }
        } else {
            while (getHeading() > 13) {
                myRobot.rotateCCW(rotateSpeed);
            }
        }
        myRobot.driveStop();
        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            myRobot.advancedEncoderDrive(myOpMode, 5, "Forward", driveSpeed);
            myOpMode.sleep(sleepTime);
        }

        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Blue") ? 15 : 20), (color.equals("Red") ? "Right" : "Left"), driveSpeed);
        myOpMode.sleep(sleepTime);

        myRobot.driveToDist(myOpMode, (color.equals("Red") ? "Right" : "Left"), (color.equals("Red") ? 2 : 2), driveSpeed/2);
        myOpMode.sleep(sleepTime);

//        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Red") ? 13.5 : 8.5), "Forward", driveSpeed/2);
        myRobot.driveToDist(myOpMode, "Front", (color.equals("Red") ? 6.77 : 7.3), driveSpeed/2);

        myOpMode.sleep(sleepTime);

        myRobot.spinDuck(myOpMode, color);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Red") ? 13 : 13), "Backward", driveSpeed);
    }

    public void shipDuckParkWarehouse(String color) {
        double driveSpeed = 0.4;
        double rotateSpeed = 0.4;
        int sleepTime = 500;

        myRobot.advancedEncoderDrive(myOpMode, 24 + lineToHubDist, "Backward", driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.rotisserie.setPosition(FreightFrenzyPackBot.rotisserieOut);

        myOpMode.sleep(1000);

        myRobot.rotisserie.setPosition(FreightFrenzyPackBot.rotisserieIn);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, lineToHubDist, "Forward", driveSpeed);

        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            while (getHeading() > -90) {
                myRobot.rotateCW(rotateSpeed);
            }
        } else {
            while (getHeading() < 90) {
                myRobot.rotateCCW(rotateSpeed);
            }
        }
        myRobot.driveStop();

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 48, "Forward", driveSpeed);

        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            while (getHeading() < 0) {
                myRobot.rotateCCW(rotateSpeed);
            }
        } else {
            while (getHeading() > 0) {
                myRobot.rotateCW(rotateSpeed);
            }
        }
        myRobot.driveStop();

        myOpMode.sleep(sleepTime);

//        myRobot.advancedEncoderDrive(myOpMode, lineToCarouselDist, "Forward", driveSpeed);
        myRobot.driveToDist(myOpMode, "Front", 9, driveSpeed/2);

        myOpMode.sleep(sleepTime);

        myRobot.spinDuck(myOpMode, color);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, lineToCarouselDist, "Backward", driveSpeed);

        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            while (getHeading() < 90) {
                myRobot.rotateCCW(rotateSpeed);
            }
        } else {
            while (getHeading() > -90) {
                myRobot.rotateCW(rotateSpeed);
            }
        }
        myRobot.driveStop();

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 72, "Forward", driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 15, (color.equals("Red") ? "Right" : "Left"), driveSpeed);

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 20, "Forward", driveSpeed);
    }

    public void readShipParkWarehouse(String color) {
        double driveSpeed = 0.4;
        double rotateSpeed = 0.25;
        int sleepTime = 500;
        String markerPos = "Left";

//        myOpMode.sleep(2000); //give TFOD time to init
        markerPos = scanSaved();
        myOpMode.telemetry.addData("Marker Pos", markerPos);
        myOpMode.telemetry.update();

        myRobot.advancedEncoderDrive(myOpMode, 1, "Backward", driveSpeed);
//            myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            myOpMode.sleep(sleepTime);
            myRobot.advancedEncoderDrive(myOpMode, 6, "Right", driveSpeed);
            myOpMode.sleep(sleepTime);
        }

        myRobot.advancedEncoderDrive(myOpMode, 20, "Backward", driveSpeed);
        myOpMode.sleep(sleepTime);


        if (color.equals("Red")) {
            while (getHeading() < 60) {
                myRobot.rotateCW(rotateSpeed);
            }
        } else {
            while (getHeading() > -45) {
                myRobot.rotateCCW(rotateSpeed);
            }
        }
        myRobot.driveStop();

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Red") ? 3 : 3.5), "Backward", driveSpeed/2);

        myOpMode.sleep(sleepTime);

        dropBlock(markerPos);

        myOpMode.sleep(sleepTime);

        if (color.equals("Red")) {
            while (getHeading() < 85) {
                myRobot.rotateCW(rotateSpeed);
            }
        } else {
            while (getHeading() > -85) {
                myRobot.rotateCCW(rotateSpeed);
            }
        }
        myRobot.driveStop();

        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, (color.equals("Red") ? 36 : 34.5), (color.equals("Red") ? "Right" : "Left"), driveSpeed);
        myOpMode.sleep(sleepTime);

//        myRobot.driveToDist(myOpMode, (color.equals("Red") ? "Right" : "Left"), (color.equals("Red") ? 0.25 : 0.25), driveSpeed/2);
//        myOpMode.sleep(sleepTime);

        myRobot.advancedEncoderDrive(myOpMode, 35, "Forward", driveSpeed);

    }

    public void initCV() {
        // Documentation omitted for brevity - please see ocvWebcamExample.java for documentation
        int cameraMonitorViewId = myHardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myHardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(myRobot.webcamName, cameraMonitorViewId);
        pipeline = new TSEDeterminationPipeline();
        webcam.setPipeline(pipeline);

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

    public void snapshotCV() {
        savedAnalysis = pipeline.getAnalysis();
    }

    /*
    This is broken into its own method in case we want to
    save the snapshot and analyze it at different times.
     */
    public String scanSaved() {
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

    public String scanBarcodeCV() {
        snapshotCV();
        return scanSaved();
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
     * and extracting the position of out TSE. This code, and the rest of the OpenCV
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
         * TODO: Check these points w/ our camera position
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

}
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

package org.firstinspires.ftc.teamcode.hardware;


import static org.firstinspires.ftc.teamcode.driver.Calibration.D;
import static org.firstinspires.ftc.teamcode.driver.Calibration.F;
import static org.firstinspires.ftc.teamcode.driver.Calibration.I;
import static org.firstinspires.ftc.teamcode.driver.Calibration.P;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RiskyHardware {
    public IMU imu;
    public ElapsedTime runtime = new ElapsedTime();

    public Gamepad.LedEffect redled = new Gamepad.LedEffect.Builder()
            .addStep(1,0,0,1000000)
            .build();
    public Gamepad.LedEffect blueled = new Gamepad.LedEffect.Builder()
            .addStep(0,0,1,1000000)
            .build();
    public Gamepad.LedEffect cool = new Gamepad.LedEffect.Builder()
            .addStep(1,0,0,250)
            .addStep(0,0,1,250)
            .addStep(0,1,0,250)
            .addStep(1,1,0,250)
            .addStep(0,1,1,250)
            .addStep(1,0,1,250)
            .addStep(1,0,0,250)
            .addStep(0,0,1,250)
            .addStep(0,1,0,250)
            .addStep(1,1,0,250)
            .addStep(0,1,1,250)
            .addStep(1,0,1,250)
            .addStep(1,0,0,250)
            .addStep(0,0,1,250)
            .addStep(0,1,0,250)
            .addStep(1,1,0,250)
            .addStep(0,1,1,250)
            .addStep(1,1,1,250)
            .build();
    public RevBlinkinLedDriver.BlinkinPattern pattern;

    //public Servo leftFlap;
    //public Servo rightFlap;

    public DcMotor backleft;
    public DcMotor backright;
    public DcMotor frontright;
    public DcMotor frontleft;

    public DcMotorEx turt;

    public DcMotorEx spinner;
    public DcMotorEx spinner2;
    public DcMotorEx intake;
    public Follower drive;

    public Servo spanker;
    public CRServo leftLift;
    public CRServo rightLift;

    public CRServo intake3;

    public boolean isinit = false;

    public kachow.cameraPipeline camera = new kachow.cameraPipeline();

    public kachow kachow;
    public TouchSensor magnet;

    public HardwareMap hardwareMap =  null;

    /* Constructor */
    public void init(HardwareMap hwMap) {
        runtime.reset();
        hardwareMap = hwMap;
        isinit = true;
        kachow = new kachow();
        kachow.init(hwMap);
        drive = kachow.drive;

        //leftFlap = hardwareMap.get(Servo.class, "LeftFlap");
        //rightFlap = hardwareMap.get(Servo.class, "RightFlap");
        spinner = hardwareMap.get(DcMotorEx.class, "Launcher");
        spinner2 = hardwareMap.get(DcMotorEx.class, "Launcher2");
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        turt = hardwareMap.get(DcMotorEx.class, "Turt");
        backleft = hardwareMap.dcMotor.get("Backleft");
        imu = hardwareMap.get(IMU.class, "imu");
        backright = hardwareMap.dcMotor.get("Backright");
        frontleft = hardwareMap.dcMotor.get("Frontleft");
        frontright = hardwareMap.dcMotor.get("Frontright");
        magnet = hardwareMap.touchSensor.get("Magnet");

        spanker = hardwareMap.get(Servo.class, "Spanker");

        rightLift = hardwareMap.get(CRServo.class, "RightLift");
        leftLift = hardwareMap.get(CRServo.class, "LeftLift");
        intake3 = hardwareMap.get(CRServo.class, "Intake3");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        spinner.setDirection(DcMotor.Direction.FORWARD);
        spinner2.setDirection(DcMotor.Direction.FORWARD);


        turt.setDirection(DcMotorEx.Direction.REVERSE);
//        spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        //spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinner2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        spinner2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        turt.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        spinner2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        //aimer.scaleRange(0, 100);
        //aimer.setDirection(Servo.Direction.REVERSE);


    }


    //Apriltag Detection
    public VisionPortal visionPortal;               // Used to manage the video source.
    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public boolean poseupdated = false;
    public boolean targetFound = false;
    public void initAprilTag(int desiredTag) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        DESIRED_TAG_ID = desiredTag;
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    public void pauseAT(){
        visionPortal.stopStreaming();
    }
    public void resumeAT(){
        visionPortal.resumeStreaming();
    }
    public void stopDetecting(){
        visionPortal.close();
    }
    public void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {

            }
        }
        // Set camera controls unless we are stopping.

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
    public void Atagscan_n_setPose(Follower drive, int milliseconds, int id) {
        long t = System.currentTimeMillis();
        long end = t + milliseconds;
        while (System.currentTimeMillis() < end) {
            detectAT(id);
            apriltag_setPose(drive);
            if (poseupdated) {
                break;// might be best to remove this to get most accurate reading
            }
        }
    }

    public void Atagscan_n_setPose(Follower drive, int milliseconds) {
        long t = System.currentTimeMillis();
        long end = t + milliseconds;
        while (System.currentTimeMillis() < end) {
            detectAT(DESIRED_TAG_ID);
            apriltag_setPose(drive);
            if (poseupdated) {
                break;// might be best to remove this to get most accurate reading
            }
        }
    }

    public void detectAT(int id) {
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((id < 0) || (detection.id == id))  ){
                targetFound = true;
                desiredTag = detection;

                break;  // don't look any further.
            } else {
                //send robot on regular path if not found
            }
        }
    }

    public void detectAT() {
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                targetFound = true;
                desiredTag = detection;

                break;  // don't look any further.
            } else {
                //send robot on regular path if not found
            }
        }
    }

    public void apriltag_setPose(Follower drive) {
        if (targetFound) {
            double xvalue = desiredTag.ftcPose.x;
            double yvalue = desiredTag.ftcPose.y;
            drive.setPose(new Pose((xvalue + 3),-(yvalue),0));
            poseupdated = true;
        }
    }

    public boolean stateChanged = false;
    public state lastState;
    public state previous;
    public state currentState = state.driving;
    public ElapsedTime stateTime = new ElapsedTime();

    public void stateUpdate(state state) {
        previous = currentState;
        currentState = state;
        this.stateChanged = previous != currentState;
        if (stateChanged) {
            stateTime.reset();
            this.lastState = previous;
        }
    }

    public enum state {
        OpenGate, idle, driving, intaking, aimbot, ShootPosetoIntake1, shooting1, shooting2, parking, Intake1toGreenBall, Intake1toShootPose, StartPoseToShootPose, ShootPosetoIntake2, IntakePose2, ShootPosetoIntake3, StartPose2, Intake3toGreenBall, ShootingPose3, ShootingPose4,PerChance1, GoBackFool, Balls, Bald;
    }
}
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


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class SampleHardware {
    public static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
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
    public RevBlinkinLedDriver leds;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    public DigitalChannel magnet;
    public DcMotor backleft;
    public DcMotor backright;
    public DcMotor frontright;
    public DcMotor frontleft;
    public DcMotorEx slide;
    public DcMotor slide2;
    public DcMotor spinny1;
    public DcMotor spinny2;
    public Servo IntakeSlideLeft;

    //public CRServo intake;
    //public CRServo hang1;
    //public CRServo hang2;
    public Servo IntakeSlideRight;
    public Servo IntakePivotLeft = null;
    public Servo IntakePivotRight = null;
    public Servo claw = null;
    public Servo IntakeClaw = null;
    public Servo IntakeWrist = null;
    public Servo IntakeRotate = null;
    public Servo shoulder = null;
    public Servo bucket = null;
    //public Servo clawShoulder = null;
    //public Servo clawElbow = null;
    //public Servo clawWrist = null;
    //public Servo claw = null;
    public Servo bucket1 = null;
    public boolean isinit = false;
    public kachow.cameraPipeline camera = new kachow.cameraPipeline();
    public kachow kachow;
    public HardwareMap hardwareMap =  null;
    //Apriltag Detection
    public VisionPortal visionPortal;               // Used to manage the video source.
    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public boolean poseupdated = false;
    public boolean targetFound = false;
    /**
     *
     * State Machine
     *
     */
    public boolean stateChanged = false;
    public state lastState;
    public state previous;
    public state currentState = state.driving;
    public ElapsedTime stateTime = new ElapsedTime();

    /* Constructor */
    public void init(HardwareMap hwMap) {
        runtime.reset();
        hardwareMap = hwMap;
        isinit = true;
        kachow = new kachow();
        kachow.init(hwMap);


        backleft = hardwareMap.dcMotor.get("Backleft");
        imu = hardwareMap.get(IMU.class, "imu");
        backright = hardwareMap.dcMotor.get("Backright");
        frontleft = hardwareMap.dcMotor.get("Frontleft");
        frontright = hardwareMap.dcMotor.get("Frontright");
        slide = hardwareMap.get(DcMotorEx.class, "Slide");
        slide2 = hardwareMap.dcMotor.get("Slide2");

        spinny1 = hardwareMap.dcMotor.get("Spinny1");
        spinny2 = hardwareMap.dcMotor.get("Spinny2");

        magnet = hardwareMap.get(DigitalChannel.class, "Magnet");
        //slide = hardwareMap.dcMotor.get("Slide");

        leds = hardwareMap.get(RevBlinkinLedDriver .class, "LEDS");

        IntakePivotLeft = hardwareMap.servo.get("IntakepivotLeft");
        IntakePivotRight = hardwareMap.servo.get("IntakepivotRight");

        IntakeSlideLeft = hardwareMap.servo.get("Intakeslideleft");
        IntakeSlideRight = hardwareMap.servo.get("Intakeslideright");
        IntakeClaw = hardwareMap.servo.get("IntakeClaw");
        IntakeRotate = hardwareMap.servo.get("IntakeRotate");
        IntakeWrist = hardwareMap.servo.get("IntakeWrist");

        //hang1 = hardwareMap.get(CRServo.class, "Hang1");
        //hang2 = hardwareMap.get(CRServo.class, "Hang2");

        claw = hardwareMap.servo.get("claw");
        shoulder = hardwareMap.get(Servo.class, "Shoulder");
        bucket = hardwareMap.get(Servo.class, "Bucket");
        bucket1 = hardwareMap.get(Servo.class, "Bucket1");
        //clawShoulder = hardwareMap.servo.get("clawShoulder");
        //clawElbow = hardwareMap.servo.get("clawElbow");
        //clawWrist = hardwareMap.servo.get("clawWrist");
        //claw = hardwareMap.servo.get("claw");



        //reverse``
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide2.setDirection(DcMotor.Direction.REVERSE);
        spinny2.setDirection(DcMotor.Direction.FORWARD);
        spinny1.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        IntakeSlideRight.setDirection(Servo.Direction.REVERSE);
        IntakeSlideLeft.setDirection(Servo.Direction.FORWARD);
        IntakePivotLeft.setDirection(Servo.Direction.FORWARD);
        IntakePivotRight.setDirection(Servo.Direction.FORWARD);
        //intake.setDirection(CRServo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);
        bucket.setDirection(Servo.Direction.FORWARD);
        bucket1.setDirection(Servo.Direction.FORWARD);
        shoulder.setDirection(Servo.Direction.FORWARD);
        //zero behavior
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //imu
        //BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        //imu.initialize(parameters);


        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

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

    public void stateUpdate(state state){
        previous = currentState;
        currentState = state;
        this.stateChanged = previous != currentState;
        if(stateChanged){
            stateTime.reset();
            this.lastState = previous;
        }
    }
    public enum state{
        driving, drivingFirstSample, drivingSecondSample, drivingThirdSample, intaking, rotating, scoring, humanPlayer, drivingHumanPlayer, Transfer, Transfer1, Transfer2, Transfer3, Transfer4, pathFollowing, retract, extend, turnto, aimbot, detecting, outTaking, firstClip, firstSample, secondSample, secondClip, thirdSample, thirdClip, fourthClip, allClips1, allClips2, allClips3, allClips, driving2, park, firstIntake, homeMechanisms; //lineSearch, detectBeacon, distanceSensor, homeMechanisms,
    }

}
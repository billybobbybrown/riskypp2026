package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.driver.Teleop.MaxTicks;
import static org.firstinspires.ftc.teamcode.driver.Teleop.MinTicks;
import static org.firstinspires.ftc.teamcode.driver.Teleop.ticksPerDegree;
import static java.lang.Math.pow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class kachow extends SampleHardware {
    public Pose startTeleop;
    public Boolean isTurned = false;
    public Follower drive;
    public double DriveSpeed = .8;

    public void init(HardwareMap hardwareMap){
        if(kaze.robotPose == null){
            this.drive = Constants.createFollower(hardwareMap);
        } else if (kaze.robotPose != null){
            this.drive = Constants.createFollower(hardwareMap);
            drive.setPose(kaze.robotPose);
        }
    }
    public void robotCentric(double DriveSpeed, boolean opmodeIsActive, Gamepad gamepad1, Gamepad gamepad2, SampleHardware drive) {
        double FrontLeft;
        double FrontRight;
        double BackLeft;
        double BackRight;
        double x;
        double y;
        this.DriveSpeed = DriveSpeed;
        //double length = distance.getDistance(DistanceUnit.INCH);

        if (drive.runtime.seconds() > 100 && drive.runtime.seconds() < 115) {
            gamepad1.rumble(100);
            gamepad2.runLedEffect(cool);
            gamepad1.runLedEffect(cool);
        }
        if (opmodeIsActive) {
            boolean SlowModeIsOn = false;
            boolean MediumModeIsOn = false;
            if (gamepad1.right_trigger > 0) {

                SlowModeIsOn = true;
            }

            if (gamepad1.left_trigger > 0) {
                MediumModeIsOn = true;
            }

            if (MediumModeIsOn) {
                this.DriveSpeed = .24;
            }

            if (SlowModeIsOn && gamepad1.right_trigger == 0) {
                SlowModeIsOn = false;
            }

            if (SlowModeIsOn) {
                this.DriveSpeed = .15;
            }

            if (!SlowModeIsOn && !MediumModeIsOn) {
                this.DriveSpeed = DriveSpeed;
            }

           /* if (gamepad1.touchpad_finger_1) {
                x = gamepad1.touchpad_finger_1_x / 2;
                y = gamepad1.touchpad_finger_1_y / 2;
            } else {
                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
            }*/
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            if(isTurned){
                x= -x;
                y= -y;
            }
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double maxPower;

            FrontLeft = power * cos / max + turn;
            FrontRight = power * sin / max - turn;
            BackLeft = power * sin / max + turn;
            BackRight = power * cos / max - turn;

            maxPower = Math.max(Math.abs(FrontLeft), Math.abs(FrontRight));
            maxPower = Math.max(maxPower, Math.abs(BackLeft));
            maxPower = Math.max(maxPower, Math.abs(BackRight));

            if (maxPower > 1.0) {
                FrontLeft /= max;
                FrontRight /= max;
                BackLeft /= max;
                BackRight /= max;
            }
            drive.backleft.setPower(Range.clip(BackLeft, -this.DriveSpeed, this.DriveSpeed));
            drive.backright.setPower(Range.clip(BackRight, -this.DriveSpeed, this.DriveSpeed));
            drive.frontleft.setPower(Range.clip(FrontLeft, -this.DriveSpeed, this.DriveSpeed));
            drive.frontright.setPower(Range.clip(FrontRight, -this.DriveSpeed, this.DriveSpeed));
            kaze.update(this.drive);
        }

    }

    public void robotCentric(double DriveSpeed, boolean opmodeIsActive, Gamepad gamepad1, Gamepad gamepad2, RiskyHardware drive, boolean slowmode) {
        double FrontLeft;
        double FrontRight;
        double BackLeft;
        double BackRight;
        double x;
        double y;
        this.DriveSpeed = DriveSpeed;
        //double length = distance.getDistance(DistanceUnit.INCH);

        if (drive.runtime.seconds() > 100 && drive.runtime.seconds() < 115) {
            gamepad1.rumble(100);
            gamepad2.runLedEffect(cool);
            gamepad1.runLedEffect(cool);
        }
        if (opmodeIsActive) {
            if(slowmode){
                boolean SlowModeIsOn = false;
                boolean MediumModeIsOn = false;
                if (gamepad1.right_trigger > 0) {

                    SlowModeIsOn = true;
                }

                if (gamepad1.left_trigger > 0) {
                    MediumModeIsOn = true;
                }

                if (MediumModeIsOn) {
                    this.DriveSpeed = .24;
                }

                if (SlowModeIsOn && gamepad1.right_trigger == 0) {
                    SlowModeIsOn = false;
                }

                if (SlowModeIsOn) {
                    this.DriveSpeed = .15;
                }
            } else {
                this.DriveSpeed = DriveSpeed;
            }


           /* if (gamepad1.touchpad_finger_1) {
                x = gamepad1.touchpad_finger_1_x / 2;
                y = gamepad1.touchpad_finger_1_y / 2;
            } else {
                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
            }*/
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            if(isTurned){
                x= -x;
                y= -y;
            }
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double maxPower;

            FrontLeft = power * cos / max + turn;
            FrontRight = power * sin / max - turn;
            BackLeft = power * sin / max + turn;
            BackRight = power * cos / max - turn;

            maxPower = Math.max(Math.abs(FrontLeft), Math.abs(FrontRight));
            maxPower = Math.max(maxPower, Math.abs(BackLeft));
            maxPower = Math.max(maxPower, Math.abs(BackRight));

            if (maxPower > 1.0) {
                FrontLeft /= max;
                FrontRight /= max;
                BackLeft /= max;
                BackRight /= max;
            }
            drive.backleft.setPower(Range.clip(BackLeft, -this.DriveSpeed, this.DriveSpeed));
            drive.backright.setPower(Range.clip(BackRight, -this.DriveSpeed, this.DriveSpeed));
            drive.frontleft.setPower(Range.clip(FrontLeft, -this.DriveSpeed, this.DriveSpeed));
            drive.frontright.setPower(Range.clip(FrontRight, -this.DriveSpeed, this.DriveSpeed));
            kaze.update(this.drive);
        }

    }

    public void robotCentric(boolean opmodeIsActive, Gamepad gamepad1, Gamepad gamepad2, RiskyHardware drive, double drivespeed) {
        double FrontLeft;
        double FrontRight;
        double BackLeft;
        double BackRight;
        double x;
        double y;
        //double length = distance.getDistance(DistanceUnit.INCH);
        if (runtime.seconds() > 84.8 && runtime.seconds() < 85.2) {
            gamepad1.rumble(5000);
            gamepad2.rumble(5000);
        }
        if (runtime.seconds() > 109 && runtime.seconds() < 110) {
            gamepad1.rumble(10000);
            gamepad2.rumble(10000);
            gamepad2.runLedEffect(cool);
            gamepad1.runLedEffect(cool);
        }
        if (opmodeIsActive) {
            DriveSpeed = drivespeed;
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            //}
            if(isTurned){
                x= -x;
                y= -y;
            }
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));
            double maxPower;

            FrontLeft = power * cos / max + turn;
            FrontRight = power * sin / max - turn;
            BackLeft = power * sin / max + turn;
            BackRight = power * cos / max - turn;

            maxPower = Math.max(Math.abs(FrontLeft), Math.abs(FrontRight));
            maxPower = Math.max(maxPower, Math.abs(BackLeft));
            maxPower = Math.max(maxPower, Math.abs(BackRight));

            if (maxPower > 1.0) {
                FrontLeft /= max;
                FrontRight /= max;
                BackLeft /= max;
                BackRight /= max;
            }
            drive.backleft.setPower(Range.clip(BackLeft, -DriveSpeed, DriveSpeed));
            drive.backright.setPower(Range.clip(BackRight, -DriveSpeed, DriveSpeed));
            drive.frontleft.setPower(Range.clip(FrontLeft, -DriveSpeed, DriveSpeed));
            drive.frontright.setPower(Range.clip(FrontRight, -DriveSpeed, DriveSpeed));
            kaze.update(this.drive);
        }

    }


    public void fieldCentric(boolean opmodeIsActive, Gamepad gamepad1, Gamepad gamepad2, SampleHardware drive) {
        if (runtime.seconds() > 84.8 && runtime.seconds() < 85.2) {
            gamepad1.rumble(5000);
            gamepad2.rumble(5000);
        }
        if (runtime.seconds() > 109 && runtime.seconds() < 110) {
            gamepad1.rumble(10000);
            gamepad2.rumble(10000);
            gamepad2.runLedEffect(cool);
            gamepad1.runLedEffect(cool);
        }
        if (opmodeIsActive) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }
            if(isTurned){
                x= -x;
                y= -y;
            }
            double botHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;


            drive.frontright.setPower(frontRightPower);
            drive.frontleft.setPower(frontLeftPower);
            drive.backleft.setPower(backLeftPower);
            drive.backright.setPower(backRightPower);
            kaze.update(this.drive);
        }
    }

    public static double angleDifference2way(double initialAngle, double targetAngle) {
        double error;
        boolean turnLeft = Math.abs(Math.toDegrees(initialAngle)) <= Math.abs(Math.toDegrees(targetAngle));
        double difference = Math.toDegrees((targetAngle - initialAngle) % 360);

        if (difference < -180) {
            difference += 360;
        } else if (difference > 179) {
            difference -= 360;
        }
        error = difference;
        return error;
    }
    public static double angleDifference1way(double initialAngle, double targetAngle) {
        double error;
        boolean turnLeft = Math.abs(Math.toDegrees(initialAngle)) <= Math.abs(Math.toDegrees(targetAngle));
        double difference = Math.toDegrees((targetAngle - initialAngle) % 360);

        if (difference < 0) {
            difference += 360;
        } else if (difference > 360) {
            difference -= 360;
        }
        error = difference;
        return error;
    }



    //turns 180 and reverses the controls for a smooth and quick transition
    public void rotate(double initialHeading, Gamepad gamepad1, Gamepad gamepad2, SampleHardware drive) {
        double error;
        double errorPower;
        double x;
        double y;
        double turn;
        double heading;
        double targetHeading = Math.toRadians(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180);
        while (true) {
            if (drive.runtime.seconds() > 84.8 && drive.runtime.seconds() < 85.2) {
                gamepad1.rumble(5000);
                gamepad2.rumble(5000);
            }
            if (drive.runtime.seconds() > 109 && drive.runtime.seconds() < 110) {
                gamepad1.rumble(10000);
                gamepad2.rumble(10000);
                gamepad2.runLedEffect(cool);
                gamepad1.runLedEffect(cool);
            }
            heading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;

            error = angleDifference2way(heading, targetHeading + Math.toRadians(2));
            errorPower = (error / 180) + gamepad1.right_stick_x;
            if (gamepad1.right_stick_x < -.8) {
                break;
            }
            if (Math.abs(errorPower) < .4) {
                errorPower = (errorPower / Math.abs(errorPower)) * .4;
            }
            errorPower = errorPower - gamepad1.right_stick_x;
            if (Math.abs(errorPower) > 1) {
                errorPower = (errorPower / Math.abs(errorPower)) * 1;
            }
            turn = -errorPower;
            if (Math.abs(error) < 2) {
                isTurned = !isTurned;
                double startTime = drive.runtime.seconds();
                //pause for .1 second before turning back if adjustment is needed
                while (startTime + .1 > drive.runtime.seconds() && (Math.abs(gamepad1.left_stick_y) < .37) && (Math.abs(gamepad1.left_stick_x) < .37)) {
                    drive_pause(drive);
                }
                while (Math.abs(gamepad1.left_stick_y) < .37 && Math.abs(gamepad1.left_stick_x) < .37) {
                    if (drive.runtime.seconds() > 84.8 && drive.runtime.seconds() < 85.2) {
                        gamepad1.rumble(5000);
                        gamepad2.rumble(5000);
                    }
                    if (drive.runtime.seconds() > 109 && drive.runtime.seconds() < 110) {
                        gamepad1.rumble(10000);
                        gamepad2.rumble(10000);
                        gamepad2.runLedEffect(cool);
                        gamepad1.runLedEffect(cool);
                    }
                    heading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    x = -gamepad1.left_stick_x;
                    y = gamepad1.left_stick_y;

                    error = angleDifference2way(heading, targetHeading);
                    errorPower = (error / 180) + gamepad1.right_stick_x;
                    if (Math.abs(errorPower) < .2) {
                        errorPower = (errorPower / Math.abs(errorPower)) * .2;
                    }
                    errorPower = errorPower - gamepad1.right_stick_x;
                    if (Math.abs(errorPower) > 1) {
                        errorPower = (errorPower / Math.abs(errorPower)) * 1;
                    }
                    turn = -errorPower;
                    if (Math.abs(error) < .5) {
                        break;
                    }
                    if (isTurned) {
                        initialHeading = 180;
                    }
                    double botHeading = Math.toRadians(initialHeading - error);
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
                    double frontLeftPower = (rotY + rotX + turn) / denominator;
                    double backLeftPower = (rotY - rotX + turn) / denominator;
                    double frontRightPower = (rotY - rotX - turn) / denominator;
                    double backRightPower = (rotY + rotX - turn) / denominator;


                    drive.frontright.setPower(frontRightPower);
                    drive.frontleft.setPower(frontLeftPower);
                    drive.backleft.setPower(backLeftPower);
                    drive.backright.setPower(backRightPower);
                } //turn back in the other direction for accuracy
                break;
            } // break out when target is reached
            if (isTurned) { // used to keep fieldccentric control while turning
                initialHeading = 180;
            }
            double botHeading = Math.toRadians(initialHeading - error);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;


            drive.frontright.setPower(frontRightPower);
            drive.frontleft.setPower(frontLeftPower);
            drive.backleft.setPower(backLeftPower);
            drive.backright.setPower(backRightPower);
            kaze.update(this.drive);
        }
    }

    //takes the shortest path to a specified heading while keeping xy controls
    public void rotateTo(double targetHeading, Gamepad gamepad1, Gamepad gamepad2, SampleHardware drive, double minSpeed) {
        double error;
        double errorPower;
        double x;
        double y;
        double turn;
        double heading;

        while (true) {
            if (drive.runtime.seconds() > 84.8 && drive.runtime.seconds() < 85.2) {
                gamepad1.rumble(5000);
                gamepad2.rumble(5000);
            }
            if (drive.runtime.seconds() > 109 && drive.runtime.seconds() < 110) {
                gamepad1.rumble(10000);
                gamepad2.rumble(10000);
                gamepad2.runLedEffect(cool);
                gamepad1.runLedEffect(cool);
            }
            heading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            x = 0;
            y = 0;

            error = angleDifference2way(heading, targetHeading);
            errorPower = error / 100;
            if(Math.abs(errorPower)>1){
                errorPower = Math.abs(errorPower)/errorPower;
            }
            if(Math.abs(errorPower)<minSpeed){
                errorPower = (Math.abs(errorPower)/errorPower)*minSpeed;
            }
            turn = -errorPower;
            if (Math.abs(error) < 2) {
                break;
            }

            double botHeading = Math.toRadians(-error);
            double rotX = x*Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x*Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) +Math.abs(turn), 1);
            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;



            drive.frontright.setPower(frontRightPower);
            drive.frontleft.setPower(frontLeftPower);
            drive.backleft.setPower(backLeftPower);
            drive.backright.setPower(backRightPower);
            kaze.update(this.drive);
        }
    }

    public void drive_pause(SampleHardware robot){
        robot.backleft.setPower(0);
        robot.backright.setPower(0);
        robot.frontright.setPower(0);
        robot.frontleft.setPower(0);
        kaze.update(drive);
    }

    public static class cameraPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum detection
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar GREEN = new Scalar(0, 255, 0);
        final Scalar WHITE = new Scalar(255, 255, 255);

        /*
         * The core values which define the location and size of the sample regions
         */
        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,98);
        final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(214,98);
        final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(427,98);
        static final int REGION_WIDTH = 200;
        static final int REGION_HEIGHT = 200;

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
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile detection position = detection.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
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
                position = detection.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        WHITE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = detection.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        WHITE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = detection.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        WHITE, // The color the rectangle is drawn in
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
        public detection getAnalysis()
        {
            return position;
        }
    }
    public void rrdrive(Gamepad gamepad1, Gamepad gamepad2, double DriveSpeed, SampleHardware robot){
        double FrontLeft;
        double FrontRight;
        double BackLeft;
        double BackRight;
        double x;
        double y;
        this.DriveSpeed = DriveSpeed;
        //double length = distance.getDistance(DistanceUnit.INCH);
        if (runtime.seconds() > 84.8 && runtime.seconds() < 85.2) {
            gamepad1.rumble(5000);
            gamepad2.rumble(5000);
        }
        if (runtime.seconds() > 109 && runtime.seconds() < 110) {
            gamepad1.rumble(10000);
            gamepad2.rumble(10000);
            gamepad2.runLedEffect(cool);
            gamepad1.runLedEffect(cool);
        }
        if (true) {
            boolean SlowModeIsOn = false;
            boolean MediumModeIsOn = false;
            if (gamepad1.right_trigger > 0) {

                SlowModeIsOn = true;
            }

            if (gamepad1.left_trigger > 0) {
                MediumModeIsOn = true;
            }

            if (MediumModeIsOn) {
                this.DriveSpeed = .24;
            }

            if (SlowModeIsOn && gamepad1.right_trigger == 0) {
                SlowModeIsOn = false;
            }

            if (SlowModeIsOn) {
                this.DriveSpeed = .15;
            }

            if (!SlowModeIsOn && !MediumModeIsOn) {
                this.DriveSpeed = DriveSpeed;
            }


            y = -gamepad1.left_stick_x * this.DriveSpeed;
            x = -gamepad1.left_stick_y * this.DriveSpeed;
            double turn = -gamepad1.right_stick_x * this.DriveSpeed;
            if(isTurned){
                x= -x;
                y= -y;
            }
            drive.setTeleOpDrive(x,y,turn,true);
            //roadRunner.pose = new Pose2d(roadRunner.pose.position.x, roadRunner.pose.position.y, Math.toRadians(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+kaze.headingOffset));
            kaze.update(drive);
        }
    }

    public double bot_to_target;
    public double bot_to_wall;
    public double wall_to_target;
    public double bot_angle;
    public double wall_angle;
    public double target_angle;
    public double error;

    public void aimbot(Vector2d target, Gamepad gamepad1, Gamepad gamepad2, RiskyHardware drive, double minSpeed) {
        double errorPower;
        double x;
        double y;
        double turn;
        double heading;


        Vector2d botPoint = new Vector2d(drive.kachow.drive.getPose().getX(), drive.kachow.drive.getPose().getY());
        Vector2d wallReference = new Vector2d(144, botPoint.y);
        bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));
        bot_to_wall = Math.sqrt(Math.abs(pow(botPoint.x-wallReference.x, 2) + pow(botPoint.y-wallReference.y, 2)));
        wall_to_target = Math.sqrt(Math.abs(pow(wallReference.x-target.x, 2) + pow(wallReference.y-target.y, 2)));

        bot_angle = Math.abs(Math.acos((pow(bot_to_wall,2) + pow(bot_to_target,2) - pow(wall_to_target,2))/
                (2 * bot_to_wall * bot_to_target)));
        target_angle = Math.abs(Math.asin((bot_to_wall * Math.sin(bot_angle))/wall_to_target));
        wall_angle = Math.abs(Math.toRadians(180) - bot_angle - target_angle);

        if(botPoint.y > target.y){
            bot_angle = -bot_angle;
            target_angle = -target_angle;
            wall_angle = -wall_angle;
        }

        if (drive.runtime.seconds() > 100 && drive.runtime.seconds() < 115) {
            gamepad1.rumble(100);
            gamepad2.runLedEffect(cool);
            gamepad1.runLedEffect(cool);
        }
        heading = drive.kachow.drive.getHeading() ;

        x = -gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;

        error = angleDifference2way(heading, bot_angle);

        errorPower = error / 90;
        if(Math.abs(errorPower)>1){
            errorPower = Math.abs(errorPower)/errorPower;
        }
        if(Math.abs(errorPower)<minSpeed){
            errorPower = (Math.abs(errorPower)/errorPower)*minSpeed;
        }
        if (Math.abs(error) < 1) {
            errorPower = 0;
        }
        turn = -errorPower;


        double botHeading = heading;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;


        drive.frontright.setPower(frontRightPower);
        drive.frontleft.setPower(frontLeftPower);
        drive.backleft.setPower(backLeftPower);
        drive.backright.setPower(backRightPower);
        kaze.update(drive.kachow.drive);

    }


    public void aimTurt(Vector2d target,  Gamepad gamepad1, Gamepad gamepad2, RiskyHardware drive, double speed) {
        double errorPower;
        double x;
        double y;
        double turn;
        double heading;


        Vector2d botPoint = new Vector2d(drive.kachow.drive.getPose().getX(), drive.kachow.drive.getPose().getY());
        Vector2d wallReference = new Vector2d(144, botPoint.y);
        bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));
        bot_to_wall = Math.sqrt(Math.abs(pow(botPoint.x-wallReference.x, 2) + pow(botPoint.y-wallReference.y, 2)));
        wall_to_target = Math.sqrt(Math.abs(pow(wallReference.x-target.x, 2) + pow(wallReference.y-target.y, 2)));

        bot_angle = Math.abs(Math.acos((pow(bot_to_wall,2) + pow(bot_to_target,2) - pow(wall_to_target,2))/
                (2 * bot_to_wall * bot_to_target)));
        target_angle = Math.abs(Math.asin((bot_to_wall * Math.sin(bot_angle))/wall_to_target));
        wall_angle = Math.abs(Math.toRadians(180) - bot_angle - target_angle);

        if(botPoint.y > target.y){
            bot_angle = -bot_angle;
            target_angle = -target_angle;
            wall_angle = -wall_angle;
        }

        heading = drive.kachow.drive.getHeading();
        error = angleDifference1way(heading, bot_angle);

        if (((int) (error * (ticksPerDegree))) > MaxTicks){
            drive.turt.setTargetPosition(MaxTicks);
            gamepad1.rumble(75);
            gamepad2.rumble(75);

        } else if (((int) (error * (ticksPerDegree))) < MinTicks){
            drive.turt.setTargetPosition(MinTicks);
            gamepad1.rumble(75);
            gamepad2.rumble(75);

        } else {
            drive.turt.setTargetPosition((int) (error * (ticksPerDegree)));

            }
        drive.turt.setPower(speed);
        drive.turt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



}
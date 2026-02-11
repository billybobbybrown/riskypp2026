
package org.firstinspires.ftc.teamcode.driver;

//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RiskyHardware;
import org.firstinspires.ftc.teamcode.hardware.RiskyHardware.state;
import org.firstinspires.ftc.teamcode.hardware.Vector2d;
import org.firstinspires.ftc.teamcode.hardware.button;
import org.firstinspires.ftc.teamcode.hardware.kaze;

@Configurable
@TeleOp(name="Teleop", group="6892")
//@Disabled
public class Teleop extends LinearOpMode {

    public static double distance;
    public static double shooterVelocity;
    public static double shooterFar = 1960;
    public static double shooterClose = 1300;
    public static double ticksPerDegree = 1.45;
    public static int MaxTicks = 410;
    public static int MinTicks = 0;
    public static double velocityMultiplier = -.75;
    public static boolean canShoot = true;
    public static boolean isReset = false;




    button.ButtonReader gamePad1 = new button.ButtonReader();
    button.ButtonReader gamePad2 = new button.ButtonReader();
    /* Declare OpMode members. */
    RiskyHardware robot = new RiskyHardware();
    state State = state.intaking;

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        kaze.init(new Pose(72,72,0));//kaze init before robotinit
        robot.init(hardwareMap);
        gamepad2.runLedEffect(robot.redled);
        gamepad1.runLedEffect(robot.blueled);
        telemetry.update();
        robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        if(kaze.robotPose == null){
            kaze.init(new Pose(72,72,0));//kaze init before robotinit
        } else {
            robot.kachow.drive.setPose(kaze.robotPose);
        }
        waitForStart();
        Vector2d target;
        robot.runtime.reset();
        if(kaze.target != null){
            target = kaze.target;
        } else {
            target = new Vector2d(3,144);
        }

        robot.spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        robot.spinner2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        runtime.reset();






        while (opModeIsActive()) {
            update();

            robot.kachow.drive.updatePose();

            telemetry.addLine("x: " + robot.kachow.drive.getPose().getX());
            telemetry.addLine("y: " + robot.kachow.drive.getPose().getY());
            telemetry.addLine("heading: " + Math.toDegrees(robot.kachow.drive.getPose().getHeading()));


            telemetry.addData( "velocity X: ",robot.kachow.drive.getVelocity().getXComponent());
            telemetry.addData( "velocity Y: ",robot.kachow.drive.getVelocity().getYComponent());
            telemetry.addData( "velocity R: ",robot.kachow.drive.getAngularVelocity());

            telemetry.addData("turt: ", robot.turt.getCurrentPosition());
            telemetry.addData("targetX: ", target.x);
            telemetry.addData("targetY: ", target.y);
            Vector2d botPoint = new Vector2d(robot.kachow.drive.getPose().getX(), robot.kachow.drive.getPose().getY());
            robot.kachow.bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));

            if(robot.kachow.bot_to_target >= 160){
                shooterVelocity = 2000;
            } else if (robot.kachow.bot_to_target >= 140){
                shooterVelocity = 1840;
            } else if (robot.kachow.bot_to_target >= 120){
                shooterVelocity = 1700;
            } else if (robot.kachow.bot_to_target >= 100){
                shooterVelocity = 1560;
            } else if (robot.kachow.bot_to_target >= 90){
                shooterVelocity = 1500;
            } else if (robot.kachow.bot_to_target >= 80){
                shooterVelocity = 1460;
            } else if (robot.kachow.bot_to_target >= 70){
                shooterVelocity = 1340;
            } else if (robot.kachow.bot_to_target >= 60){
                shooterVelocity = 1280;
            } else if (robot.kachow.bot_to_target >= 50){
                shooterVelocity = 1300;
            }


            /*if(kaze.IsBlue){
                if(botPoint.y <= 60){
                    target = new Vector2d(3 + (robot.kachow.drive.getVelocity().getXComponent() * velocityMultiplier)
                            , 144+ (robot.kachow.drive.getVelocity().getYComponent() * velocityMultiplier));
                } else {
                    target = new Vector2d(0 + (robot.kachow.drive.getVelocity().getXComponent() * velocityMultiplier)
                            , 141+ (robot.kachow.drive.getVelocity().getYComponent() * velocityMultiplier));
                }
            } else {
                if(botPoint.y <= 60){
                    target = new Vector2d(141 + (robot.kachow.drive.getVelocity().getXComponent() * velocityMultiplier)
                            , 144+ (robot.kachow.drive.getVelocity().getYComponent() * velocityMultiplier));
                } else {
                    target = new Vector2d(144 + (robot.kachow.drive.getVelocity().getXComponent() * velocityMultiplier)
                            , 141+ (robot.kachow.drive.getVelocity().getYComponent() * velocityMultiplier));
                }
            }


             */
            if(kaze.IsBlue){
                if(botPoint.y <= 60){
                    target = new Vector2d(0 + (robot.kachow.drive.getVelocity().getXComponent() * velocityMultiplier)
                            , 144+ (robot.kachow.drive.getVelocity().getYComponent() * velocityMultiplier));
                } else {
                    target = new Vector2d(0 + (robot.kachow.drive.getVelocity().getXComponent() * velocityMultiplier)
                            , 144+ (robot.kachow.drive.getVelocity().getYComponent() * velocityMultiplier));
                }
            } else {
                if(botPoint.y <= 60){
                    target = new Vector2d(144 + (robot.kachow.drive.getVelocity().getXComponent() * velocityMultiplier)
                            , 144+ (robot.kachow.drive.getVelocity().getYComponent() * velocityMultiplier));
                } else {
                    target = new Vector2d(144 + (robot.kachow.drive.getVelocity().getXComponent() * velocityMultiplier)
                            , 144+ (robot.kachow.drive.getVelocity().getYComponent() * velocityMultiplier));
                }
            }

            if(gamePad1.Dpad_Down.isDown()){
                robot.leftLift.setPower(1);
                robot.rightLift.setPower(1);
            } else if(gamePad1.Dpad_Up.isDown()){
                robot.leftLift.setPower(-1);
                robot.rightLift.setPower(-1);
            } else if(gamePad1.Dpad_Left.isDown()){
                robot.leftLift.setPower(-1);
                robot.rightLift.setPower(0);
            } else if(gamePad1.Dpad_Right.isDown()){
                robot.leftLift.setPower(0);
                robot.rightLift.setPower(-1);
            } else{
                robot.leftLift.setPower(0);
                robot.rightLift.setPower(0);
            }



            switch (State) {
                case intaking:
                    telemetry.addData("mode: ", robot.turt.getMode());
                    if(robot.stateChanged){
                        robot.turt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.turt.setPower(-.5);
                    }
                    if(isReset){
                        robot.kachow.aimTurt(target, gamepad1, gamepad2, robot, 1);
                        robot.kachow.robotCentric(opModeIsActive(), gamepad1, gamepad2, robot, 1);
                    } else if (robot.magnet.isPressed()){
                        robot.turt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.turt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.turt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        isReset = true;
                    }
                    robot.spinner.setVelocity(shooterVelocity);
                    robot.spinner2.setVelocity(shooterVelocity);
                    //robot.aimer.setPosition(aimerPose);
                    if (gamePad1.Right_Trigger.isDown()) {
                        robot.intake.setPower(-1);
                        robot.intake3.setPower(0);

                    } else if (gamePad1.Left_Trigger.isDown()) {
                        robot.intake.setPower(1);
                        robot.intake3.setPower(0);

                    } else if ((!gamePad1.Right_Trigger.isDown()) && (!gamePad1.Left_Trigger.isDown())) {
                        robot.intake.setPower(0);
                        robot.intake3.setPower(0);

                    }


                    telemetry.addData("Intake: ", robot.intake.getVelocity());
                    telemetry.addData("IntakePower: ", robot.intake.getPower());
                    if (gamePad1.Right_Bumper.wasJustPressed()) {
                        changeStateTo(state.aimbot);
                        isReset = false;
                    }
                    break;

                case aimbot:
                    robot.kachow.aimTurt(target, gamepad1, gamepad2, robot, 1);
                    robot.kachow.robotCentric(opModeIsActive(), gamepad1, gamepad2, robot, .5);
                    robot.spinner.setVelocity(shooterVelocity);
                    robot.spinner2.setVelocity(shooterVelocity);
                    if (((int) (robot.kachow.error * (ticksPerDegree))) > MaxTicks){
                        gamepad1.rumble(75);
                        gamepad2.rumble(75);
                        canShoot = false;
                    } else if (((int) (robot.kachow.error * (ticksPerDegree))) < MinTicks){
                        gamepad1.rumble(75);
                        gamepad2.rumble(75);
                        canShoot = false;
                    } else {
                        canShoot = true;
                    }

                    if((robot.spinner.getVelocity() >= shooterVelocity-20) && canShoot){
                        if (gamePad1.Right_Bumper.isDown()) {
                            robot.intake.setPower(-1);
                            robot.intake3.setPower(-1);
                        }
                    }

                    if(shooterVelocity == 0){
                        robot.spinner.setPower(0);
                        robot.spinner2.setPower(0);
                    }





                    telemetry.addData("spinner: ", robot.spinner.getVelocity());
                    telemetry.addData("spinnerPower: ", robot.spinner.getPower());
                    telemetry.addData("TargetVelocity: ", shooterVelocity);

                    telemetry.addData("spinner2: ", robot.spinner2.getVelocity());
                telemetry.addData("spinner2Power: ", robot.spinner2.getPower());
                telemetry.addData("TargetVelocity: ", shooterVelocity);



                if(gamePad1.Right_Bumper.wasJustReleased()){
                        changeStateTo(state.intaking);
                        robot.intake3.setPower(0);
                        robot.intake.setPower(-1);

                }
                    break;

            }








        }
    }

    public void robot_robotCentric(double DriveSpeed, boolean slow) {
        double FrontLeft;
        double FrontRight;
        double BackLeft;
        double BackRight;
        telemetry.addLine(String.valueOf(distance));
        telemetry.addData(">", runtime.seconds());
        telemetry.update();
        if(runtime.seconds() > 84.8 && runtime.seconds() < 85.2){
            gamepad1.rumble(5000);
            gamepad2.rumble(5000);
        }
        if(runtime.seconds() > 109 && runtime.seconds() < 110){
            gamepad1.rumble(10000);
            gamepad2.rumble(10000);
        }
        //StrafeRight
        if (opModeIsActive()) {
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            FrontLeft = power * cos/max + turn;
            FrontRight = power * sin/max - turn;
            BackLeft = power * sin/max + turn;
            BackRight = power * cos/max - turn;

            if ((power + Math.abs(turn)) > 1){
                FrontLeft /= power + turn;
                FrontRight /= power + turn;
                BackLeft /= power + turn;
                BackRight /= power + turn;
            }

            robot.backleft.setPower(Range.clip(BackLeft, -DriveSpeed, DriveSpeed));
            robot.backright.setPower(Range.clip(BackRight, -DriveSpeed, DriveSpeed));
            robot.frontleft.setPower(Range.clip(FrontLeft, -DriveSpeed, DriveSpeed));
            robot.frontright.setPower(Range.clip(FrontRight, -DriveSpeed, DriveSpeed));
        }

    }
    public void update(){ //place once on start of loop
        robot.stateUpdate(State);
        if(robot.magnet.isPressed()){
            robot.turt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        gamePad2.update(gamepad2, robot);
        gamePad1.update(gamepad1, robot);
        if(gamepad1.share && gamepad1.options){
            robot.kachow.drive.setPose(new Pose(72, 72, 0));
        }
        kaze.update(robot.kachow.drive);
        telemetry.addData("state: ", State);
        telemetry.update();
        kaze.drawCurrentAndHistory(robot.kachow.drive);
    }
    public void changeStateTo(state tostate){
        State = tostate;
    }


}
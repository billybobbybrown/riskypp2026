
package org.firstinspires.ftc.teamcode.driver;

//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import static org.firstinspires.ftc.teamcode.driver.Teleop.ticksPerDegree;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RiskyHardware;
import org.firstinspires.ftc.teamcode.hardware.RiskyHardware.state;
import org.firstinspires.ftc.teamcode.hardware.Vector2d;
import org.firstinspires.ftc.teamcode.hardware.button;
import org.firstinspires.ftc.teamcode.hardware.kaze;

@Configurable
@TeleOp(name="Test", group="4848")
//@Disabled
public class Calibration extends LinearOpMode {

    public static double distance;
    public static double shooterVelocity = 0;
    public static double intakeVelocity = 0;
    public static double difference = 0;
    public static double P = 30;//300;

    public static double I = 0;
    public static double D = 5;//10;
    public static double F = 5;//20;

    public static double power = 1;//20;
    public static double deflectorLeftIn;
    public static double deflectorRightIn;
    public static double deflectorMiddle;
    public static double aimerClose;
    public static double aimerMid;
    public static double aimerFar;
    public static double aimerMin = .25;
    public static double aimerMax = .75;
    public static double shooterFar;
    public static double shooterClose;
    public static double shooterMid;
    public static double intakeFast;
    public static double intakeSlow;
    public static double leftFeederDown;
    public static double leftFeederMid;
    public static double leftFeederUp;
    public static double rightFeederDown;
    public static double rightFeederMid;
    public static double rightFeederUp;
    public static int aimerPose;
    public static int target1 = 0;
    public static int target2 = 262;






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
        waitForStart();
        robot.turt.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30, 0, 30, 10));
        runtime.reset();
        Vector2d target = new Vector2d(0,144);



        aimerPose = 0;



        while (opModeIsActive()) {
            update();

            robot.kachow.aimTurt(target, gamepad1, gamepad2, robot, 1);
            robot.kachow.robotCentric(opModeIsActive(), gamepad1, gamepad2, robot, .5);



            robot.turt.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

            robot.kachow.drive.updatePose();
            Vector2d botPoint = new Vector2d(robot.kachow.drive.getPose().getX(), robot.kachow.drive.getPose().getY());
            robot.kachow.bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));

            telemetry.addLine("Magnet: " + robot.magnet.isPressed());

            telemetry.addLine("x: " + robot.kachow.drive.getPose().getX());
            telemetry.addLine("y: " + robot.kachow.drive.getPose().getY());

            telemetry.addData( "velocity X: ",robot.kachow.drive.getVelocity().getXComponent());
            telemetry.addData( "velocity Y: ",robot.kachow.drive.getVelocity().getYComponent());

            telemetry.addData("turt: ", robot.turt.getCurrentPosition());
            telemetry.addData("turt target: ", aimerPose);
            telemetry.addData("turt angle: ", robot.turt.getCurrentPosition() / ticksPerDegree);
            //test to see if you need to constantly refresh the run to position, or if it only needs to be mentioned once
            //quickly tap any button that moves it and se if it reaches the full position or turns off when you let go
            //test to see if the position is inately stored after switching to and from differnt codes
            //initialize, check position, go to autonomous then come back and see if its still synced and if movement from autonomous translated to teleop



            if(gamePad2.TouchPad.isDown()){
                robot.turt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamePad2.triangle.isDown()){
                robot.turt.setPower(power);
                robot.turt.setTargetPosition(aimerPose);
                robot.turt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamePad2.x.isDown()){
                robot.turt.setPower(0);
            }
            if(gamePad2.Dpad_Up.isDown()){
                aimerPose = aimerPose+5;
            }
            if(gamePad2.Dpad_Down.isDown()){
                aimerPose = aimerPose-5;
            }
            if(gamePad2.Right_Bumper.isDown()){
                robot.turt.setPower(power);
                robot.turt.setTargetPosition(target1);
                robot.turt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamePad2.Left_Bumper.isDown()){
                robot.turt.setPower(power);
                robot.turt.setTargetPosition(target2);
                robot.turt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamePad2.Right_Trigger.isDown()){
                robot.intake.setVelocity(robot.intake.getVelocity()+5);
            }
            if(gamePad2.Left_Trigger.isDown()){
                robot.intake.setVelocity(robot.intake.getVelocity()-5);
            }








        }
    }

    public void update(){ //place once on start of loop
        gamePad2.update(gamepad2, robot);
        gamePad1.update(gamepad1, robot);
        if(gamepad1.share && gamepad1.options){
        }
//        kaze.update(robot.kachow.roadRunner);
        telemetry.update();
        kaze.drawCurrentAndHistory(robot.kachow.drive);
    }
    public void changeStateTo(state tostate){
        State = tostate;
        robot.stateUpdate(State);
    }


}
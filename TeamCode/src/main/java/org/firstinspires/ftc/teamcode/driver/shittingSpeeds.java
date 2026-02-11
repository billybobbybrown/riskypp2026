
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

import org.firstinspires.ftc.teamcode.hardware.RiskyHardware;
import org.firstinspires.ftc.teamcode.hardware.RiskyHardware.state;
import org.firstinspires.ftc.teamcode.hardware.Vector2d;
import org.firstinspires.ftc.teamcode.hardware.button;
import org.firstinspires.ftc.teamcode.hardware.kaze;

@Configurable
@TeleOp(name="shitting speeds", group="4848")
//@Disabled
public class shittingSpeeds extends LinearOpMode {

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

    public static int velocity = 1000;






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
        Vector2d target = new Vector2d(144,144);



        aimerPose = 0;



        while (opModeIsActive()) {
            update();

            robot.kachow.aimTurt(target, gamepad1, gamepad2, robot, 1);
            robot.kachow.robotCentric(opModeIsActive(), gamepad1, gamepad2, robot, .5);



            robot.turt.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));

            robot.kachow.drive.updatePose();
            Vector2d botPoint = new Vector2d(robot.kachow.drive.getPose().getX(), robot.kachow.drive.getPose().getY());
            robot.kachow.bot_to_target = Math.sqrt(Math.abs((botPoint.x-target.x)*(botPoint.x-target.x) + (botPoint.y-target.y)*(botPoint.y-target.y)));

            telemetry.addLine("distance: " + robot.kachow.bot_to_target);
            telemetry.addLine("Velocity: " + robot.spinner.getVelocity());
            telemetry.addLine("Target Velocity: " + velocity);



            robot.spinner.setVelocity(velocity);
            robot.spinner2.setVelocity(velocity);

            if(gamePad1.Dpad_Down.wasJustPressed()){
                velocity = velocity-20;
            }
            if(gamePad1.Dpad_Up.wasJustPressed()){
                velocity = velocity+20;
            }


            if(gamePad1.Right_Trigger.isDown()){
                robot.intake.setPower(-1);
                robot.intake3.setPower(-1);
            }
            if(gamePad1.Left_Trigger.isDown()){
                robot.intake.setPower(1);
                robot.intake3.setPower(1);
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
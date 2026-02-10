package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.auto.paths.REDStartPoseToShootPose;
import static org.firstinspires.ftc.teamcode.auto.paths.REDShootPoseToStartPointPose;
import static org.firstinspires.ftc.teamcode.auto.paths.REDStartPoseToIntakePose;
import static org.firstinspires.ftc.teamcode.auto.paths.REDIntakePoseToShootPose;
import static org.firstinspires.ftc.teamcode.auto.paths.REDStartPose2;
import static org.firstinspires.ftc.teamcode.auto.paths.REDIntakePose2;
import static org.firstinspires.ftc.teamcode.auto.paths.REDPerChance1;
import static org.firstinspires.ftc.teamcode.auto.paths.REDLastBall;
import static org.firstinspires.ftc.teamcode.auto.paths.REDBald;
import static org.firstinspires.ftc.teamcode.auto.paths.REDBalls;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.RiskyHardware;
import org.firstinspires.ftc.teamcode.hardware.RiskyHardware.state;

import org.firstinspires.ftc.teamcode.hardware.kaze;

@Autonomous(name = "RedAuto", group = "4848")
public final class RedAuto extends LinearOpMode {

    boolean wasMade = false;
    boolean isFirst = true;
    boolean firstScored = false;
    boolean secondScored = false;
    boolean thirdScored = false;
    boolean purple1 = false;
    boolean purple2 = false;
    boolean green = false;
    double waitTime = 1;
    double startTime;
    String pattern = "PPG";
    state State = state.idle;

    RiskyHardware robot = new RiskyHardware();
    double angle = 0;
    LLResult result = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setMsTransmissionInterval(11);

        final Pose startPose = new Pose(144-25.121, 127.402, Math.toRadians(180-136));
        kaze.init(startPose);
        paths actions = new paths(robot);
        robot.init(hardwareMap);

        robot.imu.resetYaw();

        while (!isStarted() && !isStopRequested()) {
            update();
            actions.buildPaths();
            gamepad2.runLedEffect(robot.redled);
            gamepad1.runLedEffect(robot.blueled);
            telemetry.update();
            robot.pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            robot.drive.setPose(startPose);
        }

        robot.spinner.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        robot.spinner2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        while (opModeIsActive()) {
            robot.spinner.setVelocity(1250);
            robot.spinner2.setVelocity(1250);
            update();

            switch (State){

                case idle:
                    robot.spinner.setVelocity(0);

                    if(!robot.drive.isBusy()) {
                        robot.drive.followPath(REDStartPoseToShootPose, true);
                        changeStateTo(state.StartPoseToShootPose);
                    }
                    break;

                case StartPoseToShootPose:
                    if(!robot.drive.isBusy()){
                        robot.intake.setPower(.2);

                        robot.spanker.setPosition(1);
                        changeStateTo(state.shooting1);
                    }
                    break;

                case shooting1:
                    if(robot.stateTime.seconds() >= 4){
                        robot.spanker.setPosition(.65);
                        robot.intake.setPower(0);
                        changeStateTo(state.ShootPosetoIntake1);
                        robot.drive.followPath(REDShootPoseToStartPointPose,true);
                        robot.intake.setPower(.6);
                    }
                    break;

                case ShootPosetoIntake1:
                    if(!robot.drive.isBusy()){
                        changeStateTo(state.Intake1toGreenBall);
                        robot.drive.followPath(REDStartPoseToIntakePose,true);
                    }
                    break;

                case Intake1toGreenBall:
                    if(!robot.drive.isBusy()){
                        robot.intake.setPower(0);
                        changeStateTo(state.shooting2);
                        robot.drive.followPath(REDStartPoseToShootPose, true);
                    }
                    break;

                case shooting2:
                    if(robot.stateChanged){
                        telemetry.addLine("Shooting second set");
                        telemetry.update();
                        robot.stateTime.reset();
                    }


                    if(robot.stateTime.seconds() > 2){
                        // fire
                        robot.intake.setPower(.2);
                        robot.spanker.setPosition(1);
                    }

                    // let it shoot for 1 second
                    if(robot.stateTime.seconds() > 4.5){
                        // stop firing
                        robot.intake.setPower(0);
                        robot.spanker.setPosition(.65);
                        if(!robot.drive.isBusy()){
                            changeStateTo(state.ShootPosetoIntake3);
                        }
                        // move on to the next state
                    }
                    break;

                // ----------------------- FIXED SECTION START -----------------------

                case ShootPosetoIntake3:
                    if(robot.stateChanged){
                        robot.drive.followPath(REDStartPose2, true);
                        robot.spanker.setPosition(.65);
                    }
                    if(!robot.drive.isBusy()){
                        changeStateTo(state.Intake3toGreenBall);
                    }
                    break;

                case Intake3toGreenBall:
                    if(robot.stateChanged){
                        robot.intake.setPower(.9);
                        robot.drive.followPath(REDIntakePose2, true);
                    }
                    if(!robot.drive.isBusy()){
                        changeStateTo(state.GoBackFool);
                    }
                    break;

                case GoBackFool:
                    if(robot.stateChanged){
                        robot.drive.followPath(REDPerChance1, true);
                    }
                    if(!robot.drive.isBusy()){
                        changeStateTo(state.ShootingPose3);
                    }
                    break;

                case ShootingPose3:
                    if(robot.stateChanged){
                        robot.intake.setPower(0);
                        robot.spanker.setPosition(.65);
                        robot.drive.followPath(REDStartPoseToShootPose, true);
                    }
                    if(!robot.drive.isBusy()){
                        changeStateTo(state.ShootingPose4);
                    }
                    break;

                case ShootingPose4:

                    if(robot.stateChanged) {
                    }
                    if(robot.stateTime.seconds() > 2){
                        robot.intake.setPower(.3);
                        robot.spanker.setPosition(1);
                        //robot.stateTime.reset();
                    }
                    if(robot.stateTime.seconds() > 4.5){
                        if(!robot.drive.isBusy()){
                            robot.intake.setPower(0);
                            robot.spanker.setPosition(.65);
                            changeStateTo(state.parking);
                        }
                    }
                    break;

                case parking:
                    if(robot.stateChanged){
                        robot.drive.followPath(REDLastBall, true);
                    }
                    if(!robot.drive.isBusy()){
                        changeStateTo(state.Balls);
                        robot.spanker.setPosition(.65);
                        robot.intake.setPower(.8);
                    }

                    break;

                case Balls:
                    if(robot.stateChanged) {
                        robot.drive.followPath(REDBald, true);

                    }


                    // ----------------------- FIXED SECTION END -----------------------
            }
        }
    }

    public void update(){
        robot.drive.update();
        robot.stateUpdate(State);

        telemetry.addData("x", robot.drive.getPose().getX());
        telemetry.addData("y", robot.drive.getPose().getY());
        telemetry.addData("heading RR", Math.toDegrees(robot.drive.getPose().getHeading()));
        telemetry.addData("State: ", State);
        telemetry.addData("isDon: ", !robot.drive.isBusy());
        telemetry.addData("statechange: ", robot.stateChanged);
        telemetry.addData("velocity", robot.spinner.getVelocity());
        telemetry.addData("Intake velocity", robot.intake.getVelocity());
        telemetry.update();
    }

    public void changeStateTo(state tostate){
        State = tostate;
    }
}

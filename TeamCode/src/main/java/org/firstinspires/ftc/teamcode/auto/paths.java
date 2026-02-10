package org.firstinspires.ftc.teamcode.auto;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.hardware.RiskyHardware;

public class paths {

    RiskyHardware robot;
    public static final Pose startPoseBlue = new Pose(64, 8.5);
    public static final Pose startPoseRed = new Pose(144-64, 8.5);
    public static final Pose launchFarBlue = new Pose(57.500, 17.700);
    public static final Pose launchFarRed = new Pose(144-57.500, 17.500);
    public static final Pose launchMidRangeRed = new Pose(85.8, 74.7, Math.toRadians(49.2));
    public static final Pose launchMidRangeBlue = new Pose(58.2, 74.7, Math.toRadians(130));
    public static final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0));
    public static Path scorePreload;
    public static PathChain start2park, back2start, leaveTriangleBlue, leaveTriangleRed, StartPoseToShootPose, StartPoseToShootPoseLast, ShootPoseToStartPointPose, StartPoseToIntakePose, IntakePoseToShootPose, IntakePose2, StartPose2, PerChance1, GoBackFool, LastBall, Bald, Balls;


    public static PathChain REDstart2park, REDback2start, REDStartPoseToShootPose, REDShootPoseToStartPointPose,OpenGate, REDStartPoseToIntakePose, REDIntakePoseToShootPose, REDIntakePose2, REDStartPose2, REDPerChance1, REDGoBackFool, REDLastBall, REDBald, REDBalls;

    public paths (RiskyHardware robot){
        this.robot = robot;
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */



        leaveTriangleBlue = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPoseBlue, new Pose(50, 25)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        leaveTriangleRed = robot.drive.pathBuilder()
                .addPath(new BezierLine(startPoseRed, new Pose(144-50, 25)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        back2start = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 96), new Pose(25.121, 127.402))
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(130))
                .build();
        start2park = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(25.121, 127.402), new Pose(15.000, 84.700))
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();

        REDback2start = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-57, 96), new Pose(144-25.121, 127.402))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-140), Math.toRadians(180-130))
                .build();
        REDstart2park = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-25.121, 127.402), new Pose(144-15.000, 84.700))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-130), Math.toRadians(180-180))
                .build();









        StartPoseToShootPose = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(25.121, 127.402), new Pose(57, 96))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();
        StartPoseToShootPoseLast = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(25.121, 127.402), new Pose(57, 96))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();
        ShootPoseToStartPointPose = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 96), new Pose(51.364, 84.700))
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();
        StartPoseToIntakePose = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(51.364, 84.700), new Pose(15.000, 84.700))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        IntakePoseToShootPose = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15, 84.7), new Pose(57, 96))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();
        StartPose2 = robot.drive .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 96), new Pose(45, 59.600))
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();
        IntakePose2 = robot.drive
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45, 59.6), new Pose(10, 59))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();



        PerChance1 = robot.drive
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10, 59), new Pose(27, 58.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
        LastBall = robot.drive
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 96), new Pose(42, 36))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Bald = robot.drive
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42, 36), new Pose(14, 36))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();









        ///RED

        REDStartPoseToShootPose = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-25.121, 127.402), new Pose(144-57, 96))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-136), Math.toRadians(180-140))
                .build();
        REDShootPoseToStartPointPose = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-57, 96), new Pose(144-51.364, 84.700))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-140), Math.toRadians(0))
                .build();
        REDStartPoseToIntakePose = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-51.364, 84.700), new Pose(144-15.000, 84.700))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        OpenGate = robot.drive.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-15, 84.700),
                                new Pose(144-40, 77),
                                new Pose(144-12, 73)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        REDIntakePoseToShootPose = robot.drive.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-15, 84.7), new Pose(144-57, 96))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        REDStartPose2 = robot.drive .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-57, 96), new Pose(144-45, 57))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        REDIntakePose2 = robot.drive
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-45, 57), new Pose(144-5, 57))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();



        REDPerChance1 = robot.drive
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-5, 57), new Pose(144-27, 58.8))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        REDLastBall = robot.drive
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-59, 96), new Pose(144-42, 36))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        REDBald = robot.drive
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-42, 36), new Pose(144-14, 36))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();


    }
}
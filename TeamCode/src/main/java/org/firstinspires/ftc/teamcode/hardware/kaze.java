package org.firstinspires.ftc.teamcode.hardware;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


public class kaze {
    public volatile static Pose robotPose;
    public volatile static Vector2d target;
    //public volatile static double headingOffset;
    public volatile static boolean IsBlue = false;
    public volatile static String pattern;

    public static void init(Pose startPose){
        robotPose = startPose;
        //headingOffset = Math.toDegrees(startPose.getHeading());
    }
    public static void init(Pose startPose, boolean isblue){
        robotPose = startPose;
        IsBlue = isblue;
        if(IsBlue){
            target = new Vector2d(3,144);
        } else {
            target = new Vector2d(141,144);
        }
        //headingOffset = Math.toDegrees(startPose.getHeading());
    }

    public static void update(Follower drive, String Pattern){
        if(robotPose == null){
            robotPose = new Pose(72,72,0);
        }
        drive.updatePose();
        robotPose = drive.getPose();

        drawCurrentAndHistory(drive);
        pattern = Pattern;
    }

    public static void update(Follower drive){
        if(robotPose == null){
            robotPose = new Pose(72,72,0);
        }
        drive.updatePose();
        robotPose = drive.getPose();

        drawCurrentAndHistory(drive);
    }

    public static void drawCurrent(Follower drive) {
        try {
            Drawing.drawRobot(drive.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory(Follower drive) {
        Drawing.drawPoseHistory(drive.getPoseHistory());
        drawCurrent(drive);
    }
}
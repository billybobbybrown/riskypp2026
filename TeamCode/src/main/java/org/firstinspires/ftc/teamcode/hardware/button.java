package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public class button {
    boolean incomingState;
    boolean currentState;
    boolean lastState;
    boolean currToggleState = false;


    public void update(boolean incomingState){
        this.lastState = this.currentState;
        this.currentState = incomingState;
    }

    public boolean isDown() {
        return this.currentState;
    }

    public boolean wasJustPressed() {
        return !this.lastState && this.currentState;
    }

    public boolean wasJustReleased() {
        return this.lastState && !this.currentState;
    }

    public boolean stateJustChanged() {
        return this.lastState != this.currentState;
    }
    public boolean getToggleState() {
        if (this.wasJustReleased()) {
            this.currToggleState = !this.currToggleState;
        }

        return this.currToggleState;
    }

    public static class ButtonReader {
        //private OpMode opMode;

        public button x = new button();
        public button square = new button();
        public button circle = new button();
        public button triangle = new button();
        public button right_Stick_Button = new button();
        public button left_Stick_Button = new button();
        public button Dpad_Up = new button();
        public button Dpad_Down = new button();
        public button Dpad_Right = new button();
        public button Dpad_Left = new button();
        public button Guide = new button();
        public button TouchPad = new button();
        public button finger1 = new button();
        public button finger2 = new button();
        public button Left_Bumper = new button();
        public button Right_Bumper = new button();
        public button Options = new button();
        public button Share = new button();
        public button Right_Trigger = new button();
        public button Left_Trigger = new button();
        public button justMagnet = new button();

    /*public void init(LinearOpMode opmode){
        this.opMode = opmode;
    }*/

        public void update(Gamepad gamepad, RiskyHardware robot){
            x.update(gamepad.a);
            square.update(gamepad.square);
            circle.update(gamepad.circle);
            triangle.update(gamepad.triangle);
            right_Stick_Button.update(gamepad.right_stick_button);
            left_Stick_Button.update(gamepad.left_stick_button);
            Dpad_Up.update(gamepad.dpad_up);
            Dpad_Down.update(gamepad.dpad_down);
            Dpad_Right.update(gamepad.dpad_right);
            Dpad_Left.update(gamepad.dpad_left);
            Guide.update(gamepad.guide);
            TouchPad.update(gamepad.touchpad);
            finger1.update(gamepad.touchpad_finger_1);
            finger2.update(gamepad.touchpad_finger_2);
            Left_Bumper.update(gamepad.left_bumper);
            Right_Bumper.update(gamepad.right_bumper);
            Options.update(gamepad.options);
            Share.update(gamepad.share);
            Right_Trigger.update(gamepad.right_trigger>.5);
            Left_Trigger.update(gamepad.left_trigger>.5);
           // justMagnet.update(robot.magnet.getState());
        }
    }
}


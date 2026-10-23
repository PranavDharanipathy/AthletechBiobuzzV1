package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.Gamepad;

/// @see Gamepad
@SuppressWarnings("all")
public class BetterGamepad {

    private Gamepad gamepad;

    public BetterGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    //BOOLEANS
    public boolean a() { return gamepad.a; }
    public boolean b() { return gamepad.b; }
    public boolean x() { return gamepad.x; }
    public boolean y() { return gamepad.y; }

    public boolean circle() { return gamepad.circle; }
    public boolean cross() { return gamepad.cross; }
    public boolean square() { return gamepad.square; }
    public boolean triangle() { return gamepad.triangle; }

    public boolean dpad_up() { return gamepad.dpad_up; }
    public boolean dpad_down() { return gamepad.dpad_down; }
    public boolean dpad_left() { return gamepad.dpad_left; }
    public boolean dpad_right() { return gamepad.dpad_right; }

    public boolean left_bumper() { return gamepad.left_bumper; }
    public boolean right_bumper() { return gamepad.right_bumper; }

    public boolean left_trigger(float threshold) { return gamepad.left_trigger >= threshold; }
    public boolean right_trigger(float threshold) { return gamepad.right_trigger >= threshold; }

    public boolean back() { return gamepad.back;}

    public boolean options() { return gamepad.options; }

    public boolean share() { return gamepad.share; }

    public boolean main_button() { return gamepad.guide || gamepad.ps; }

    public boolean left_stick_button() { return gamepad.left_stick_button; }
    public boolean right_stick_button() { return gamepad.right_stick_button; }

    public boolean isRumbling() { return gamepad.isRumbling(); }

    public boolean atRest() { return gamepad.atRest(); }

    //FLOATS
    public float left_stick_y() { return gamepad.left_stick_y; }
    public float left_stick_x() { return gamepad.left_stick_x; }
    public float right_stick_y() { return gamepad.right_stick_y; }
    public float right_stick_x() { return gamepad.right_stick_x; }

    public float left_trigger() { return gamepad.left_trigger; }
    public float right_trigger() { return gamepad.right_trigger; }

    //COMMANDS
    public void stopRumble() { gamepad.stopRumble(); }
    public void rumble(int durationMs) { gamepad.rumble(durationMs); }
    public void runRumbleEffect(Gamepad.RumbleEffect effect) { gamepad.runRumbleEffect(effect); }

    // to detect if a key has just been pressed
    public boolean curr_a = false;
    public boolean prev_a = false;
    public boolean aHasJustBeenPressed = false;

    public boolean curr_b = false;
    public boolean prev_b = false;
    public boolean bHasJustBeenPressed = false;

    public boolean curr_x = false;
    public boolean prev_x = false;
    public boolean xHasJustBeenPressed = false;

    public boolean curr_y = false;
    public boolean prev_y = false;
    public boolean yHasJustBeenPressed = false;

    public boolean curr_circle = false;
    public boolean prev_circle = false;
    public boolean circleHasJustBeenPressed = false;

    public boolean curr_cross = false;
    public boolean prev_cross = false;
    public boolean crossHasJustBeenPressed = false;

    public boolean curr_square = false;
    public boolean prev_square = false;
    public boolean squareHasJustBeenPressed = false;

    public boolean curr_triangle = false;
    public boolean prev_triangle = false;
    public boolean triangleHasJustBeenPressed = false;

    public boolean curr_right_bumper = false;
    public boolean prev_right_bumper = false;
    public boolean right_bumperHasJustBeenPressed = false;

    public boolean curr_left_bumper = false;
    public boolean prev_left_bumper = false;
    public boolean left_bumperHasJustBeenPressed = false;

    public boolean curr_dpad_up = false;
    public boolean prev_dpad_up = false;
    public boolean dpad_upHasJustBeenPressed = false;

    public boolean curr_dpad_down = false;
    public boolean prev_dpad_down = false;
    public boolean dpad_downHasJustBeenPressed = false;

    public boolean curr_dpad_left = false;
    public boolean prev_dpad_left = false;
    public boolean dpad_leftHasJustBeenPressed = false;

    public boolean curr_dpad_right = false;
    public boolean prev_dpad_right = false;
    public boolean dpad_rightHasJustBeenPressed = false;

    public boolean curr_back = false;
    public boolean prev_back = false;
    public boolean backHasJustBeenPressed = false;

    public boolean curr_main_button = false;
    public boolean prev_main_button = false;
    public boolean main_buttonHasJustBeenPressed = false;

    public boolean curr_options = false;
    public boolean prev_options = false;
    public boolean optionsHasJustBeenPressed = false;

    public boolean curr_share = false;
    public boolean prev_share = false;
    public boolean shareHasJustBeenPressed = false;

    public boolean curr_left_stick_button = false;
    public boolean prev_left_stick_button = false;
    public boolean left_stick_buttonHasJustBeenPressed = false;

    public boolean curr_right_stick_button = false;
    public boolean prev_right_stick_button = false;
    public boolean right_stick_buttonHasJustBeenPressed = false;

    public boolean curr_left_trigger = false;
    public boolean prev_left_trigger = false;
    public boolean left_triggerHasJustBeenPressed = false;

    public boolean curr_right_trigger = false;
    public boolean prev_right_trigger = false;
    public boolean right_triggerHasJustBeenPressed = false;

    public void getInformation() {

        prev_a = curr_a;
        curr_a = gamepad.a;
        aHasJustBeenPressed = !prev_a && curr_a;

        prev_b = curr_b;
        curr_b = gamepad.b;
        bHasJustBeenPressed = !prev_b && curr_b;

        prev_x = curr_x;
        curr_x = gamepad.x;
        xHasJustBeenPressed = !prev_x && curr_x;

        prev_y = curr_y;
        curr_y = gamepad.y;
        yHasJustBeenPressed = !prev_y && curr_y;

        prev_circle = curr_circle;
        curr_circle = gamepad.circle;
        circleHasJustBeenPressed = !prev_circle && curr_circle;

        prev_cross = curr_cross;
        curr_cross = gamepad.cross;
        crossHasJustBeenPressed = !prev_cross && curr_cross;

        prev_square = curr_square;
        curr_square = gamepad.square;
        squareHasJustBeenPressed = !prev_square && curr_square;

        prev_triangle = curr_triangle;
        curr_triangle = gamepad.triangle;
        triangleHasJustBeenPressed = !prev_triangle && curr_triangle;

        prev_right_bumper = curr_right_bumper;
        curr_right_bumper = gamepad.right_bumper;
        right_bumperHasJustBeenPressed = !prev_right_bumper && curr_right_bumper;

        prev_left_bumper = curr_left_bumper;
        curr_left_bumper = gamepad.left_bumper;
        left_bumperHasJustBeenPressed = !prev_left_bumper && curr_left_bumper;

        prev_dpad_up = curr_dpad_up;
        curr_dpad_up = gamepad.dpad_up;
        dpad_upHasJustBeenPressed = !prev_dpad_up && curr_dpad_up;

        prev_dpad_down = curr_dpad_down;
        curr_dpad_down = gamepad.dpad_down;
        dpad_downHasJustBeenPressed = !prev_dpad_down && curr_dpad_down;

        prev_dpad_left = curr_dpad_left;
        curr_dpad_left = gamepad.dpad_left;
        dpad_leftHasJustBeenPressed = !prev_dpad_left && curr_dpad_left;

        prev_dpad_right = curr_dpad_right;
        curr_dpad_right = gamepad.dpad_right;
        dpad_rightHasJustBeenPressed = !prev_dpad_right && curr_dpad_right;

        prev_back = curr_back;
        curr_back = gamepad.back;
        backHasJustBeenPressed = !prev_back && curr_back;

        prev_main_button = curr_main_button;
        curr_main_button = main_button();
        main_buttonHasJustBeenPressed = !prev_main_button && curr_main_button;

        prev_options = curr_options;
        curr_options = gamepad.options;
        optionsHasJustBeenPressed = !prev_options && curr_options;

        prev_share = curr_share;
        curr_share = gamepad.share;
        shareHasJustBeenPressed = !prev_share && curr_share;

        prev_left_stick_button = curr_left_stick_button;
        curr_left_stick_button = gamepad.left_stick_button;
        left_stick_buttonHasJustBeenPressed = !prev_left_stick_button && curr_left_stick_button;

        prev_right_stick_button = curr_right_stick_button;
        curr_right_stick_button = gamepad.right_stick_button;
        right_stick_buttonHasJustBeenPressed = !prev_right_stick_button && curr_right_stick_button;

        prev_left_trigger = curr_left_trigger;
        curr_left_trigger = gamepad.left_trigger > 0.2f;
        left_triggerHasJustBeenPressed = !prev_left_trigger && curr_left_trigger;

        prev_right_trigger = curr_right_trigger;
        curr_right_trigger = gamepad.right_trigger > 0.2f;
        right_triggerHasJustBeenPressed = !prev_right_trigger && curr_right_trigger;
    }

}

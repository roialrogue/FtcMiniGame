package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class myGamePad {
    Gamepad gamepad;
    boolean xButtonPressed = false;
    boolean yButtonPressed = false;
    boolean aButtonPressed = false;
    boolean bButtonPressed = false;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    boolean dpadLeftPressed = false;
    boolean dpadRightPressed = false;
    boolean dpadUpPressed = false;
    boolean dpadDownPressed = false;
    boolean leftStickButtonPressed = false;
    boolean rightstickbuttonPressed = false;


    /* Constructor */
    public myGamePad(Gamepad gamepad ) {


        this.gamepad = gamepad;
    }


    public double getLeftStickY() {


        return gamepad.left_stick_y;
    }


    public double getLeftStickX() {


        return gamepad.left_stick_x;
    }


    public double getRightStickY() {


        return gamepad.right_stick_y;
    }


    public double getRightStickX() {


        return gamepad.right_stick_x;
    }


    public double getLeftTrigger() {


        return gamepad.left_trigger;
    }


    public double getRightTrigger() {


        return gamepad.right_trigger;
    }


    public boolean isXPressed() {


        if( gamepad.x && !xButtonPressed ) {
            xButtonPressed = true;
            return true;


        } else {
            if( !gamepad.x ) {
                xButtonPressed = false;
            }


            return false;
        }
    }


    public boolean isYPressed() {


        if( gamepad.y && !yButtonPressed ) {
            yButtonPressed = true;
            return true;
        } else {
            if( !gamepad.y ) {
                yButtonPressed = false;
            }
            return false;
        }
    }


    public boolean isAPressed() {


        if( gamepad.a && !aButtonPressed ) {
            aButtonPressed = true;
            return true;
        } else {
            if( !gamepad.a ) {
                aButtonPressed = false;
            }
            return false;
        }
    }


    public boolean isBPressed() {


        if( gamepad.b && !bButtonPressed ) {
            bButtonPressed = true;
            return true;
        } else {
            if( !gamepad.b ) {
                bButtonPressed = false;
            }
            return false;
        }
    }


    public boolean isLeftBumperPressed() {


        if (gamepad.left_bumper && !leftBumperPressed) {
            leftBumperPressed = true;
            return true;
        } else {
            if (!gamepad.left_bumper) {
                leftBumperPressed = false;
            }
            return false;
        }
    }


    public boolean isRightBumperPressed() {


        if (gamepad.right_bumper && !rightBumperPressed) {
            rightBumperPressed = true;
            return true;
        } else {
            if (!gamepad.right_bumper) {
                rightBumperPressed = false;
            }
            return false;
        }
    }


    public boolean isDpadLeftPressed() {


        if (gamepad.dpad_left && !dpadLeftPressed) {
            dpadLeftPressed = true;
            return true;
        } else {
            if (!gamepad.dpad_left) {
                dpadLeftPressed = false;
            }
            return false;
        }
    }


    public boolean isDpadRightPressed() {


        if (gamepad.dpad_right && !dpadRightPressed) {
            dpadRightPressed = true;
            return true;
        } else {
            if (!gamepad.dpad_right) {
                dpadRightPressed = false;
            }
            return false;
        }
    }
    public boolean isDpadUpPressed() {


        if (gamepad.dpad_up && !dpadUpPressed) {
            dpadUpPressed = true;
            return true;
        } else {
            if (!gamepad.dpad_up) {
                dpadUpPressed = false;
            }
            return false;
        }
    }


    public boolean isDpadDownPressed() {


        if (gamepad.dpad_down && !dpadDownPressed) {
            dpadDownPressed = true;
            return true;
        } else {
            if (!gamepad.dpad_down) {
                dpadDownPressed = false;
            }
            return false;
        }
    }


    public boolean isleftstickbuttonPressed() {


        if (gamepad.left_stick_button && !leftStickButtonPressed) {
            leftStickButtonPressed = true;
            return true;
        } else {
            if (!gamepad.left_stick_button) {
                leftStickButtonPressed = false;
            }
            return false;
        }
    }


    public boolean isrightstickbuttonPressed() {


        if (gamepad.right_stick_button && !rightstickbuttonPressed) {
            rightstickbuttonPressed = true;
            return true;
        } else {
            if (!gamepad.right_stick_button) {
                rightstickbuttonPressed = false;
            }
            return false;
        }
    }
}

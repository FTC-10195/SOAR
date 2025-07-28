package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ElephantFoot {
    public enum States {
        UP,
        DOWN
    }

    States currentState = States.UP;
    Servo servo;
    public static double upPos = 0;
    public static double downPos = 1;

    public void setCurrentState(States newState) {
        currentState = newState;
    }

    public States getState() {
        return currentState;
    }

    public void initiate(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get("foot");
    }

    public void flip() {
        switch (currentState) {
            case UP:
                currentState = States.DOWN;
                break;
            case DOWN:
                currentState = States.UP;
                break;
        }
    }

    public void update() {
        switch (currentState) {
            case UP:
                servo.setPosition(upPos);
                break;
            case DOWN:
                servo.setPosition(downPos);
                break;
        }
    }

}

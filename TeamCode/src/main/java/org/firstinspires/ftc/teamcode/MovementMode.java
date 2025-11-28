package org.firstinspires.ftc.teamcode;

public enum MovementMode {
    FORWARD("forward"),
    BACKWARD("backward"),
    STRAFE_LEFT("strafe_left"),
    STRAFE_RIGHT("strafe_right"),
    FORWARD_LEFT("forward_left"),
    FORWARD_RIGHT("forward_right"),
    BACKWARD_LEFT("backward_left"),
    BACKWARD_RIGHT("backward_right"),
    TURN_LEFT("turn_left"),
    TURN_RIGHT("turn_right");

    private final String name;

    MovementMode(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
}

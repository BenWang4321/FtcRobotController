package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="Just Servo", group="testing")
public class justServo extends LinearOpMode {

    private Servo servo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(gamepad1.left_trigger);
        }
    }
}

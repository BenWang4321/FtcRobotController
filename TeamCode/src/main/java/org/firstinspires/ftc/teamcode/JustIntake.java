package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "just intake", group = "testing")
public class JustIntake extends LinearOpMode {

    DcMotor intake = null;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(gamepad1.left_stick_y);
        }
    }
}

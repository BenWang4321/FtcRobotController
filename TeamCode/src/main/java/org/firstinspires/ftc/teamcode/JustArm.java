package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Just Arm", group = "testing")
public class JustArm extends LinearOpMode {

    public DcMotor ejector1 = null;
    public DcMotor ejector2 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ejector1 = hardwareMap.get(DcMotor.class, "ejector1");
        ejector2 = hardwareMap.get(DcMotor.class, "ejector2");

        waitForStart();

        while (opModeIsActive()) {
            double eject = gamepad2.right_stick_y;

            ejector1.setPower(eject);
            ejector2.setPower(-eject);
        }

    }
}

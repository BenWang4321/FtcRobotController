package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "Just Arm", group = "testing")
public class JustArm extends LinearOpMode {

    public DcMotor ejector1 = null;
    public DcMotor ejector2 = null;

    public CRServoImplEx servo1, servo2 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ejector1 = hardwareMap.get(DcMotor.class, "ejector1");
        ejector2 = hardwareMap.get(DcMotor.class, "ejector2");

        servo1 = hardwareMap.get(CRServoImplEx.class, "servo1");
        servo2 = hardwareMap.get(CRServoImplEx.class, "servo2");


        waitForStart();

        while (opModeIsActive()) {
            double eject = gamepad2.right_stick_y;

            ejector1.setPower(eject);
            ejector2.setPower(eject);

            if (gamepad2.a) {
                servo1.setPower(0.7);

            } else {
                servo2.setPower(0);
            }

            if (gamepad2.b) {
                servo1.setPower(-0.7);
            } else {
                servo1.setPower(0);
            }
        }
    }
}

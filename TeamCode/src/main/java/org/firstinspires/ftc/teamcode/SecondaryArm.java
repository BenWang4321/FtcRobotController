package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Just second arm")
public class SecondaryArm extends LinearOpMode {

    private DcMotor frontLeft, backLeft, backRight, frontRight;
    private CRServo primaryArm, secondaryArm;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        try {
            primaryArm = hardwareMap.get(CRServo.class, "primary_arm");
        } catch (Exception e) {
            telemetry.addData("Error", "primary_arm (Servo) not found in config");
            primaryArm = null;
        }

        try {
            secondaryArm = hardwareMap.get(CRServo.class, "secondary_arm");
        } catch (Exception e) {
            telemetry.addData("Error", "secondary_arm (Servo) not found in config");
            secondaryArm = null;
        }

        double drivePowerMax = 1;
        double armPowerMax = 0.75;

        waitForStart();

        while (opModeIsActive()) {
            double forwardBackward = -gamepad1.left_stick_y * 0.7; // Scale for smoother control
            double strafe = -gamepad1.left_stick_x * 0.7; // Scale strafing
            double turn = gamepad1.right_stick_x * 0.6; // Reduce turning sensitivity

            double primaryPower = -gamepad2.left_stick_y;
            double secondaryPower = -gamepad2.right_stick_y;
            double primaryArmPower;
            double secondaryArmPower;

            if (gamepad1.a) {
                drivePowerMax = 1;
            } else if (gamepad1.b) {
                drivePowerMax = 0.75;
            } else if (gamepad1.x) {
                drivePowerMax = 0.25;
            } else if (gamepad1.y) {
                drivePowerMax = 0.5;
            }
            if (gamepad1.left_bumper && drivePowerMax < 1) {
                drivePowerMax += 0.001;
            } else if (gamepad1.right_bumper && drivePowerMax > 0) {
                drivePowerMax -= 0.001;
            }

            double frontLeftPower = forwardBackward + strafe + turn;
            double backLeftPower = forwardBackward - strafe + turn;
            double backRightPower = forwardBackward - strafe - turn;
            double frontRightPower = forwardBackward + strafe - turn;

            // Normalize power values to prevent exceeding motor limits
            double max = Math.max(1.0, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            max = Math.max(max, Math.abs(frontRightPower));

            if (max > 1) {
                frontLeft.setPower(frontLeftPower / max / drivePowerMax);
                frontRight.setPower(frontRightPower / max / drivePowerMax);
                backLeft.setPower(backLeftPower / max / drivePowerMax);
                backRight.setPower(backRightPower / max / drivePowerMax);
            } else {
                frontLeft.setPower(frontLeftPower / drivePowerMax);
                frontRight.setPower(frontRightPower / drivePowerMax);
                backLeft.setPower(backLeftPower / drivePowerMax);
                backRight.setPower(backRightPower / drivePowerMax);
            }

            primaryArmPower = Range.clip(primaryPower + 0.15, -0.35, armPowerMax);
            secondaryArmPower = Range.clip(secondaryPower, -1, 1);
            primaryArm.setPower(primaryArmPower);
            secondaryArm.setPower(secondaryArmPower);

            if (gamepad2.x) {
                armPowerMax = 1;
            } else if (gamepad2.y) {
                armPowerMax = 0.8;
            } else if (gamepad2.dpad_left) {
                armPowerMax = 0.6;
            } else if (gamepad2.dpad_right) {
                armPowerMax = 0.4;
            }

            if (gamepad2.dpad_up && armPowerMax < 1) {
                armPowerMax += 0.001;
            } else if (gamepad2.dpad_down && armPowerMax > 0.15) {
                armPowerMax -= 0.001;
            }
        }
    }
}

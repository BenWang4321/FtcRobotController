package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum Drive V2", group = "TeleOp")
public class MecanumDriveWithArm extends LinearOpMode {

    private DcMotor frontLeft, backLeft, backRight, frontRight;
    private CRServo primaryArm, secondaryArm, sweeperServo;
    private Servo pincherServo;

    public IMU imu = null;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        primaryArm = hardwareMap.get(CRServo.class, "primary_arm");
        secondaryArm = hardwareMap.get(CRServo.class, "secondary_arm");

        pincherServo = hardwareMap.get(Servo.class, "pincher_servo");
        sweeperServo = hardwareMap.get(CRServo.class, "sweeper_servo");

        imu = hardwareMap.get(IMU.class, "imu");
        // Reverse the right side motors for proper direction
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick values
            double y = -gamepad1.left_stick_y; // Forward/Backward
            double x = gamepad1.left_stick_x * 1.1; // Strafe (adjusted for imperfect strafing)
            double turn = gamepad1.right_stick_x; // Rotation

            double primaryPower = -gamepad2.left_stick_y;
            double secondaryPower = -gamepad2.right_stick_y;
            double primaryArmPower;
            double secondaryArmPower;

            // Calculate power for each wheel
            double frontLeftPower = y + x + turn;
            double frontRightPower = y - x - turn;
            double backLeftPower = y - x + turn;
            double backRightPower = y + x - turn;

            // Normalize powers if any exceed 1
            double max = Math.max(1.0, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1) {
                frontLeft.setPower(frontLeftPower / max);
                frontRight.setPower(frontRightPower / max);
                backLeft.setPower(backLeftPower / max);
                backRight.setPower(backRightPower / max);
            } else {
                frontLeft.setPower(frontLeftPower);
                frontRight.setPower(frontRightPower);
                backLeft.setPower(backLeftPower);
                backRight.setPower(backRightPower);
            }

            //Arm control
            primaryArmPower = Range.clip(primaryPower + 0.15, -0.35, 1);
            secondaryArmPower = Range.clip(secondaryPower, -1, 1);

            primaryArm.setPower(primaryArmPower);

            // Secondary arm control (up/down with right stick on gamepad2)

            secondaryArm.setPower(secondaryArmPower);

            // Pincher servo control (e.g. A button = closed, B button = open)
            if (gamepad2.a) {
                pincherServo.setPosition(0.57);
            } else if (gamepad2.b) {
                pincherServo.setPosition(0.67);
            }

            // Sweeper servo control (continuous rotation)
            // Left bumper: spin one way, Right bumper: spin opposite way,
            // no bumper: stop.
            if (gamepad2.left_bumper) {
                sweeperServo.setPower(1.5); // spin forward
            } else if (gamepad2.right_bumper) {
                sweeperServo.setPower(-1.5); // spin backward
            } else {
                sweeperServo.setPower(0.0); // stop
            }

            // Telemetry for debugging
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("BR Power", backRightPower);
            telemetry.addData("Primary Arm Power", primaryArmPower);
            telemetry.addData("Secondary Arm Power", secondaryArmPower);
            telemetry.addData("Pincher Position", pincherServo.getPosition());
            telemetry.addData("Sweeper Power", sweeperServo.getPower());
            telemetry.update();
        }
    }
}

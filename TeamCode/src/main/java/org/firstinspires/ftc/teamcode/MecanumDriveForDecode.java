package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum drive for Decode 2025-2026", group = "TeleOp")
public class MecanumDriveForDecode extends LinearOpMode {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight, arm1, arm2, ejector1, ejector2 = null;
    public Servo elevator = null;

    private double armPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        ejector1 = hardwareMap.get(DcMotorEx.class, "ejector1");
        ejector2 = hardwareMap.get(DcMotorEx.class, "ejector2");
        elevator = hardwareMap.get(Servo.class, "elevator");
        // Reverse the right side motors for proper direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setDirection(DcMotorSimple.Direction.FORWARD);
        arm2.setDirection(DcMotorSimple.Direction.FORWARD);
        ejector1.setDirection(DcMotorSimple.Direction.FORWARD);
        ejector2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ejector1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ejector2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        arm1.setPower(1);
        arm2.setPower(1);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setTargetPosition(0);
        arm2.setTargetPosition(0);

        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive()) {
            // Get joystick values
            double fb = -gamepad1.left_stick_y; // Scale for smoother control
            double strafe = -gamepad1.left_stick_x; // Scale strafing
            double turn = gamepad1.right_stick_x;

            //fb & turn: fr, fl inverted
            //strafe: fr, bl inverted
            /*original code:
            double frontLeftPower = fb + strafe + turn;
            double backLeftPower = fb - strafe + turn;
            double backRightPower = fb - strafe - turn;
            double frontRightPower = fb + strafe - turn;
            */
            double frontLeftPower = -fb + strafe - turn;
            double backLeftPower = fb + strafe + turn;
            double backRightPower = fb - strafe - turn;
            double frontRightPower = -fb - strafe + turn;

            double max = Math.max(1.0, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            max = Math.max(max, Math.abs(frontRightPower));


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

            //arm control
            double armPower = gamepad2.left_stick_y;
            double ejectorPower = 0.5 * gamepad2.right_stick_y;

            armPosition += armPower;

            ejector1.setPower(ejectorPower);
            ejector2.setPower(ejectorPower);

            arm1.setTargetPosition(Math.min((int) Math.round(armPosition), 1000));
            arm2.setTargetPosition(Math.min((int) Math.round(armPosition), 1000));

            telemetry.addData("arm position", armPosition);
            telemetry.update();
        }
    }
}

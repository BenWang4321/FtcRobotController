package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Mecanum Drive without arm", group = "TeleOp")
public class MecanumDriveWithoutArm extends LinearOpMode {


    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        // Reverse the right side motors for proper direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (opModeIsActive()) {
            // Get joystick values
            double fb = -gamepad1.left_stick_x * 0.7; // Scale for smoother control
            double strafe = -gamepad1.left_stick_y * 0.7; // Scale strafing
            double turn = gamepad1.right_stick_x * 0.6;

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
        }
    }
}

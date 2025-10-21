package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auto park with wait")
public class AutoParkWithWait extends LinearOpMode {
    private IMU imu;

    private DcMotor frontLeft, backLeft, backRight, frontRight;
    private CRServo primaryArm, secondaryArm, sweeperServo;
    private Servo pincherServo;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

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

        sweeperServo = hardwareMap.get(CRServo.class, "sweeper_servo");
        pincherServo = hardwareMap.get(Servo.class, "pincher_servo");

        imu = hardwareMap.get(IMU.class, "imu");

        // Reverse the right side motors for proper direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        pincherServo.setPosition(0.67);
        sweeperServo.setPower(0);

        sleep(28000);

        primaryArm.setPower(1);
        sleep(218);
        primaryArm.setPower(0);

        frontLeft.setPower(-0.7);
        backLeft.setPower(-0.7);
        backRight.setPower(-0.7);
        frontRight.setPower(-0.7);
        sleep(500);
        frontLeft.setPower(-0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.7);
        frontRight.setPower(-0.7);
        sleep(4000);
        stopMoving();
    }

    private void stopMoving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}

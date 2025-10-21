package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotAutoDriveByGyro_Linear.getHeading;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Two-Driver Arcade Drive with Arm", group="Linear Opmode")
public class ArcadeArm extends LinearOpMode {
    public static final double TIME_PER_TILE = 1000;
    public static final double TIME_PER_TURN = 1000;

    public final DcMotor leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
    public final DcMotor rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
    public final DcMotor primaryArm = hardwareMap.get(DcMotor.class, "primary_arm");
    public final DcMotor secondaryArm = hardwareMap.get(DcMotor.class, "secondary_arm");

    // Initialize servos
    // Servos
    public final Servo pincherServo = hardwareMap.get(Servo.class, "pincher_servo");
    public final CRServo sweeperServo = hardwareMap.get(CRServo.class, "sweeper_servo");

    public final IMU imu = hardwareMap.get(IMU.class, "imu");

    @Override
    public void runOpMode() {
        // Initialize drive motors
        // Drive motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize arm motors
        // Arm motors

        // Initialize IMU
        IMU.Parameters myIMUParameters;

        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.XYZ,
                                AngleUnit.DEGREES, 0,0,0,0
                        )
                )
        );
        imu.initialize(myIMUParameters);

        // Set initial positions or states for servos if desired

        double drivePowerMax = 1;
        // Wait for the game to start

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        pincherServo.setPosition(0.67); // Neutral position (adjust as needed)
        sweeperServo.setPower(0);      // Stop sweeper initially

        while (opModeIsActive()) {
            // ---------------------------
            // Driver 1 (Arcade Drive)
            // ---------------------------
            double drive = -gamepad1.left_stick_y;  // forward/back
            double turn = gamepad1.left_stick_x;    // left/right
            double primaryPower = -gamepad2.left_stick_y;
            double secondaryPower = -gamepad2.right_stick_y;
            double leftPower = drive + turn;
            double rightPower = drive - turn;
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

            if (gamepad1.dpad_right) {

            } else if (gamepad1.dpad_up) {

            } else if (gamepad1.dpad_left) {

            } else if (gamepad1.dpad_down) {

            }
            if (gamepad1.left_bumper && drivePowerMax < 1) {
                drivePowerMax += 0.001;
            } else if (gamepad1.right_bumper && drivePowerMax > 0) {
                drivePowerMax -= 0.001;
            }

            if (leftPower >= 0) {
                leftDrive.setPower(Math.min(leftPower, drivePowerMax));
            } else {
                leftDrive.setPower(Math.max(leftPower, -drivePowerMax));
            }
            if (rightPower >= 0) {
                rightDrive.setPower(Math.min(rightPower, drivePowerMax));
            } else {
                rightDrive.setPower(Math.max(rightPower, -drivePowerMax));
            }

            // ---------------------------
            // Driver 2 (Arm Control)
            // ---------------------------

            // Primary arm control (up/down with left stick on gamepad2)
            // Adjust scaling if needed

            primaryArmPower = Range.clip(primaryPower + 0.15, -0.35, 1);
            secondaryArmPower = Range.clip(secondaryPower, -1, 1);


            if ((primaryArm.getCurrentPosition() < 800 && primaryArm.getCurrentPosition() > -300) ||
                    (primaryArm.getCurrentPosition() < -1350 && primaryArm.getCurrentPosition() > -2600)) {
                primaryArm.setPower(primaryArmPower);
            } else {
                primaryArm.setPower(-0.5);
            }

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

            // ---------------------------
            // Telemetry (Optional)
            // ---------------------------
            telemetry.addData("Left Drive Power", leftPower);
            telemetry.addData("Right Drive Power", rightPower);
            telemetry.addData("Primary Arm Power", primaryArmPower);
            telemetry.addData("Secondary Arm Power", secondaryArmPower);
            telemetry.addData("Pincher Position", pincherServo.getPosition());
            telemetry.addData("Sweeper Power", sweeperServo.getPower());
            telemetry.addData("Drive Power Maximum", drivePowerMax);
            telemetry.addData("Primary Arm Position", primaryArm.getCurrentPosition());
            telemetry.addData("Secondary Arm Position", secondaryArm.getCurrentPosition());
            telemetry.update();
        }
    }
}

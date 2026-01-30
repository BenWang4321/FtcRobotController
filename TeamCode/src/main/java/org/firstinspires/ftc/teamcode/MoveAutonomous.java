package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "move off line", group = "autonomous")
public class MoveAutonomous extends LinearOpMode {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight = null;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx[] motorGroup = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        waitForStart();

        for (DcMotorEx motor : motorGroup) {
            motor.setTargetPosition(40000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }

        while (opModeIsActive()) {
            for (DcMotorEx motor : motorGroup) {
                telemetry.addData(motor.getDeviceName() + " current position", motor.getCurrentPosition());
                telemetry.addData(motor.getDeviceName() + " velocity", motor.getVelocity());
            }
            telemetry.update();
        }
    }
}

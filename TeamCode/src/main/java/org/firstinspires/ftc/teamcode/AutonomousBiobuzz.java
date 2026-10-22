package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Autonomous for Biobuzz", group = "auto")
public class AutonomousBiobuzz extends LinearOpMode {
    public DcMotor[] motorGroup = new DcMotor[4];

    @Override
    public void runOpMode() throws InterruptedException {
        motorGroup[0] = hardwareMap.get(DcMotor.class, "frontLeft");
        motorGroup[1] = hardwareMap.get(DcMotor.class, "frontRight");
        motorGroup[2] = hardwareMap.get(DcMotor.class, "backLeft");
        motorGroup[3] = hardwareMap.get(DcMotor.class, "backRight");

        for (DcMotor motor : motorGroup) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Motor Positions:");
            telemetry.addData("  FL motor pos", motorGroup[0].getCurrentPosition());
            telemetry.addData("  FR motor pos", motorGroup[1].getCurrentPosition());
            telemetry.addData("  BL motor pos", motorGroup[2].getCurrentPosition());
            telemetry.addData("  BR motor pos", motorGroup[3].getCurrentPosition());
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

@Autonomous(name="supra autonomous", group = "autonomous")

public class SupraAutonomous extends LinearOpMode {

    DcMotorEx frontLeft, backLeft, backRight, frontRight = null;
    DcMotorEx[] motorGroup = {frontRight, frontLeft, backRight, backLeft};
    double cameraDist;
    DcMotor ejector1, ejector2 = null;
    IMU imu;
    int positionTolerance = 10;
    int precisionPositionTolerance = 50;
    double DISTANCE_PER_ROTATION = 23.5619449019;
    double ROBOT_LENGTH = 38; //In Centimeters
    double speed;
    List<int[]> checkPoints = new ArrayList<>();
    int[] currentPosition = new int[4];
    int[] currentRotation = new int[1];
    double[] motorDistancesFromRobot = {1,1,1,1};
    int currentCheckPoint = 0;
    int maxVelocity = 3000;
    int preciseVelocity = 1000;

    List<int[]> supraCheckpoints = new ArrayList<>();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setTargetPositionTolerance(positionTolerance);
        backLeft.setTargetPositionTolerance(positionTolerance);
        frontRight.setTargetPositionTolerance(positionTolerance);
        backRight.setTargetPositionTolerance(positionTolerance);
        IMU.Parameters myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(myIMUparameters);

        initAprilTag();

        waitForStart();


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMotorEnable();
        frontRight.setMotorEnable();
        backLeft.setMotorEnable();
        backRight.setMotorEnable();

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {

            frontLeft.setPower(1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(1);

            frontLeft.setTargetPosition(supraCheckpoints.get(currentCheckPoint)[0]);
            frontRight.setTargetPosition(supraCheckpoints.get(currentCheckPoint)[1]);
            backLeft.setTargetPosition(supraCheckpoints.get(currentCheckPoint)[2]);
            backRight.setTargetPosition(supraCheckpoints.get(currentCheckPoint)[3]);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            speed = frontLeft.getVelocity() * DISTANCE_PER_ROTATION / 100;
            // Share the CPU

            //detects if the motor has arrived to position
            if (frontRight.getPower() < 0.1 && frontLeft.getPower() < 0.1 &&
                    backRight.getPower() < 0.1 && backLeft.getPower() < 0.1 &&
                    currentCheckPoint < supraCheckpoints.size()) {
                currentCheckPoint += 1;
            }

            telemetryAprilTag();
            telemetry.addData("speed (m/s)", speed);
            telemetry.update();

        }

        // Save more CPU resources when camera is no longer needed.
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }   // end method telemetryAprilTag()

    /**
     * Adds a checkpoint for the motor on the robot.
     * Does all the calculations of the positions for you,
     * so you can just add how much you want the motor to change,
     * not where you want the motor to change to.
     */
    private void addCheckpoint() {
        supraCheckpoints.add(new int[]{0, 0, 0, 0});
    }

    private void executeCheckpoint() {
        int[] objective = checkPoints.get(currentCheckPoint);
        double orientationObjective;
        if (objective[3] - currentRotation[0] == 0) {
            orientationObjective = (objective[4] - currentRotation[0]) / (2 * maxVelocity * DISTANCE_PER_ROTATION) / 360;
        } else {
            orientationObjective = objective[3];
        }
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        robotOrientation.getYaw(AngleUnit.DEGREES);
        boolean IsAt = false;

        IntStream.range(0, 4).forEachOrdered(i -> motorGroup[i].setTargetPosition((int)
                Math.round(orientationObjective * (motorDistancesFromRobot[i] * 2 * Math.PI))));

        while (!IsAt) {
            for (DcMotorEx motor : motorGroup) {
                if (Math.abs(motor.getCurrentPosition() - orientationObjective) < positionTolerance) {
                    IsAt = true;
                } else if (Math.abs(motor.getCurrentPosition() - orientationObjective) < precisionPositionTolerance) {
                    motor.setVelocity(preciseVelocity);
                } else {
                    IsAt = false;
                }
            }
        }
        if (objective[0] == 0 && objective[1] == 0) {
            int distanceObjective = (int) Math.sqrt(Math.pow(objective[0], 2) + Math.pow(objective[1], 2));
            IsAt = false;

            while (!IsAt) {
                for (DcMotorEx motor : motorGroup) {
                    if (Math.abs(motor.getCurrentPosition() - distanceObjective) < positionTolerance) {
                        IsAt = true;
                    } else if (Math.abs(motor.getCurrentPosition() - distanceObjective) < precisionPositionTolerance) {
                        motor.setVelocity(preciseVelocity);
                    } else {
                        IsAt = false;
                    }
                }
            }
        }
    }

    private void driveToAprilTag(AprilTagDetection tag, double abortDistance) {
        if (tag.metadata != null) {
            double distance = abortDistance - tag.ftcPose.range;
        }
    }

    private void turnToAprilTag(AprilTagDetection tag, double abortAngle) {
        if (tag.metadata != null) {
            double angle = abortAngle - tag.ftcPose.bearing;
        }
    }

    private double calculateLaunchPower(double angle, double distance) {
        return 0; //just a placeholder
    }
    private double[] calculateRobotPosition() {
        return new double[]{};
    }

}
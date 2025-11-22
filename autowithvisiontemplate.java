package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.util.List;
import java.util.Map;

@Config
@Autonomous(name = "TEMPLATE", group = "Autonomous")
public class autowithvisiontemplate extends LinearOpMode {

    private DcMotorEx intakeMotor;
    private DcMotorEx outtakeMotorLeft;
    private DcMotorEx outtakeMotorRight;
    private DcMotorEx transferMotor;
    private CRServo intakeServo;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int detectedTagId = -1;
    private String detectedTagName = "UNKNOWN";

    private final Map<Integer, String> tagNames = Map.of(
            20, "BLUE",
            21, "GPP",
            22, "PGP",
            23, "PPG",
            24, "RED"
    );

    @Override
    public void runOpMode() throws InterruptedException {

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outake_motor_left");
        outtakeMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outake_motor_right");
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor_left");
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        Pose2d startPose = new Pose2d(-55, -55, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addLine("Initializing vision");
        telemetry.update();

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        telemetry.addLine("Initialized");
        telemetry.update();

        // ===== BUILD MOVEMENTS =====
        Action moveForward = drive.actionBuilder(startPose)
                .lineToX(-10)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        try {

            Actions.runBlocking(moveForward);

            outtakeMotorLeft.setPower(-0.7);
            outtakeMotorRight.setPower(0.7);
            transferMotor.setPower(-1.0);
            intakeMotor.setPower(1.0);
            intakeServo.setPower(1.0);

            // Scan for AprilTag using simple sleep (10 seconds total)
            for (int scan = 0; scan < 100; scan++) {
                List<AprilTagDetection> detections = aprilTag.getDetections();

                if (!detections.isEmpty()) {
                    AprilTagDetection tag = detections.get(0);
                    detectedTagId = tag.id;
                    detectedTagName = tagNames.getOrDefault(tag.id, "Unknown");

                    telemetry.addData("Tag Name", detectedTagName);
                    if (tag.ftcPose != null) {
                        telemetry.addData("Range (in)", String.format("%.1f", tag.ftcPose.range));
                    }
                } else {
                    telemetry.addLine("Scanning for tag");
                }
                telemetry.update();
                sleep(100);
            }

            outtakeMotorLeft.setPower(0.0);
            outtakeMotorRight.setPower(0.0);
            transferMotor.setPower(0.0);
            intakeMotor.setPower(0.0);
            intakeServo.setPower(0.0);


            // Save detected tag to file
            if (detectedTagId != -1) {
                File idFile = AppUtil.getInstance().getSettingsFile("lastTagId.txt");
                File nameFile = AppUtil.getInstance().getSettingsFile("lastTagLabel.txt");
                ReadWriteFile.writeFile(idFile, Integer.toString(detectedTagId));
                ReadWriteFile.writeFile(nameFile, detectedTagName);
            }

            Pose2d currentPose = drive.localizer.getPose();

            switch (detectedTagId) {
                case 21:  // GPP
                    // TODO: ADD CODE HERE
                    // To end code in a switch case, add a line with "break;"

                case 22:  // PGP
                    //  TODO: ADD CODE HERE

                case 23:  // PPG
                    // TODO: ADD CODE HERE

                default:  // No tag detected - use fallback
                    // TODO: ADD CODE HERE
            }

            telemetry.addData("Status", "Complete");

        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
        }
        telemetry.update();

        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        outtakeMotorLeft.setPower(0);
        outtakeMotorRight.setPower(0);
        intakeServo.setPower(0);

        if (visionPortal != null) visionPortal.close();

        Pose2d finalPose = drive.localizer.getPose();
        telemetry.addData("Final X", String.format("%.2f", finalPose.position.x));
        telemetry.addData("Final Y", String.format("%.2f", finalPose.position.y));
        telemetry.addData("Final Heading (deg)", String.format("%.2f", Math.toDegrees(finalPose.heading.toDouble())));
        telemetry.addLine("\nDone");
        telemetry.update();
    }
}

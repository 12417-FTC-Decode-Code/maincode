package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.247)
            .lateralZeroPowerAcceleration(-62.771538)
            .forwardZeroPowerAcceleration(-33.514266)
            .centripetalScaling(0.0007)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.03, 0.02));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(59.093)
            .yVelocity(42.10045)

            // STEP 1: RESTORE THE ORIGINAL NAMES
            // (We are undoing the swap from the previous step)
            .leftFrontMotorName("front_left_motor")
            .leftRearMotorName("back_left_motor")
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("back_right_motor")

            // STEP 2: FLIP EVERY SINGLE DIRECTION
            // If it was FORWARD, it is now REVERSE.
            // If it was REVERSE, it is now FORWARD.
            // This fixes the X-Axis runaway.
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(-0.0029289532)
            .strafeTicksToInches(-0.002863977438)
            .turnTicksToInches(-.003111037817)
            .leftPodY(7)
            .rightPodY(-7)
            .strafePodX(-3)
            .leftEncoder_HardwareMapName("back_left_motor")
            .rightEncoder_HardwareMapName("back_right_motor")
            .strafeEncoder_HardwareMapName("front_right_motor")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);
}

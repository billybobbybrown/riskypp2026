package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

        public static FollowerConstants followerConstants = new FollowerConstants()
                .mass(17)
                .forwardZeroPowerAcceleration(-33.3)
                .lateralZeroPowerAcceleration(-61.3)

                .useSecondaryTranslationalPIDF(true)
                .useSecondaryHeadingPIDF(true)
                .useSecondaryDrivePIDF(true)
                .centripetalScaling(0.0002)

                /*.translationalPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, .07))
                .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.07))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006, 0, 0.001, 0.01, .06))

                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
                .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.9, 0, 0.01, .02))
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.001, 0, 0.0001, 0.01, .019));
                 */

                .translationalPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, .07))
                .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.07))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006, 0, 0.001, 0.01, .06))

                .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.017))
                .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.9, 0, 0.01, .017))
                .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.001, 0, 0.0001, 0.01, .019));



        public static MecanumConstants driveConstants = new MecanumConstants()
                .maxPower(1)
                .leftFrontMotorName("Frontleft")
                .leftRearMotorName("Backleft")
                .rightFrontMotorName("Frontright")
                .rightRearMotorName("Backright")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                .xVelocity(76.64108060100885)
                .yVelocity(60.894321711983274);


        public static PinpointConstants localizerConstants = new PinpointConstants()
                .forwardPodY(-.75)
                .strafePodX(1.5)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName("Pinpoint")
                //.yawScalar(1.0)
                .encoderResolution(
                        GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                )
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        public static PathConstraints pathConstraints = new PathConstraints(
                0.995,
                500,
                1,
                1
        );

        public static Follower createFollower(HardwareMap hardwareMap) {
            return new FollowerBuilder(followerConstants, hardwareMap)
                    .mecanumDrivetrain(driveConstants)
                    .pinpointLocalizer(localizerConstants)
                    .pathConstraints(pathConstraints)
                    .build();
        }
    }



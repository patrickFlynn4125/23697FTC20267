package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("frontRight");
        c.yPodName.set("backRight");
        c.imuName.set("imu");
        c.xPodOffset.set(0.8323517850291493);
        c.yPodOffset.set(-14.742358396480405);
        c.forwardTicksToInches.set(9.449794946287871E-4);
        c.strafeTicksToInches.set(9.422323728823066E-4);
        c.xPodDirection.set(Encoder.FORWARD);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.214653333079084);
                Controller secondaryTranslationalForward = Controller.proportional(0.0793086709300268);
                Controller primaryTranslationalLateral = Controller.proportional(1.1776351083999326);
                Controller secondaryTranslationalLateral = Controller.proportional(0.4351047055641429);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015788426982528022));
                c.brake.set(Controller.proportionalFeedforward(0.013420162935148819));

                c.headingFeedback.set(Controller.proportional(5.770345939821472));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03905020086724554, 0.010145116484595549));
                c.headingDriveRatio.set(0.25);

                c.linearBrakeCoefficients.set(Matrix.diag(0.035993961490092354, 0.055290934503332535));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002013284531520517, 0.0015903362594962894));

                c.maxAchievableForwardVelocity.set(65.65668684250481);
                c.maxAchievableStrafeVelocity.set(54.95706812434348);
                c.naturalForwardDeceleration.set(60.841562833144565);
                c.naturalStrafeDeceleration.set(80.67226392302543);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new TwoWheelLocalizer(h,localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
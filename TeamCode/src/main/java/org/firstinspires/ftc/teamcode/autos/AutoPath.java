package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(66.4072, 10.2155, 90);
    private final Pose path1 = poseFactory.of(22.524, 22.7844, 90);
    private final Pose path1Control1 = poseFactory.of(38.4476, 5.8201, 0);
    private final Pose point2 = poseFactory.of(22.2949, 119.0442, 90);
    private final Pose point2Control1 = poseFactory.of(0, 46.729, 0);
    private final Pose point2Control2 = poseFactory.of(80.2315, 70.2269, 0);
    private final Pose point2Control3 = poseFactory.of(0, 94.3088, 0);
    private final Pose point3 = poseFactory.of(71.993, 125.1077, 90);
    private final Pose point3Control1 = poseFactory.of(37.5613, 136.6014, 0);
    private final Pose point3Control2 = poseFactory.of(63.8146, 136.6834, 0);
    private final Pose point4 = poseFactory.of(66.3121, 10.3687, 90);
    private final Pose point4Control1 = poseFactory.of(98.1155, 96.8893, 0);
    private final Pose point4Control2 = poseFactory.of(54.842, 75.2328, 0);
    private final Pose point4Control3 = poseFactory.of(92.8176, 16.0365, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2Control3, point2).constant(point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4Control2, point4Control3, point4).constant(point4);
    }
}

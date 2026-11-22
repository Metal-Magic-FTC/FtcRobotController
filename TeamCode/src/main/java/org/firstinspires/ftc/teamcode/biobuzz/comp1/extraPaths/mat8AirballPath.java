package org.firstinspires.ftc.teamcode.biobuzz.comp1.extraPaths;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Auto V2 paths (RED): partner-doesn't-shoot scenario.
 *
 * Default match setup: the red HIVE's AUDIENCE-side CELL starts tilted up (shootable) and its far-side CELL
 * starts tilted down. So the robot starts on the FAR (rear) wall, where it cannot score, waits for the partner
 * to tip, then drives around the red side of the HIVE to the audience side and shoots from there.
 *
 * Coordinates are Pedro visualizer inches: x = 0 is the red wall, y = 0 is the audience wall, field = 141.5 in.
 * Heading 90 = facing +y (toward the far wall), 270 = facing the audience wall.
 */
public class mat8AirballPath {

    private static final PoseFactory P = PoseFactory.degrees();

    public static final double FIELD_SIZE = 141.5;

    private static final double WALL_OFFSET = 6.851;


    public static final Pose START_POSE = P.of(30.725, FIELD_SIZE - WALL_OFFSET, 270);


    public static final Pose SHOOT_POSE = P.of(55.4, 7.8, 90);
    private static final Pose CONTROL = P.of(12.0, 30.0, 0);

    public Path driveToShoot() {
        return curve(START_POSE, CONTROL, SHOOT_POSE).linear(Math.toRadians(270), Math.toRadians(90));
    }
}
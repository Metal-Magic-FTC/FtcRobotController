
package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.hiveShoot;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;

public class HivePath {

    private final PoseFactory poseFactory = PoseFactory.degrees();


    public final Pose point1 = poseFactory.of(23.184, 13.844, 90);
    public final Pose point2 = poseFactory.of(57.082, 28.932, 90);
    public final Pose point3 = poseFactory.of(57.016, 8.466, 90);
    public final Pose point4 = poseFactory.of(8.025, 5.500, 90);
    public final Pose point5 = poseFactory.of(37.123, 117.360, 90);
    public final Pose point6 = poseFactory.of(9.531, 54.217, 90);
    public final Pose point7 = poseFactory.of(41.306, 118.111, 90);

 
    public Path path1() {
        return line(point1, point2).linear(point1, point2);
    }


    public Path path2() {
        return line(point2, point3).linear(point2, point3);
    }


    public Path path3() {
        return line(point3, point4).linear(point3, point4);
    }


    public Path path4() {
        return line(point4, point5).linear(point4, point5);
    }


    public Path path5() {
        return line(point5, point6).linear(point5, point6);
    }


    public Path path6() {
        return line(point6, point7).linear(point6, point7);
    }
}

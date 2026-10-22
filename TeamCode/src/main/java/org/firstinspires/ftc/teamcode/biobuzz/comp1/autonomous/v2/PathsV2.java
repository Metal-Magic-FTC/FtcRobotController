package org.firstinspires.ftc.teamcode.biobuzz.comp1.autonomous.v2;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Auto V2 paths, exported from the Pedro visualizer (class renamed from Paths, start pose made public).
 */
public class PathsV2 {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static final Pose start = poseFactory.of(38.2155, 8.1302, 180);
    private final Pose point11Shootpreloadsthensweepgarden = poseFactory.of(9, 7.5, 180);
    private final Pose point22Runtofarwallflower = poseFactory.of(48, 131, 90);
    private final Pose point22RuntofarwallflowerControl1 = poseFactory.of(22, 40, 0);
    private final Pose point22RuntofarwallflowerControl2 = poseFactory.of(26, 112, 0);
    private final Pose point33Toleftwallflower = poseFactory.of(12.5, 49, 180);
    private final Pose point33ToleftwallflowerControl1 = poseFactory.of(30, 118, 0);
    private final Pose point33ToleftwallflowerControl2 = poseFactory.of(14, 80, 0);
    private final Pose point44Dropintonearband = poseFactory.of(27.3851, 18.527, 180);
    private final Pose point55Parkinloadingzone = poseFactory.of(10, 104, 180);
    private final Pose point55ParkinloadingzoneControl1 = poseFactory.of(14, 75, 0);

    public Path path11Shootpreloadsthensweepgarden() {
        return line(start, point11Shootpreloadsthensweepgarden).linear(start, point11Shootpreloadsthensweepgarden);
    }

    public Path path22Runtofarwallflower() {
        return curve(point11Shootpreloadsthensweepgarden, point22RuntofarwallflowerControl1, point22RuntofarwallflowerControl2, point22Runtofarwallflower).linear(point11Shootpreloadsthensweepgarden, point22Runtofarwallflower);
    }

    public Path path33Toleftwallflower() {
        return curve(point22Runtofarwallflower, point33ToleftwallflowerControl1, point33ToleftwallflowerControl2, point33Toleftwallflower).linear(point22Runtofarwallflower, point33Toleftwallflower);
    }

    public Path path44Dropintonearband() {
        return line(point33Toleftwallflower, point44Dropintonearband).linear(point33Toleftwallflower, point44Dropintonearband);
    }

    public Path path55Parkinloadingzone() {
        return curve(point44Dropintonearband, point55ParkinloadingzoneControl1, point55Parkinloadingzone).linear(point44Dropintonearband, point55Parkinloadingzone);
    }
}

package org.firstinspires.ftc.teamcode.decode.teleOp.statesTele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Alliance;

@TeleOp(name="RedTele")
public class Red extends Tele {
    public Red() {
        super(Alliance.RED);
    }
}
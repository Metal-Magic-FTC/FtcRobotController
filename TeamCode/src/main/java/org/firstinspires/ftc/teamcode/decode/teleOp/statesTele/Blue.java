package org.firstinspires.ftc.teamcode.decode.teleOp.statesTele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.pedroPathing.Alliance;

@TeleOp(name="BlueTele")
public class Blue extends Tele {
    public Blue() {
        super(Alliance.BLUE);
    }
}
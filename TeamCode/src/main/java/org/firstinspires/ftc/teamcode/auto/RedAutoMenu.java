package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;

import java.util.List;

/**
 * Red alliance, with the auto chosen during init; see {@link AutoMenuBase}. The options are
 * {@link AutoMenus#RED}. The status line shows what vision reads for the target until the
 * routine decides whether to shoot. Driving and parking need Pedro (a localizer and Foresight).
 */
@Autonomous(name = "Red: Auto Menu", group = "BIOBUZZ")
public class RedAutoMenu extends AutoMenuBase {

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected Cell target() {
        return Cell.RED_AUDIENCE;
    }

    @Override
    List<AutoOption> options() {
        return AutoMenus.RED;
    }
}

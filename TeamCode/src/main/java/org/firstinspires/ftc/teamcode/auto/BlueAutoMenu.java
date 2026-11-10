package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;

import java.util.List;

/**
 * Blue alliance, with the auto chosen during init; see {@link AutoMenuBase}. The options are
 * {@link AutoMenus#BLUE}. The status line shows what vision reads for the target until the
 * routine decides whether to shoot. Driving and parking need Pedro (a localizer and Foresight).
 */
@Autonomous(name = "Blue: Auto Menu", group = "BIOBUZZ")
public class BlueAutoMenu extends AutoMenuBase {

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Cell target() {
        return Cell.BLUE_FAR;
    }

    @Override
    List<AutoOption> options() {
        return AutoMenus.BLUE;
    }
}

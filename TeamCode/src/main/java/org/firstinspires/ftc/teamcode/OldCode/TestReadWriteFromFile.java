package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.io.File;

/**
 * Created by Jyoti on 12/9/2016.
 * The name is not accurate. This file writes to a file called configuration.txt.
 * You can see the file in Phone/FIRST/settings/configuration.txt
 */

@Autonomous(group = "test", name = "File Write test")
public class TestReadWriteFromFile extends LinearVisionOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        String filename = "configuration.txt";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, "jyoti");
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.io.File;

/**
 * Created by Jyoti on 12/9/2016.
 * This class reads from the file configuration.txt in Phone/FIRST/settings.
 */

@Autonomous(group = "test", name = "File Read test")
public class TestReadingFromFile extends LinearVisionOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        String filename = "configuration.txt";
        File file = AppUtil.getInstance().getSettingsFile(filename);

        String s = ReadWriteFile.readFile(file);
        telemetry.addData("data read",s );
    }
}
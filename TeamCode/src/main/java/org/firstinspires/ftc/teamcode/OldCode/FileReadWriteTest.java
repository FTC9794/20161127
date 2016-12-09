package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import java.io.File;

/**
 * Created by Ishaan Oberoi on 12/8/2016.
 */

public class FileReadWriteTest {

    static String filename = "AutoAngle.txt";
    static File myFile = new File(filename);

    static void writeToFile() {
        ReadWriteFile.writeFile(myFile, "Jyoti");
    }

    static String voidReadFromFile() {
        return ReadWriteFile.readFile(myFile);
    }
}

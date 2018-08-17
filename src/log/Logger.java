package log;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;

public class Logger {

    public static void write(String prefix, String message) {
        try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/log.txt", true))) {
            writer.println("-------------------------------");
            writer.print(new Date().toString());
            writer.println(prefix + message); 
            writer.println("-------------------------------");
        } catch (IOException e) {

        }
    }

}

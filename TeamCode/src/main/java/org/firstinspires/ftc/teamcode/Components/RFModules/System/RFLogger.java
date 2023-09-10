package org.firstinspires.ftc.teamcode.Components.RFModules.System;

import android.annotation.SuppressLint;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class RFLogger {
    public static Logger LOGGER;
//    HashMap<String, FileHandler> fileMap = new HashMap<>();
    FileHandler fh;
    SimpleFormatter sh = new SimpleFormatter();
    EnumMap<File, FileHandler> fileMap = new EnumMap<>(File.class);
    public static enum File {
        GeneralLog,
        AutoLog,
        HardwareLog,
        QueuerLog;
    }
    public RFLogger (String className){
        LOGGER = Logger.getLogger(className);
        LOGGER.setLevel(Level.ALL);
        try {
            fh = new FileHandler("/sdcard/tmp/MotorLogs");
        } catch (IOException e) {
            e.printStackTrace();
        }
        fh.setFormatter(sh);
        fileMap.put(File.GeneralLog,fh);
        LOGGER.addHandler(fh);
    }

    @SuppressLint("SdCardPath")
    public void createFile (String p_fileName) {
        try {
            fh = new FileHandler("/sdcard/tmp/"+p_fileName+"Log.csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
        fh.setFormatter(sh);
        fileMap.put(File.GeneralLog, fh);
    }

    public void setFile(String p_fileName){
        LOGGER.addHandler(fileMap.get("/sdcard/tmp/"+p_fileName+"Log.csv"));
    }



    public void logSevere(String error){
        LOGGER.severe(error);
    }

    public void logWarning(String warning){
        LOGGER.warning(warning);
    }

    public void logInfo(String info){
        LOGGER.info(info);
    }

    public void logConfig(String config){
        LOGGER.config(config);
    }

    public void logFine(String logMsg){
        LOGGER.fine(logMsg);
    }

    public void logFiner(String logMsg){
        LOGGER.finer(logMsg);
    }

    public void logFinest(String logMsg){
        LOGGER.finest(logMsg);
    }
}

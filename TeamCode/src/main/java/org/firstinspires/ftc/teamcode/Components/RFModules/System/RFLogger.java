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
    Level logLevel = Level.ALL;
    public enum Files {
        GENERAL_LOG("/sdcard/tmp/GeneralLog.csv"),
        AUTONOMOUS_LOG("/sdcard/tmp/AutonomousLog.csv"),
        HARDWARE_LOG("/sdcard/tmp/HardwareLog.csv"),
        QUEUER_LOG("/sdcard/tmp/QueuerLog.csv");

        String filePath;

        Files(String p_filePath){
            filePath = p_filePath;
        }
    }

    public enum Severity {
        ALL(Level.ALL),
        SEVERE(Level.SEVERE),
        WARNING(Level.WARNING),
        INFO(Level.INFO),
        CONFIG(Level.CONFIG),
        FINE(Level.FINE),
        FINER(Level.FINER),
        FINEST(Level.FINEST);

        Level logSeverity;

        Severity(Level p_logSeverity){
            logSeverity = p_logSeverity;
        }
    }
    public RFLogger (String className){
        LOGGER = Logger.getLogger(className);
        LOGGER.setLevel(logLevel);
        try {
            fh = new FileHandler(Files.GENERAL_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
        fh.setFormatter(sh);
        LOGGER.addHandler(fh);
    }

//    @SuppressLint("SdCardPath")
//    public void createFile (String p_fileName) {
//        try {
//            fh = new FileHandler("/sdcard/tmp/"+p_fileName+"Log.csv");
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//        fh.setFormatter(sh);
//        fileMap.put(File.GeneralLog, fh);
//    }

//    public void setFile(String p_fileName){
//        LOGGER.addHandler(fileMap.get("/sdcard/tmp/"+p_fileName+"Log.csv"));
//    }
    public void setLogLevel(Severity p_severity){
        logLevel = p_severity.logSeverity;
        LOGGER.setLevel(logLevel);
    }

    public void log(Severity p_severity, Files p_file, String info){
        try {
            fh = new FileHandler(p_file.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
        fh.setFormatter(sh);
        LOGGER.addHandler(fh);
        logLevel = p_severity.logSeverity;
        LOGGER.log(logLevel, info);
    }

    public void log(Files p_file, String info){
        try {
            fh = new FileHandler(p_file.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
        fh.setFormatter(sh);
        LOGGER.addHandler(fh);
        LOGGER.log(logLevel, info);
    }

    public void log(String info){
        try {
            fh = new FileHandler(Files.GENERAL_LOG.filePath);
        } catch (IOException e) {
            e.printStackTrace();
        }
        fh.setFormatter(sh);
        LOGGER.addHandler(fh);
        LOGGER.log(logLevel, info);
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

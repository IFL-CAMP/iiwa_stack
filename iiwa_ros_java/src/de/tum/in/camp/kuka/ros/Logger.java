/**
 * Copyright (C) 2018 Arne Peters - arne.peters@tum.de 
 * Technische Universität München
 * Chair for Robotics, Artificial Intelligence and Embedded Systems 
 * Fakultät für Informatik / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany 
 * http://www6.in.tum.de 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided
 * that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package de.tum.in.camp.kuka.ros;

import org.apache.commons.logging.Log;

import com.kuka.task.ITaskLogger;

public class Logger {
  public static enum Level {
    DEBUG(4), INFO(3), WARN(2), ERROR(1), FATAL(0);

    private final int value;

    private Level(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  public static enum Target {
    BOTH, SUNRISE, ROS
  }

  private static ITaskLogger sunriseLogger = null;
  private static Log rosLogger = null;
  private static Level logLevel = Level.INFO;

  public static void setRosLogger(Log logger) {
    rosLogger = logger;
  }

  public static void setSunriseLogger(ITaskLogger logger) {
    sunriseLogger = logger;
  }

  public static void debug(String message) {
    log(message, Level.DEBUG, Target.BOTH);
  }

  public static void info(String message) {
    log(message, Level.INFO, Target.BOTH);
  }

  public static void warn(String message) {
    log(message, Level.WARN, Target.BOTH);
  }

  public static void error(String message) {
    log(message, Level.ERROR, Target.BOTH);
  }

  public static void fatal(String message) {
    log(message, Level.FATAL, Target.BOTH);
  }

  public static void log(String message, Level level, Target target) {
    if (level.getValue() <= logLevel.getValue()) {
      if (target == Target.BOTH || target == Target.SUNRISE) {
        if (sunriseLogger != null) {
          switch (level) {
            case DEBUG:
              sunriseLogger.fine(message); // fine does not get displayed on SmartPad
              sunriseLogger.info(message);
              break;
            case INFO:
              sunriseLogger.info(message);
              break;
            case WARN:
              sunriseLogger.warn(message);
              break;
            case ERROR:
              sunriseLogger.error(message);
              break;
            case FATAL:
              sunriseLogger.error(message);
              break;
          }
        }
      }

      if (target == Target.BOTH || target == Target.ROS) {
        if (rosLogger != null) {
          switch (level) {
            case DEBUG:
              rosLogger.debug(message);
              break;
            case INFO:
              rosLogger.info(message);
              break;
            case WARN:
              rosLogger.warn(message);
              break;
            case ERROR:
              rosLogger.error(message);
              break;
            case FATAL:
              rosLogger.error(message);
              break;
          }
        }
      }
    }
  }

  public static void setLogLevel(Level level) {
    Logger.logLevel = level;
    debug("Setting logger level to " + level);

    // TODO: change setting of sunrise and ROS logger to display log messages accordingly
  }

  public static Level getLogLevel() {
    return logLevel;
  }
}

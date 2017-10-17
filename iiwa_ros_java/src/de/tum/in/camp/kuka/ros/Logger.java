package de.tum.in.camp.kuka.ros;

import org.apache.commons.logging.Log;

import com.kuka.task.ITaskLogger;

public class Logger {
	public static enum Level {
		DEBUG,
		INFO,
		WARN,
		ERROR,
		FATAL
	}

	public static enum Target {
		BOTH,
		SUNRISE,
		ROS
	}

	private static ITaskLogger sunriseLogger = null;
	private static Log rosLogger = null;

	public static void setRosLogger(Log logger) {
		rosLogger = logger;
	}

	public static void setSunriseLogger(ITaskLogger logger) {
		sunriseLogger = logger;
	}

	public static void debug(String message) {
		log(message, Level.DEBUG, Target.ROS);
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
		if (target == Target.BOTH || target == Target.SUNRISE) {
			if (sunriseLogger != null) {
				switch (level) {
				case DEBUG:
					sunriseLogger.fine(message);
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



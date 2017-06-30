package de.tum.in.camp.kuka.ros;

public class UnsupportedControlModeException extends RuntimeException {
	private static final long serialVersionUID = 1L;
	public UnsupportedControlModeException() { super(); }
	public UnsupportedControlModeException(String message) { super(message); }
	public UnsupportedControlModeException(String message, Throwable cause) { super(message, cause); }
	public UnsupportedControlModeException(Throwable cause) { super(cause); }
}

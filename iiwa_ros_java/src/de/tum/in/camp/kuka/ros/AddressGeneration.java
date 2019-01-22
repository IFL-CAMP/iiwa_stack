package de.tum.in.camp.kuka.ros;

public class AddressGeneration {
	static int address = 30000;
	
	public static int getNewAddress() {
		int newAddress = address;
		address++;
		return newAddress;
	}
}

#pragma once

#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/CartesianQuantity.h>

namespace iiwa_ros {
	
	
	/**
	 * @brief Creates a JointQuantity with the same value in all its components.
	 * 
	 * @param value the value to use for all the JointQuantity components.
	 * @return iiwa_msgs::JointQuantity
	 */
	iiwa_msgs::JointQuantity jointQuantityFromDouble(const double value) {
		iiwa_msgs::JointQuantity quantity;
		quantity.a1 = value;
		quantity.a2 = value;
		quantity.a3 = value;
		quantity.a4 = value;
		quantity.a5 = value;
		quantity.a6 = value;
		quantity.a7 = value;
		return quantity;
	}
	
	/**
	 * @brief Creates a JointQuantity with the given values for as components.
	 * 
	 * @param a1
	 * @param a2
	 * @param a3
	 * @param a4
	 * @param a5
	 * @param a6
	 * @param a7
	 * @return iiwa_msgs::JointQuantity
	 */
	iiwa_msgs::JointQuantity jointQuantityFromDouble(const double a1, const double a2, const double a3, const double a4, const double a5, const double a6, const double a7) {
		iiwa_msgs::JointQuantity quantity;
		quantity.a1 = a1;
		quantity.a2 = a2;
		quantity.a3 = a3;
		quantity.a4 = a4;
		quantity.a5 = a5;
		quantity.a6 = a6;
		quantity.a7 = a7;
		return quantity;
	}
	
	/**
	 * @brief Creates a CartesianQuantity with the same value in all its components.
	 * 
	 * @param value the value to use for all the CartesianQuantity components.
	 * @return iiwa_msgs::CartesianQuantity
	 */
	iiwa_msgs::CartesianQuantity CartesianQuantityFromDouble(const double value) {
		iiwa_msgs::CartesianQuantity quantity;
		quantity.x = value;
		quantity.y = value;
		quantity.z = value;
		quantity.a = value;
		quantity.b = value;
		quantity.c = value;
		return quantity;
	}
	
	/**
	 * @brief Creates a CartesianQuantity with the given values for as components.
	 * 
	 * @param x 
	 * @param y 
	 * @param z 
	 * @param a 
	 * @param b 
	 * @param c 
	 * @return iiwa_msgs::CartesianQuantity
	 */
	iiwa_msgs::CartesianQuantity CartesianQuantityFromDouble(const double x, const double y, const double z, const double a, const double b, const double c) {
		iiwa_msgs::CartesianQuantity quantity;
		quantity.x = x;
		quantity.y = y;
		quantity.z = z;
		quantity.a = a;
		quantity.b = b;
		quantity.c = c;
		return quantity;
	}
	
	/**
	 * @brief Creates a CartesianQuantity with the given values for its translational and rotational component respectively.
	 * 
	 * @param translation_value value to use for all the transflational components (x,y,z) of the CartesianQuantity
	 * @param rotation_value value to use for all the rotational components (a,b,c) of the CartesianQuantity
	 * @return iiwa_msgs::CartesianQuantity
	 */
	iiwa_msgs::CartesianQuantity CartesianQuantityFromDouble(const double translation_value, const double rotation_value) {
		iiwa_msgs::CartesianQuantity quantity;
		quantity.x = translation_value;
		quantity.y = translation_value;
		quantity.z = translation_value;
		quantity.a = rotation_value;
		quantity.b = rotation_value;
		quantity.c = rotation_value;
		return quantity;
	}
}
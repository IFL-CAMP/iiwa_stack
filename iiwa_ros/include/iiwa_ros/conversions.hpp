/**
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <iiwa_msgs/CartesianQuantity.h>
#include <iiwa_msgs/JointQuantity.h>

#include <ros/ros.h>

namespace iiwa_ros {
namespace conversions {

/**
 * @brief Creates a JointQuantity with the same value in all its components.
 *
 * @param [in] value - the value to use for all the JointQuantity components.
 */
iiwa_msgs::JointQuantity jointQuantityFromFloat(const float value) {
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
 * @brief Creates a JointQuantity with the given the values of its components.
 */
iiwa_msgs::JointQuantity jointQuantityFromFloat(const float a1, const float a2, const float a3, const float a4,
                                                const float a5, const float a6, const float a7) {
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


template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
/**
 * @brief Converts a JointQuantity message to a std::vector<T>. T must be a numberic type.
 */
std::vector<T> jointQuantityToVector(const iiwa_msgs::JointQuantity& quantity) {
  return {quantity.a1, quantity.a2, quantity.a3, quantity.a4, quantity.a5, quantity.a6, quantity.a7};
}

template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
/**
 * @brief Convers an std::vector<T> to a JointQuantity message. T must be a numberic type.
 */
iiwa_msgs::JointQuantity jointQuantityFromVector(const std::vector<T>& v) {
  iiwa_msgs::JointQuantity return_value;
  return_value.a1 = v[0];
  return_value.a2 = v[1];
  return_value.a3 = v[2];
  return_value.a4 = v[3];
  return_value.a5 = v[4];
  return_value.a6 = v[5];
  return_value.a7 = v[6];
  return return_value;
}

/**
 * @brief Creates a CartesianQuantity with the same value in all its components.
 *
 * @param [in] value - the value to use for all the CartesianQuantity components.
 */
iiwa_msgs::CartesianQuantity CartesianQuantityFromFloat(const float value) {
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
 * @brief Creates a CartesianQuantity with the given values of its components.
 */
iiwa_msgs::CartesianQuantity CartesianQuantityFromFloat(const float x, const float y, const float z, const float a,
                                                        const float b, const float c) {
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
 * @param [in] translation_value - value to use for all the transflational components (x,y,z) of the CartesianQuantity.
 * @param [in] rotation_value - value to use for all the rotational components (a,b,c) of the CartesianQuantity.
 */
iiwa_msgs::CartesianQuantity CartesianQuantityFromFloat(const float translation_value, const float rotation_value) {
  iiwa_msgs::CartesianQuantity quantity;
  quantity.x = translation_value;
  quantity.y = translation_value;
  quantity.z = translation_value;
  quantity.a = rotation_value;
  quantity.b = rotation_value;
  quantity.c = rotation_value;
  return quantity;
}

}  // namespace conversions
}  // namespace iiwa_ros

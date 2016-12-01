/*
 * type_macros.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once


//-------------------------------------------------------------
//                DEFINE SUPPORTED TYPES
//-------------------------------------------------------------

#define PH_ROS_INTEGRAL_SCALAR_TYPES bool, char, char16_t, char32_t, wchar_t, signed char, short int, \
    int, long int, unsigned char, unsigned short int

#define PH_ROS_INTEGRAL_MATRIX_TYPES Eigen::Vector3i

#define PH_ROS_INTEGRAL_TYPES PH_ROS_INTEGRAL_SCALAR_TYPES, PH_ROS_INTEGRAL_MATRIX_TYPES

#define PH_ROS_FLOATING_POINT_SCALAR_TYPES double, float

#define PH_ROS_FLOATING_POINT_MATRIX_TYPES Eigen::Vector3d, Eigen::Matrix3d

#define PH_ROS_FLOATING_POINT_TYPES PH_ROS_FLOATING_POINT_SCALAR_TYPES, PH_ROS_FLOATING_POINT_MATRIX_TYPES

//-------------------------------------------------------------

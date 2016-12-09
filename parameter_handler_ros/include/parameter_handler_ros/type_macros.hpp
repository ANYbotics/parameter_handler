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

#define PH_ROS_INTEGRAL_MATRIX_TYPES Eigen::Matrix3i, Eigen::MatrixXi, Eigen::Vector2i, Eigen::Vector3i, Eigen::Vector4i, Eigen::VectorXi

#define PH_ROS_INTEGRAL_TYPES PH_ROS_INTEGRAL_SCALAR_TYPES, PH_ROS_INTEGRAL_MATRIX_TYPES

#define PH_ROS_FLOATING_POINT_SCALAR_TYPES double, float

#define PH_ROS_FLOATING_POINT_MATRIX_TYPES Eigen::Matrix3f, Eigen::MatrixXf, Eigen::Vector2f, Eigen::Vector3f, Eigen::Vector4f, Eigen::VectorXf, \
    Eigen::Matrix3d, Eigen::MatrixXd, Eigen::Vector2d, Eigen::Vector3d, Eigen::Vector4d, Eigen::VectorXd

#define PH_ROS_FLOATING_POINT_TYPES PH_ROS_FLOATING_POINT_SCALAR_TYPES, PH_ROS_FLOATING_POINT_MATRIX_TYPES

//-------------------------------------------------------------

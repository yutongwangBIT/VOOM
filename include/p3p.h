/*
 * Copyright (c) 2011, Laurent Kneip, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * p3p.h
 *
 *       Created on: Aug 6, 2013
 *  Original Author: Laurent Kneip
 *  Modifications by: Karl Schwabe
 *       Description: Compute the absolute pose of a camera using three 3D-to-2D correspondences.
 *    Modifications form original:
 *                   Using the Eigen linear algebra library instead of the TooN library that Laurent Kneip used in his original implementation.
 *                   Output data as a vector of poses, as opposed to the large 3x16 matrix output by original implementation by Laurent Kneip.
 *
 *       Reference: A Novel Parametrization of the P3P-Problem for a Direct Computation of
 *                  Absolute Camera Position and Orientation
 *
 *           Input: featureVectors: 3x3 matrix with UNITARY feature vectors (each column is a vector)
 *                   worldPoints: 3x3 matrix with corresponding 3D world points (each column is a point)
 *                   solutions: A vector of 4 elements where each element is a 3x4 matrix that contains the solutions
 *                                      of the form:
 *                                      [[3x4] (3x3 orientation and 3x1 position (solution1));
 *                                       [3x4] (3x3 orientation and 3x1 position (solution2));
 *                                       [3x4] (3x3 orientation and 3x1 position (solution3));
 *                                       [3x4] (3x3 orientation and 3x1 position (solution4))]
 *                               The obtained orientation matrices are defined as transforming points from the camera to the world frame
 *          Output: int: 0 if correct execution
 *                  -1 if world points aligned/colinear
 */

/**
 * \file p3p.h
 * \brief File containing the includes and the function prototypes for the p3p.cpp file.
 *
 */

#ifndef P3P_H_
#define P3P_H_

#include <Eigen/Dense>
#include <math.h>
#include <stdlib.h>

namespace monocular_pose_estimator
{

/**
 * The P3P class that solves the Perspective from Three Points (P3P) problem.
 *
 * The absolute pose of a camera is calculated using three 3D-to-2D correspondences using an implementation of Laurent Kneip's P3P algorithm.
 *
 * \note The Eigen linear algebra library is used instead of the TooN library that Laurent Kneip used in his original implementation.
 *
 * \author Karl Schwabe
 * \author Laurent Kneip (original author) (http://www.laurentkneip.de)
 *
 * \cite Kneip:2011 Reference: A Novel Parametrization of the Perspective-Three-Point Problem for a Direct Computation of Absolute Camera Position and Orientation. DOI: 10.1109/CVPR.2011.5995464
 *
 *
 */
class P3P
{
public:

  /**
   * Solves the P3P problem.
   *
   * Computes the four possible poses for the P3P problem, given three 3D-to-2D point correspondences.
   *
   * For further details on how the algorithm works, see \cite Kneip:2011.
   *
   * \param feature_vectors a 3x3 matrix containing the unit vectors that point from the centre of projection of the camera to the world points used in the P3P algorithm. Each column of the matrix represents a unit vector.
   * \param world_points the 3D coordinates of the world points that are used in the P3P algorithm
   * \param solutions (output) the four solutions to the P3P problem. It is passed as a vector of 4 elements, where each element is a 3x4 matrix that contains the solutions of the form: \n
   *                             [ [3x4] (3x3 rotation matrix and 3x1 position vector (solution1));\n
   *                               [3x4] (3x3 rotation matrix and 3x1 position vector (solution2));\n
   *                               [3x4] (3x3 rotation matrix and 3x1 position vector (solution3));\n
   *                               [3x4] (3x3 rotation matrix and 3x1 position vector (solution4)) ]\n
   *                             The obtained orientation matrices are defined as transforming points from the camera to the world frame
   *
   * \returns
   * - \b 0 if executed correctly
   * - \b -1 if the world points are colinear and it was unable to solve the P3P problem
   *
   */
  static int computePoses(const Eigen::Matrix3d & feature_vectors, const Eigen::Matrix3d & world_points,
                          Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> & solutions);

  /**
   * Solves a quartic equation.
   *
   * Computes the solution to the quartic equation \f$a_4 \cdot x^4 + a_3 \cdot x^3 + a_2 \cdot x^2 + a_1 \cdot x + a_0 = 0\f$ using Ferrari's closed form solution for finding the roots of a fourth order polynomial.
   *
   * For further details on how the algorithm works, see \cite Kneip:2011.
   *
   * \param factors vector of the coefficients of \f$x\f$, listed in decending powers of \f$x\f$, i.e., factors(0)&nbsp;=&nbsp;\f$a_4\f$, factors(1)&nbsp;=&nbsp;\f$a_3\f$, factors(2)&nbsp;=&nbsp;\f$a_2\f$, factors(3)&nbsp;=&nbsp;\f$a_1\f$, and factors(4)&nbsp;=&nbsp;\f$a_0\f$
   * \param real_roots (output) vector containing the four solutions to the quartic equation
   *
   * \returns
   * \b 0 if executed correctly
   *
   */
  static int solveQuartic(const Eigen::Matrix<double, 5, 1> & factors, Eigen::Matrix<double, 4, 1> & real_roots);
};

}

#endif /* P3P_H_ */
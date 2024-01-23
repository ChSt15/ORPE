/*
 * typedefs.h
 *
 *  Created on: Oct 14, 2016
 *      Author: strohmi
 */

#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include "matlib/api/generic.h"
#include "matlib/api/matrix.h"
#include "matlib/api/vector.h"
#include "matlib/api/quaternion.h"
#include "matlib/api/ypr.h"
#include "matlib/api/angleaxis.h"
#include "matlib/api/coordinateframe3d.h"
#include "matlib/api/polar.h"
#include "matlib/api/complex.h"
#include "matlib/api/homogenous.h"

#ifndef NO_RODOS_NAMESPACE
namespace RODOS {
#endif

typedef AngleAxis_<double> AngleAxis;
typedef AngleAxis_<float>  AngleAxis_F;

typedef Complex_<double> Complex;
typedef Complex_<float>  Complex_F;

typedef CoordinateFrame3D_<double> CoordinateFrame3D;
typedef CoordinateFrame3D_<float>  CoordinateFrame3D_F;

typedef HCoord_<double> HCoord;
typedef HCoord_<float>  HCoord_F;

typedef HTransform_<double> HTransform;
typedef HTransform_<float>  HTransform_F;

typedef Matrix3D_<double> Matrix3D;
typedef Matrix3D_<float>  Matrix3D_F;
typedef Matrix6D_<double> Matrix6D;
typedef Matrix6D_<float>  Matrix6D_F;

template <size_t ROW, size_t COL>
using Matrix = Matrix_<ROW,COL,double>;
template <size_t ROW, size_t COL>
using Matrix_F = Matrix_<ROW,COL,float>;

typedef Polar_<double> Polar;
typedef Polar_<float>  Polar_F;

typedef Quaternion_<double> Quaternion;
typedef Quaternion_<float>  Quaternion_F;

typedef Vector3D_<double> Vector3D;
typedef Vector3D_<float>  Vector3D_F;
typedef Vector6D_<double> Vector6D;
typedef Vector6D_<float>  Vector6D_F;

template <size_t ROW>
using Vector = Vector_<ROW,double>;
template <size_t ROW>
using Vector_F = Vector_<ROW,float>;

typedef YPR_<double> YPR;
typedef YPR_<float>  YPR_F;

#ifndef NO_RODOS_NAMESPACE
}
#endif



#endif /* TYPEDEFS_H_ */

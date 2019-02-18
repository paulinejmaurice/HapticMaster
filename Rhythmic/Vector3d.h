//*********************************************************************
// Copyright © 2007 Fokker Control Systems B.V.
//*********************************************************************
//
// file    : Vector3d.h
// purpose : 3D vector struct
//
// --------------------------------------------------------------------
//  2008-08-17 PL 3.1.0 "normalize" now returns booleans, zero protect
//                      added set() function
//  2008-08-17 PL 3.0.1 replaced tabs by 3 spaces
//  2007-11-21 PL 3.0.0 added zero and component product
//  2007-01-23 PL 1.0 0 created, using elements from CHAI and H3D
// --------------------------------------------------------------------
//
// History logging by RCS
//*********************************************************************

#ifndef VECTOR3D_H_
#define VECTOR3D_H_

#include "math.h"

//---------------------------------------------------------------------
//
//    struct Vector3d
//
//---------------------------------------------------------------------

struct Vector3d
{
   double x, y, z;

   // -----------------------------------------------------------------
   // constructors
   // -----------------------------------------------------------------

   Vector3d( void )
   {
      x = 0.0; y = 0.0; z = 0.0;
   }

   Vector3d( const Vector3d &v )
   {
      x = v.x; y = v.y; z = v.z;
   }

   Vector3d( const double xNew,
             const double yNew,
             const double zNew)
   {
      x = xNew; y = yNew; z = zNew;
   }

   Vector3d( const double a[3] )
   {
      x = a[0]; y = a[1]; z = a[2];
   }

   Vector3d( const double*& a )
   {
      x = a[0]; y = a[1]; z = a[2];
   }

   // -----------------------------------------------------------------
   // note:
   //    Assignment between vectors with the = operator is
   //    member by member. This is an automatic compiler default in C++.
   // -----------------------------------------------------------------

   // -----------------------------------------------------------------
   // cast to a double* (array)
   // -----------------------------------------------------------------

   operator double* ()
   { return &x; }

   operator const double* () const
   { return (const double*) &x; }

   // TODO - find a solution to call a get function
   //        defined with Vector3d& with a double[3] argument

   // -----------------------------------------------------------------
   // assigment (element copy) from and to array
   // -----------------------------------------------------------------

   inline void operator=( const double a[3] )
   {
      x = a[0];
      y = a[1];
      z = a[2];
   }

   inline void operator=( const float a[3] )
   {
      x = a[0];
      y = a[1];
      z = a[2];
   }

   inline void operator+=( const Vector3d a )
    {
      x += a[0];
      y += a[1];
      z += a[2];
   }

   inline void copyTo( double a[3] )
   {
      a[0] = x;
      a[1] = y;
      a[2] = z;
   }

   inline void copyTo( float a[3] )
   {
      a[0] = static_cast<float>(x);
      a[1] = static_cast<float>(y);
      a[2] = static_cast<float>(z);
   }

   // -----------------------------------------------------------------
   // array-like element referencing
   // -----------------------------------------------------------------

   inline double &operator[]( int i )
   {
      if( i <= 0 ) return x;
      if( i == 1 ) return y;
      return z;
   }

   inline const double &operator[]( int i ) const {
      if( i <= 0 ) return x;
      if( i == 1 ) return y;
      return z;
   }

   // -----------------------------------------------------------------
   // length and norm, zero
   // -----------------------------------------------------------------

   inline double lengthSq()
   {
      return x*x + y*y + z*z;
   }

   inline double length()
   {
      return sqrt( x*x + y*y + z*z );
   }

   inline bool normalizeFromLength( const double inLength )
   {
      if ( inLength <= 0.0 ) {
         return false;
      }
      x /= inLength;
      y /= inLength;
      z /= inLength;
      return true;
   }

   inline bool normalize()
   {
      double lengthSq = x*x + y*y + z*z;

      if ( lengthSq <= 0.0 ) {
         return false;
      }

      double length = sqrt( lengthSq );
      x /= length;
      y /= length;
      z /= length;
      return true;
   }

   inline double normalizeReturnLength()
   {
      double lengthSq = x*x + y*y + z*z;
      if ( lengthSq <= 0.0 ) {
         return 0.0;
      }
      double length = sqrt( lengthSq );
         x /= length;
         y /= length;
         z /= length;
      return length;
   }
   
   Vector3d normalized()
   {
      Vector3d outVec( x, y, z);
      outVec.normalize();
      return outVec;
   }
   
   inline void zero() {
      x = 0.0;
      y = 0.0;
      z = 0.0;
   }

   inline void set(  const double inX,
                     const double inY,
                     const double inZ )
   {
      x = inX;
      y = inY;
      z = inZ;
   }
};

// end of struct Vector3d
// --------------------------------------------------------------------


//---------------------------------------------------------------------
//    OVERLOADED OPERATORS
// note:
//    All of these operators *return* results (via the stack).
//    Hopefully this does not hurt performance too much.
//    For assigmnents ( - ) see inside struct above.
//---------------------------------------------------------------------


inline bool operator==( const Vector3d &v1, const Vector3d &v2 )
{
   return ( v1.x == v2.x
         && v1.y == v2.y
         && v1.z == v2.z );
}

inline Vector3d operator+( const Vector3d &v1, const Vector3d &v2 )
{
   return Vector3d( v1.x + v2.x, v1.y + v2.y, v1.z + v2.z );
}

inline Vector3d operator-( const Vector3d &v1, const Vector3d &v2 )
{
   return Vector3d(  v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

inline Vector3d operator*( const Vector3d &v, const double &a )
{
   return Vector3d( v.x * a, v.y * a, v.z * a );
}

inline Vector3d operator*( const double &a, const Vector3d &v1 )
{
   return Vector3d( v1.x * a, v1.y * a, v1.z * a );
}

// unary minus
inline Vector3d operator-( const Vector3d &v )
{
   return Vector3d(  -v.x, -v.y, -v.z );
}

//---------------------------------------------------------------------
//    PRODUCTS
//       dotProduct, crossProduct, componentProduct
//---------------------------------------------------------------------

inline double dotProduct( const Vector3d &v1,
                          const Vector3d &v2 )
{
   return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

inline double operator*( const Vector3d &v1,
                         const Vector3d &v2 )
{
   return dotProduct( v1, v2 );
}

inline Vector3d crossProduct( const Vector3d &v1,
                              const Vector3d &v2 )
{
   return Vector3d( v1.y * v2.z - v1.z * v2.y,
                    v1.z * v2.x - v1.x * v2.z,
                    v1.x * v2.y - v1.y * v2.x );
}

inline Vector3d componentProduct( const Vector3d &v1,
                                  const Vector3d &v2 )
{
   return  Vector3d( v1.x * v2.x,
                     v1.y * v2.y,
                     v1.z * v2.z );
}

//---------------------------------------------------------------------
//
//   DISTANCES
//
//   distances between vectors (as points)
//   (why not via subtraction and functions?)
//
//---------------------------------------------------------------------

inline double distSq( const Vector3d &v1, const Vector3d &v2 )
{
   Vector3d dif(  v1.x - v2.x,   v1.y - v2.y,   v1.z - v2.z );
   return ( dif.x * dif.x + dif.y * dif.y + dif.z * dif.z );
}

inline double dist( const Vector3d &v1, const Vector3d &v2 )
{
   Vector3d dif( v1.x - v2.x,
                 v1.y - v2.y,
                 v1.z - v2.z );
   double lengthSq = dif.x*dif.x + dif.y* dif.y + dif.z* dif.z;
   return sqrt( lengthSq );
}

#endif

// ===========================  *  *  *  ==============================

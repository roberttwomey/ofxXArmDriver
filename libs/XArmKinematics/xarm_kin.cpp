// #include "ikfast.h" // found inside share/openrave-X.Y/python/ikfast.h
#include "xarm_kin.h"

const double ZERO_THRESH = 0.00000001;
int SIGN(double x) {
    return (x > 0) - (x < 0);
}

XArmKinematics::XArmKinematics(RobotType type){
    // if (type == RobotType::UR3){
    //     d1 =  0.1519;
    //     a2 = -0.24365;
    //     a3 = -0.21325;
    //     d4 =  0.11235;
    //     d5 =  0.08535;
    //     d6 =  0.0819;
    // }
    // else if (type == RobotType::UR5){
    //     d1 =  0.089159;
    //     a2 = -0.42500;
    //     a3 = -0.39225;
    //     d4 =  0.10915;
    //     d5 =  0.09465;
    //     d6 =  0.0823;
    // }
    // else if (type == RobotType::UR10){
    //     d1 =  0.1273;
    //     a2 = -0.612;
    //     a3 = -0.5723;
    //     d4 =  0.163941;
    //     d5 =  0.1157;
    //     d6 =  0.0922;
    // }
}

XArmKinematics::XArmKinematics(){
    
}
XArmKinematics::~XArmKinematics(){
    
}

void XArmKinematics::forward(const double* q, double* T) {
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
    q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q);
    double s234 = sin(q234), c234 = cos(q234);
    *T = ((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0; T++;
    *T = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) -
          (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0); T++;
    *T = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 -
          s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0)); T++;
    *T = ((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 -
          d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 -
          a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3); T++;
    *T = c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0; T++;
    *T = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) +
          s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0)); T++;
    *T = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) -
          s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0)); T++;
    *T = ((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 +
          (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 -
          a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3); T++;
    *T = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0); T++;
    *T = ((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6); T++;
    *T = (s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0); T++;
    *T = (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 -
          (d6*(c234*c5+s234*s5))/2.0 - d5*c234); T++;
    *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
}

// void XArmKinematics::forward_all(const double* q, double* T1, double* T2, double* T3,
//                                double* T4, double* T5, double* T6, double* T7) {
//     double s1 = sin(*q), c1 = cos(*q); q++; // q1
//     double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
//     double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++; // q3
//     q234 += *q; q++; // q4
//     double s5 = sin(*q), c5 = cos(*q); q++; // q5
//     double s6 = sin(*q), c6 = cos(*q); // q6
//     double s23 = sin(q23), c23 = cos(q23);
//     double s234 = sin(q234), c234 = cos(q234);
    
//     if(T1 != NULL) {
//         *T1 = c1; T1++;
//         *T1 = 0; T1++;
//         *T1 = s1; T1++;
//         *T1 = 0; T1++;
//         *T1 = s1; T1++;
//         *T1 = 0; T1++;
//         *T1 = -c1; T1++;
//         *T1 = 0; T1++;
//         *T1 =       0; T1++;
//         *T1 = 1; T1++;
//         *T1 = 0; T1++;
//         *T1 =d1; T1++;
//         *T1 =       0; T1++;
//         *T1 = 0; T1++;
//         *T1 = 0; T1++;
//         *T1 = 1; T1++;
//     }
    
//     if(T2 != NULL) {
//         *T2 = c1*c2; T2++;
//         *T2 = -c1*s2; T2++;
//         *T2 = s1; T2++;
//         *T2 =a2*c1*c2; T2++;
//         *T2 = c2*s1; T2++;
//         *T2 = -s1*s2; T2++;
//         *T2 = -c1; T2++;
//         *T2 =a2*c2*s1; T2++;
//         *T2 =         s2; T2++;
//         *T2 = c2; T2++;
//         *T2 = 0; T2++;
//         *T2 =   d1 + a2*s2; T2++;
//         *T2 =               0; T2++;
//         *T2 = 0; T2++;
//         *T2 = 0; T2++;
//         *T2 =                 1; T2++;
//     }
    
//     if(T3 != NULL) {
//         *T3 = c23*c1; T3++;
//         *T3 = -s23*c1; T3++;
//         *T3 = s1; T3++;
//         *T3 =c1*(a3*c23 + a2*c2); T3++;
//         *T3 = c23*s1; T3++;
//         *T3 = -s23*s1; T3++;
//         *T3 = -c1; T3++;
//         *T3 =s1*(a3*c23 + a2*c2); T3++;
//         *T3 =         s23; T3++;
//         *T3 = c23; T3++;
//         *T3 = 0; T3++;
//         *T3 =     d1 + a3*s23 + a2*s2; T3++;
//         *T3 =                    0; T3++;
//         *T3 = 0; T3++;
//         *T3 = 0; T3++;
//         *T3 =                                     1; T3++;
//     }
    
//     if(T4 != NULL) {
//         *T4 = c234*c1; T4++;
//         *T4 = s1; T4++;
//         *T4 = s234*c1; T4++;
//         *T4 =c1*(a3*c23 + a2*c2) + d4*s1; T4++;
//         *T4 = c234*s1; T4++;
//         *T4 = -c1; T4++;
//         *T4 = s234*s1; T4++;
//         *T4 =s1*(a3*c23 + a2*c2) - d4*c1; T4++;
//         *T4 =         s234; T4++;
//         *T4 = 0; T4++;
//         *T4 = -c234; T4++;
//         *T4 =                  d1 + a3*s23 + a2*s2; T4++;
//         *T4 =                         0; T4++;
//         *T4 = 0; T4++;
//         *T4 = 0; T4++;
//         *T4 =                                                  1; T4++;
//     }
    
//     if(T5 != NULL) {
//         *T5 = s1*s5 + c234*c1*c5; T5++;
//         *T5 = -s234*c1; T5++;
//         *T5 = c5*s1 - c234*c1*s5; T5++;
//         *T5 =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T5++;
//         *T5 = c234*c5*s1 - c1*s5; T5++;
//         *T5 = -s234*s1; T5++;
//         *T5 = - c1*c5 - c234*s1*s5; T5++;
//         *T5 =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; T5++;
//         *T5 =                           s234*c5; T5++;
//         *T5 = c234; T5++;
//         *T5 = -s234*s5; T5++;
//         *T5 =                          d1 + a3*s23 + a2*s2 - d5*c234; T5++;
//         *T5 =                                                   0; T5++;
//         *T5 = 0; T5++;
//         *T5 = 0; T5++;
//         *T5 =                                                                                 1; T5++;
//     }
    
//     if(T6 != NULL) {
//         *T6 =   c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T6++;
//         *T6 = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T6++;
//         *T6 = c5*s1 - c234*c1*s5; T6++;
//         *T6 =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T6++;
//         *T6 = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T6++;
//         *T6 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T6++;
//         *T6 = - c1*c5 - c234*s1*s5; T6++;
//         *T6 =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; T6++;
//         *T6 =                                       c234*s6 + s234*c5*c6; T6++;
//         *T6 = c234*c6 - s234*c5*s6; T6++;
//         *T6 = -s234*s5; T6++;
//         *T6 =                                                      d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; T6++;
//         *T6 =                                                                                                   0; T6++;
//         *T6 = 0; T6++;
//         *T6 = 0; T6++;
//         *T6 =                                                                                                                                            1; T6++;
//     }
// }

// Old URKinematics::inverse

// int XArmKinematics::inverse(const double* T, double* q_sols, double q6_des) {
//     int num_sols = 0;
//     double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++;
//     double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++;
//     double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;
    
// //    cout << "XArmKinematics :: inverse : ------------ | " << ofGetFrameNum() << endl;
// //    cout << "T02: " << T02 << " T00: " << T00 << " T01: " << T01 << " T03: " << T03 << endl;
// //    cout << "T12: " << T12 << " T10: " << T10 << " T11: " << T11 << " T13: " << T13 << endl;
// //    cout << "T22: " << T22 << " T20: " << T20 << " T21: " << T21 << " T23: " << T23 << endl;
    
//     ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
//     double q1[2];
//     {
//         double A = d6*T12 - T13;
//         double B = d6*T02 - T03;
//         double R = A*A + B*B;
//         if(fabs(A) < ZERO_THRESH) {
//             double div;
//             if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
//                 div = -SIGN(d4)*SIGN(B);
//             else
//                 div = -d4/B;
// //            double tdiv = div;
// //            if( tdiv > 1.0 ) tdiv = 1.0;
//             if( div > 1.0 ) {
//                 return num_sols;
//             }
            
//             double arcsin = asin(div);
//             if(fabs(arcsin) < ZERO_THRESH) {
//                 arcsin = 0.0;
//             }
//             if(arcsin < 0.0) {
//                 q1[0] = arcsin + 2.0*PI;
//             } else {
//                 q1[0] = arcsin;
//             }
//             q1[1] = PI - arcsin;
            
// //            cout << "div: " << div << " asin(div): " << asin( div ) << " arcsin: " << arcsin << " d4: " << d4 << " fabs(fabs(d4) - fabs(B)): " << fabs(fabs(d4) - fabs(B)) << " | " << ofGetFrameNum() << endl;
//         }
//         else if(fabs(B) < ZERO_THRESH) {
//             double div;
//             if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
//                 div = SIGN(d4)*SIGN(A);
//             else
//                 div = d4/A;
//             double arccos = acos(div);
//             q1[0] = arccos;
//             q1[1] = 2.0*PI - arccos;
//         }
//         else if(d4*d4 > R) {
// //            cout << " xxxxxxxx d4: " << d4 << " d4*d4: " << (d4*d4) << " R: " << R << " | " << ofGetFrameNum() << endl;
//             return num_sols;
//         }
//         else {
//             double d4SqrtR = d4 / sqrt(R);
//             if( d4SqrtR > 1.0 ) d4SqrtR = 1.0;
// //            if( d4SqrtR > 1.0 ) {
// //                return num_sols;
// //            }
//             double arccos = acos( d4SqrtR );
//             double arctan = atan2(-B, A);
//             double pos = arccos + arctan;
//             double neg = -arccos + arctan;
//             if(fabs(pos) < ZERO_THRESH)
//                 pos = 0.0;
//             if(fabs(neg) < ZERO_THRESH)
//                 neg = 0.0;
//             if(neg >= 0.0)
//                 q1[1] = neg;
//             else
//                 q1[1] = 2.0*PI + neg;
//             if(pos >= 0.0)
//                 q1[0] = pos;
//             else
//                 q1[0] = 2.0*PI + pos;
            
// //            cout << "arccos: " << arccos << " arctan: " << arctan << " sqrt(R): " << sqrt(R) << " d4: " << d4 << " (d4 / sqrt(R)): " << (d4 / sqrt(R)) << endl;

//         }
        
// //        cout << "fabs(A): " << fabs(A) << " fabs(B): " << fabs(B) << " | " << ofGetFrameNum() << endl;
//     }
    
    
//     ////////////////////////////////////////////////////////////////////////////////
    
//     ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
//     double q5[2][2];
//     {
//         for(int i=0;i<2;i++) {
//             double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
//             double div;
//             if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
//                 div = SIGN(numer) * SIGN(d6);
//             else
//                 div = numer / d6;
//             double arccos = acos(div);
//             q5[i][0] = arccos;
//             q5[i][1] = 2.0*PI - arccos;
//         }
//     }
//     ////////////////////////////////////////////////////////////////////////////////
    
//     {
//         for(int i=0;i<2;i++) {
//             for(int j=0;j<2;j++) {
//                 double c1 = cos(q1[i]), s1 = sin(q1[i]);
//                 double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
//                 double q6;
//                 ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
//                 if(fabs(s5) < ZERO_THRESH)
//                     q6 = q6_des;
//                 else {
//                     q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
//                                SIGN(s5)*(T00*s1 - T10*c1));
//                     if(fabs(q6) < ZERO_THRESH)
//                         q6 = 0.0;
//                     if(q6 < 0.0) {
//                         q6 += 2.0*PI;
// //                        while( q6 < 0 ) {
// //                            q6 += 2.0*PI;
// //                        }
//                     }
// //                    if( q6 > TWO_PI ) {
// //                        q6 = TWO_PI;
// //                    }
//                 }
//                 ////////////////////////////////////////////////////////////////////////////////
                
//                 double q2[2], q3[2], q4[2];
//                 ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
//                 double c6 = cos(q6), s6 = sin(q6);
//                 double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
//                 double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
//                 double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
//                 T03*c1 + T13*s1;
//                 double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);
                
//                 double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
                
//                 double fabsC3 = fabs( c3 );
                
// //                if( fabsC3 > 1.0 ) fabsC3 = 1.0;
                
//                 if( j == 0 ) {
// //                    cout << " q1[i]: " << q1[i] << " q5[i][j]: " << q5[i][j] << endl;
// //                    cout << " c6: " << c6 << " s6: " << s6 << " q6: " << q6 << " s5: " << s5 << " c5: " << c5 << " d5: " << d5 << endl;
// //                    cout << " p13y: " << p13y << " c3: " << c3 << " fabsC3: " << fabsC3 << " fabs( fabsC3 - 1.0): " << (fabs( fabsC3 - 1.0)) << endl;
//                 }
                
//                 if(fabs( fabsC3 - 1.0) < ZERO_THRESH)
//                     c3 = SIGN(c3);
//                 else if( fabsC3 > 1.0) {
//                     // TODO NO SOLUTION
//                     continue;
//                 }
//                 double arccos = acos(c3);
//                 q3[0] = arccos;
//                 q3[1] = 2.0*PI - arccos;
//                 double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
//                 double s3 = sin(arccos);
//                 double A = (a2 + a3*c3), B = a3*s3;
//                 q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
//                 q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
//                 double c23_0 = cos(q2[0]+q3[0]);
//                 double s23_0 = sin(q2[0]+q3[0]);
//                 double c23_1 = cos(q2[1]+q3[1]);
//                 double s23_1 = sin(q2[1]+q3[1]);
//                 q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
//                 q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
//                 ////////////////////////////////////////////////////////////////////////////////
//                 for(int k=0;k<2;k++) {
//                     if(fabs(q2[k]) < ZERO_THRESH)
//                         q2[k] = 0.0;
//                     else if(q2[k] < 0.0) q2[k] += 2.0*PI;
//                     if(fabs(q4[k]) < ZERO_THRESH)
//                         q4[k] = 0.0;
//                     else if(q4[k] < 0.0) q4[k] += 2.0*PI;
//                     q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k];
//                     q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k];
//                     q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6;
//                     num_sols++;
//                 }
                
//             }
//         }
//     }
//     return num_sols;
// }

// end Old URKinematics::inverse()


// IKFAST GENERATED CODE

using namespace ikfast;

#define IKFAST_ASSERT(b) { if( !(b) ) { std::stringstream ss; ss << "ikfast exception: " << __FILE__ << ":" << __LINE__ << ": " <<__PRETTY_FUNCTION__ << ": Assertion '" << #b << "' failed"; throw std::runtime_error(ss.str()); } }
#define IKFAST_ALIGNED16(x) x __attribute((aligned(16)))

#define IK2PI  ((IkReal)6.28318530717959)
#define IKPI  ((IkReal)3.14159265358979)
#define IKPI_2  ((IkReal)1.57079632679490)

// // lapack routines
// extern "C" {
//   void dgetrf_ (const int* m, const int* n, double* a, const int* lda, int* ipiv, int* info);
//   void zgetrf_ (const int* m, const int* n, std::complex<double>* a, const int* lda, int* ipiv, int* info);
//   void dgetri_(const int* n, const double* a, const int* lda, int* ipiv, double* work, const int* lwork, int* info);
//   void dgesv_ (const int* n, const int* nrhs, double* a, const int* lda, int* ipiv, double* b, const int* ldb, int* info);
//   void dgetrs_(const char *trans, const int *n, const int *nrhs, double *a, const int *lda, int *ipiv, double *b, const int *ldb, int *info);
//   void dgeev_(const char *jobvl, const char *jobvr, const int *n, double *a, const int *lda, double *wr, double *wi,double *vl, const int *ldvl, double *vr, const int *ldvr, double *work, const int *lwork, int *info);
// }

using namespace std; // necessary to get std math routines

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

inline float IKabs(float f) { return fabsf(f); }
inline double IKabs(double f) { return fabs(f); }

inline float IKsqr(float f) { return f*f; }
inline double IKsqr(double f) { return f*f; }

inline float IKlog(float f) { return logf(f); }
inline double IKlog(double f) { return log(f); }

// allows asin and acos to exceed 1. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_SINCOS_THRESH
#define IKFAST_SINCOS_THRESH ((IkReal)1e-7)
#endif

// used to check input to atan2 for degenerate cases. has to be smaller than thresholds used for branch conds and evaluation
#ifndef IKFAST_ATAN2_MAGTHRESH
#define IKFAST_ATAN2_MAGTHRESH ((IkReal)1e-7)
#endif

// minimum distance of separate solutions
#ifndef IKFAST_SOLUTION_THRESH
#define IKFAST_SOLUTION_THRESH ((IkReal)1e-6)
#endif

// there are checkpoints in ikfast that are evaluated to make sure they are 0. This threshold speicfies by how much they can deviate
#ifndef IKFAST_EVALCOND_THRESH
#define IKFAST_EVALCOND_THRESH ((IkReal)0.00001)
#endif


inline float IKasin(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(-IKPI_2);
else if( f >= 1 ) return float(IKPI_2);
return asinf(f);
}
inline double IKasin(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return -IKPI_2;
else if( f >= 1 ) return IKPI_2;
return asin(f);
}

// return positive value in [0,y)
inline float IKfmod(float x, float y)
{
    while(x < 0) {
        x += y;
    }
    return fmodf(x,y);
}

// return positive value in [0,y)
inline double IKfmod(double x, double y)
{
    while(x < 0) {
        x += y;
    }
    return fmod(x,y);
}

inline float IKacos(float f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return float(IKPI);
else if( f >= 1 ) return float(0);
return acosf(f);
}
inline double IKacos(double f)
{
IKFAST_ASSERT( f > -1-IKFAST_SINCOS_THRESH && f < 1+IKFAST_SINCOS_THRESH ); // any more error implies something is wrong with the solver
if( f <= -1 ) return IKPI;
else if( f >= 1 ) return 0;
return acos(f);
}
inline float IKsin(float f) { return sinf(f); }
inline double IKsin(double f) { return sin(f); }
inline float IKcos(float f) { return cosf(f); }
inline double IKcos(double f) { return cos(f); }
inline float IKtan(float f) { return tanf(f); }
inline double IKtan(double f) { return tan(f); }
inline float IKsqrt(float f) { if( f <= 0.0f ) return 0.0f; return sqrtf(f); }
inline double IKsqrt(double f) { if( f <= 0.0 ) return 0.0; return sqrt(f); }
inline float IKatan2Simple(float fy, float fx) {
    return atan2f(fy,fx);
}
inline float IKatan2(float fy, float fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return float(IKPI_2);
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2f(fy,fx);
}
inline double IKatan2Simple(double fy, double fx) {
    return atan2(fy,fx);
}
inline double IKatan2(double fy, double fx) {
    if( isnan(fy) ) {
        IKFAST_ASSERT(!isnan(fx)); // if both are nan, probably wrong value will be returned
        return IKPI_2;
    }
    else if( isnan(fx) ) {
        return 0;
    }
    return atan2(fy,fx);
}

template <typename T>
struct CheckValue
{
    T value;
    bool valid;
};

template <typename T>
inline CheckValue<T> IKatan2WithCheck(T fy, T fx, T epsilon)
{
    CheckValue<T> ret;
    ret.valid = false;
    ret.value = 0;
    if( !isnan(fy) && !isnan(fx) ) {
        if( IKabs(fy) >= IKFAST_ATAN2_MAGTHRESH || IKabs(fx) > IKFAST_ATAN2_MAGTHRESH ) {
            ret.value = IKatan2Simple(fy,fx);
            ret.valid = true;
        }
    }
    return ret;
}

inline float IKsign(float f) {
    if( f > 0 ) {
        return float(1);
    }
    else if( f < 0 ) {
        return float(-1);
    }
    return 0;
}

inline double IKsign(double f) {
    if( f > 0 ) {
        return 1.0;
    }
    else if( f < 0 ) {
        return -1.0;
    }
    return 0;
}

template <typename T>
inline CheckValue<T> IKPowWithIntegerCheck(T f, int n)
{
    CheckValue<T> ret;
    ret.valid = true;
    if( n == 0 ) {
        ret.value = 1.0;
        return ret;
    }
    else if( n == 1 )
    {
        ret.value = f;
        return ret;
    }
    else if( n < 0 )
    {
        if( f == 0 )
        {
            ret.valid = false;
            ret.value = (T)1.0e30;
            return ret;
        }
        if( n == -1 ) {
            ret.value = T(1.0)/f;
            return ret;
        }
    }

    int num = n > 0 ? n : -n;
    if( num == 2 ) {
        ret.value = f*f;
    }
    else if( num == 3 ) {
        ret.value = f*f*f;
    }
    else {
        ret.value = 1.0;
        while(num>0) {
            if( num & 1 ) {
                ret.value *= f;
            }
            num >>= 1;
            f *= f;
        }
    }
    
    if( n < 0 ) {
        ret.value = T(1.0)/ret.value;
    }
    return ret;
}

// /// solves the forward kinematics equations.
// /// \param pfree is an array specifying the free joints of the chain.
void XArmKinematics::forward_all(const double* q, double* T1, double* T2, double* T3,
                               double* T4, double* T5, double* T6, double* T7) {

// IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot) {

const double* j = q;

// hold the results
IkReal* eetrans = new IkReal[9];
IkReal* eerot = new IkReal[3];

IkReal x0,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32,x33,x34,x35,x36,x37,x38,x39,x40,x41,x42,x43,x44,x45,x46,x47,x48,x49,x50,x51,x52,x53,x54,x55,x56,x57,x58,x59,x60,x61,x62,x63,x64,x65;
x0=IKcos(j[0]);
x1=IKcos(j[3]);
x2=IKsin(j[1]);
x3=IKsin(j[3]);
x4=IKcos(j[2]);
x5=IKsin(j[0]);
x6=IKcos(j[1]);
x7=IKsin(j[2]);
x8=IKcos(j[4]);
x9=IKsin(j[4]);
x10=IKcos(j[6]);
x11=IKsin(j[6]);
x12=IKcos(j[5]);
x13=IKsin(j[5]);
x14=((0.097)*x2);
x15=((0.0775)*x4);
x16=((0.3425)*x5);
x17=((1.0)*x1);
x18=((1.0)*x5);
x19=((0.3425)*x4);
x20=((1.0)*x2);
x21=((0.0525)*x4);
x22=((0.0775)*x5);
x23=((1.0)*x4);
x24=((0.293)*x2);
x25=((0.076)*x8);
x26=(x7*x9);
x27=(x1*x6);
x28=(x5*x6);
x29=(x1*x2);
x30=(x0*x7);
x31=((-1.0)*x12);
x32=(x3*x6);
x33=(x2*x3);
x34=((-1.0)*x13);
x35=(x0*x6);
x36=((-0.097)*x8);
x37=(x0*x33);
x38=(x17*x2*x4);
x39=(x20*x26);
x40=((((-1.0)*x18*x7))+((x35*x4)));
x41=(((x30*x6))+((x4*x5)));
x42=(x30+((x28*x4)));
x43=((((-1.0)*x0*x23))+((x28*x7)));
x44=((((-1.0)*x38))+x32);
x45=(x41*x8);
x46=(x3*x40);
x47=((((-1.0)*x20*x3*x4))+(((-1.0)*x17*x6)));
x48=(x41*x9);
x49=(x1*x40);
x50=(x3*x42);
x51=(x44*x8);
x52=(x43*x9);
x53=(x13*x47);
x54=(x37+x49);
x55=((((-1.0)*x0*x17*x2))+x46);
x56=((((-1.0)*x17*x2*x5))+x50);
x57=(((x33*x5))+((x1*x42)));
x58=((((-1.0)*x39))+x51);
x59=((((-1.0)*x20*x7*x8))+((x9*(((((-1.0)*x32))+x38)))));
x60=(x57*x8);
x61=(x12*x58);
x62=(x54*x8);
x63=(x48+x62);
x64=(x52+x60);
x65=(((x9*(((((-1.0)*x18*x33))+(((-1.0)*x17*x42))))))+((x43*x8)));
eerot[0]=(((x10*((((x12*x63))+((x13*x55))))))+((x11*((x45+((x9*(((((-1.0)*x37))+(((-1.0)*x49)))))))))));
IkReal x66=((1.0)*x0);
eerot[1]=(((x11*((((x31*x63))+((x34*((x46+(((-1.0)*x29*x66))))))))))+((x10*((((x9*(((((-1.0)*x20*x3*x66))+(((-1.0)*x17*x40))))))+x45)))));
eerot[2]=(((x13*(((((-1.0)*x48))+(((-1.0)*x62))))))+((x12*x55)));
IkReal x67=((1.0)*x7);
IkReal x68=(x0*x29);
eetrans[0]=(((x0*x24))+((x3*(((((-1.0)*x16*x67))+((x19*x35))))))+((x13*((((x36*x54))+(((-0.097)*x48))))))+((x13*(((((0.076)*x46))+(((-0.076)*x68))))))+((x21*x35))+((x12*(((((-0.097)*x68))+(((0.097)*x46))))))+((x1*((((x15*x35))+(((-1.0)*x22*x67))))))+(((-0.3425)*x68))+(((-0.0525)*x5*x7))+(((0.0775)*x37))+((x12*((((x25*x54))+(((0.076)*x48)))))));
eerot[3]=(((x11*x65))+((x10*((((x12*x64))+((x13*x56)))))));
eerot[4]=(((x10*x65))+((x11*((((x34*x56))+((x31*x64)))))));
eerot[5]=(((x12*x56))+((x13*(((((-1.0)*x52))+(((-1.0)*x60)))))));
eetrans[1]=(((x21*x28))+((x12*(((((0.076)*x52))+((x25*x57))))))+((x13*((((x36*x57))+(((-0.097)*x52))))))+(((-1.0)*x16*x29))+((x22*x33))+((x1*(((((0.0775)*x30))+((x15*x28))))))+((x12*(((((-1.0)*x1*x14*x5))+(((0.097)*x50))))))+((x13*(((((0.076)*x50))+(((-0.076)*x29*x5))))))+((x3*(((((0.3425)*x30))+((x16*x4*x6))))))+(((0.0525)*x30))+((x24*x5)));
eerot[6]=(((x10*((x53+x61))))+((x11*x59)));
eerot[7]=(((x10*x59))+((x11*(((((-1.0)*x53))+(((-1.0)*x61)))))));
eerot[8]=(((x13*((x39+(((-1.0)*x51))))))+((x12*x47)));
eetrans[2]=((0.267)+((x12*((((x25*x44))+(((-0.076)*x2*x26))))))+(((-1.0)*x15*x29))+((x13*((((x36*x44))+((x14*x26))))))+(((-1.0)*x2*x21))+(((-1.0)*x19*x33))+(((0.0775)*x32))+(((-0.3425)*x27))+((x13*(((((-0.076)*x27))+(((-0.076)*x33*x4))))))+(((0.293)*x6))+((x12*(((((-0.097)*x27))+(((-1.0)*x14*x3*x4)))))));

for(int i=0; i< 3;++i){
    T7[i*4+0] = eerot[i*3+0];
    T7[i*4+1] = eerot[i*3+1];
    T7[i*4+2] = eerot[i*3+2];
    T7[i*4+3] = eetrans[i];
}
T7[3*4+0] = 0;
T7[3*4+1] = 0;
T7[3*4+2] = 0;
T7[3*4+3] = 1;
}

IKFAST_API int GetNumFreeParameters() { return 1; }
IKFAST_API int* GetFreeParameters() { static int freeparams[] = {6}; return freeparams; }
IKFAST_API int GetNumJoints() { return 7; }

IKFAST_API int GetIkRealSize() { return sizeof(IkReal); }

IKFAST_API int GetIkType() { return 0x67000001; }

class IKSolver {
public:
IkReal j0,cj0,sj0,htj0,j0mul,j1,cj1,sj1,htj1,j1mul,j2,cj2,sj2,htj2,j2mul,j3,cj3,sj3,htj3,j3mul,j4,cj4,sj4,htj4,j4mul,j5,cj5,sj5,htj5,j5mul,j6,cj6,sj6,htj6,new_r00,r00,rxp0_0,new_r01,r01,rxp0_1,new_r02,r02,rxp0_2,new_r10,r10,rxp1_0,new_r11,r11,rxp1_1,new_r12,r12,rxp1_2,new_r20,r20,rxp2_0,new_r21,r21,rxp2_1,new_r22,r22,rxp2_2,new_px,px,npx,new_py,py,npy,new_pz,pz,npz,pp;
unsigned char _ij0[2], _nj0,_ij1[2], _nj1,_ij2[2], _nj2,_ij3[2], _nj3,_ij4[2], _nj4,_ij5[2], _nj5,_ij6[2], _nj6;

IkReal j100, cj100, sj100;
unsigned char _ij100[2], _nj100;
bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
j0=numeric_limits<IkReal>::quiet_NaN(); _ij0[0] = -1; _ij0[1] = -1; _nj0 = -1; j1=numeric_limits<IkReal>::quiet_NaN(); _ij1[0] = -1; _ij1[1] = -1; _nj1 = -1; j2=numeric_limits<IkReal>::quiet_NaN(); _ij2[0] = -1; _ij2[1] = -1; _nj2 = -1; j3=numeric_limits<IkReal>::quiet_NaN(); _ij3[0] = -1; _ij3[1] = -1; _nj3 = -1; j4=numeric_limits<IkReal>::quiet_NaN(); _ij4[0] = -1; _ij4[1] = -1; _nj4 = -1; j5=numeric_limits<IkReal>::quiet_NaN(); _ij5[0] = -1; _ij5[1] = -1; _nj5 = -1;  _ij6[0] = -1; _ij6[1] = -1; _nj6 = 0; 
for(int dummyiter = 0; dummyiter < 1; ++dummyiter) {
    solutions.Clear();
j6=pfree[0]; cj6=cos(pfree[0]); sj6=sin(pfree[0]), htj6=tan(pfree[0]*0.5);
r00 = eerot[0*3+0];
r01 = eerot[0*3+1];
r02 = eerot[0*3+2];
r10 = eerot[1*3+0];
r11 = eerot[1*3+1];
r12 = eerot[1*3+2];
r20 = eerot[2*3+0];
r21 = eerot[2*3+1];
r22 = eerot[2*3+2];
px = eetrans[0]; py = eetrans[1]; pz = eetrans[2];

new_r00=r00;
new_r01=r01;
new_r02=r02;
new_px=((((-0.097)*r02))+px);
new_r10=r10;
new_r11=r11;
new_r12=r12;
new_py=((((-0.097)*r12))+py);
new_r20=r20;
new_r21=r21;
new_r22=r22;
new_pz=((-0.267)+(((-0.097)*r22))+pz);
r00 = new_r00; r01 = new_r01; r02 = new_r02; r10 = new_r10; r11 = new_r11; r12 = new_r12; r20 = new_r20; r21 = new_r21; r22 = new_r22; px = new_px; py = new_py; pz = new_pz;
IkReal x69=((1.0)*px);
IkReal x70=((1.0)*pz);
IkReal x71=((1.0)*py);
pp=((px*px)+(py*py)+(pz*pz));
npx=(((px*r00))+((py*r10))+((pz*r20)));
npy=(((px*r01))+((py*r11))+((pz*r21)));
npz=(((px*r02))+((py*r12))+((pz*r22)));
rxp0_0=((((-1.0)*r20*x71))+((pz*r10)));
rxp0_1=(((px*r20))+(((-1.0)*r00*x70)));
rxp0_2=((((-1.0)*r10*x69))+((py*r00)));
rxp1_0=((((-1.0)*r21*x71))+((pz*r11)));
rxp1_1=(((px*r21))+(((-1.0)*r01*x70)));
rxp1_2=((((-1.0)*r11*x69))+((py*r01)));
rxp2_0=((((-1.0)*r22*x71))+((pz*r12)));
rxp2_1=(((px*r22))+(((-1.0)*r02*x70)));
rxp2_2=((((-1.0)*r12*x69))+((py*r02)));
{
IkReal j3array[2], cj3array[2], sj3array[2];
bool j3valid[2]={false};
_nj3 = 2;
if( (((-0.986058506449639)+(((0.727076843872457)*npy*sj6))+(((-0.727076843872457)*cj6*npx))+(((4.78340028863459)*pp)))) < -1-IKFAST_SINCOS_THRESH || (((-0.986058506449639)+(((0.727076843872457)*npy*sj6))+(((-0.727076843872457)*cj6*npx))+(((4.78340028863459)*pp)))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x72=IKasin(((-0.986058506449639)+(((0.727076843872457)*npy*sj6))+(((-0.727076843872457)*cj6*npx))+(((4.78340028863459)*pp))));
j3array[0]=((-1.97062567615197)+(((-1.0)*x72)));
sj3array[0]=IKsin(j3array[0]);
cj3array[0]=IKcos(j3array[0]);
j3array[1]=((1.17096697743782)+x72);
sj3array[1]=IKsin(j3array[1]);
cj3array[1]=IKcos(j3array[1]);
if( j3array[0] > IKPI )
{
    j3array[0]-=IK2PI;
}
else if( j3array[0] < -IKPI )
{    j3array[0]+=IK2PI;
}
j3valid[0] = true;
if( j3array[1] > IKPI )
{
    j3array[1]-=IK2PI;
}
else if( j3array[1] < -IKPI )
{    j3array[1]+=IK2PI;
}
j3valid[1] = true;
for(int ij3 = 0; ij3 < 2; ++ij3)
{
if( !j3valid[ij3] )
{
    continue;
}
_ij3[0] = ij3; _ij3[1] = -1;
for(int iij3 = ij3+1; iij3 < 2; ++iij3)
{
if( j3valid[iij3] && IKabs(cj3array[ij3]-cj3array[iij3]) < IKFAST_SOLUTION_THRESH && IKabs(sj3array[ij3]-sj3array[iij3]) < IKFAST_SOLUTION_THRESH )
{
    j3valid[iij3]=false; _ij3[1] = iij3; break; 
}
}
j3 = j3array[ij3]; cj3 = cj3array[ij3]; sj3 = sj3array[ij3];

{
IkReal j5eval[2];
IkReal x73=(cj6*npx);
IkReal x74=(npy*sj6);
j5eval[0]=((1.0)+(((173.130193905817)*(x73*x73)))+(((-346.260387811634)*x73*x74))+(((173.130193905817)*(npz*npz)))+(((-26.3157894736842)*x73))+(((173.130193905817)*(x74*x74)))+(((26.3157894736842)*x74)));
j5eval[1]=((IKabs(((0.076)+x74+(((-1.0)*x73)))))+(IKabs(npz)));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal j4eval[1];
j4eval[0]=((1.47619047619048)+cj3+(((5.58095238095238)*sj3)));
if( IKabs(j4eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((0.440693041647182)+j3)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
sj4array[0]=((((-51305507480.2948)*npx*sj6))+(((-51305507480.2948)*cj6*npy)));
if( sj4array[0] >= -1-IKFAST_SINCOS_THRESH && sj4array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j4valid[0] = j4valid[1] = true;
    j4array[0] = IKasin(sj4array[0]);
    cj4array[0] = IKcos(j4array[0]);
    sj4array[1] = sj4array[0];
    j4array[1] = j4array[0] > 0 ? (IKPI-j4array[0]) : (-IKPI-j4array[0]);
    cj4array[1] = -cj4array[0];
}
else if( isnan(sj4array[0]) )
{
    // probably any value will work
    j4valid[0] = true;
    cj4array[0] = 1; sj4array[0] = 0; j4array[0] = 0;
}
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
IkReal j5eval[3];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
IkReal x75=pp*pp;
IkReal x76=npz*npz;
j5eval[0]=((-1.0)+(((-297.537495925507)*x76))+(((226.964147749891)*pp))+(((-12878.1810909586)*x75)));
j5eval[1]=IKsign(((-50425664951339.0)+(((1.14448180704022e+16)*pp))+(((-6.49390844875346e+17)*x75))+(((-1.500352608e+16)*x76))));
j5eval[2]=((IKabs(((((-826688032417458.0)*npz))+(((-1923914.47368421)*cj4*pp))+(((16953.4659351072)*cj4)))))+(IKabs(((47925957551422.4)+(((-5.43873705537801e+15)*pp))+(((292435.0)*cj4*npz))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
IkReal x77=npz*npz;
IkReal x78=(pp*sj4);
IkReal x79=((1.64512347368421e+16)*npx);
IkReal x80=(npy*sj6);
IkReal x81=((144967695558428.0)*sj4);
IkReal x82=(cj4*sj6);
IkReal x83=((13.1578947368421)*sj4);
IkReal x84=((2.50058768e+15)*npz);
IkReal x85=((137781338736243.0)*sj4);
IkReal x86=(cj6*npx);
IkReal x87=(cj4*cj6*npy);
IkReal x88=(sj4*x77);
j5eval[0]=((((-1.0)*sj4))+(((-1.0)*x80*x83))+(((1493.18518256507)*x78*x80))+(((-1493.18518256507)*x78*x86))+(((-226.964147749891)*x88))+((x83*x86))+(((113.482073874945)*x78)));
j5eval[1]=((IKabs(((((10471381743954.5)*sj4))+(((-1.0)*npx*x82*x84))+(((-1.0)*x84*x87))+((x80*x85))+(((-1.0)*x85*x86)))))+(IKabs(((((-144967695558428.0)*x87))+(((1.64512347368421e+16)*pp*x87))+(((-144967695558428.0)*npx*x82))+(((-1.0)*npz*x85))+((pp*x79*x82))))));
j5eval[2]=IKsign(((((1.64512347368421e+16)*x78*x80))+(((-1.0)*x80*x81))+((x81*x86))+(((1.25029384e+15)*x78))+(((-1.0)*cj6*x78*x79))+(((-11017544862440.5)*sj4))+(((-2.50058768e+15)*x88))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
IkReal x89=npz*npz;
IkReal x90=(cj4*pp);
IkReal x91=(npy*sj6);
IkReal x92=(cj6*npx);
IkReal x93=((144967695558428.0)*cj4);
IkReal x94=((13.1578947368421)*cj4);
IkReal x95=(cj4*x89);
j5eval[0]=(((x92*x94))+(((1493.18518256507)*x90*x91))+(((-1.0)*x91*x94))+(((-226.964147749891)*x95))+(((113.482073874945)*x90))+(((-1493.18518256507)*x90*x92))+(((-1.0)*cj4)));
j5eval[1]=IKsign(((((1.64512347368421e+16)*x90*x91))+((x92*x93))+(((-1.0)*x91*x93))+(((1.25029384e+15)*x90))+(((-11017544862440.5)*cj4))+(((-1.64512347368421e+16)*x90*x92))+(((-2.50058768e+15)*x95))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[3];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
sj4=1.0;
cj4=0;
j4=1.5707963267949;
IkReal x96=pp*pp;
IkReal x97=npz*npz;
j5eval[0]=((-1.0)+(((-12878.1810909586)*x96))+(((226.964147749891)*pp))+(((-297.537495925507)*x97)));
j5eval[1]=((((7.25787693146396e-15)*(IKabs(((7987659591903.73)+(((-906456175896336.0)*pp)))))))+(IKabs(npz)));
j5eval[2]=IKsign(((-8404277491889.83)+(((1.90746967840037e+15)*pp))+(((-1.08231807479224e+17)*x96))+(((-2.50058768e+15)*x97))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
sj4=1.0;
cj4=0;
j4=1.5707963267949;
IkReal x98=npz*npz;
IkReal x99=(npy*sj6);
IkReal x100=(cj6*npx);
IkReal x101=((1493.18518256507)*pp);
IkReal x102=((1.64512347368421e+16)*pp);
j5eval[0]=((-1.0)+(((-13.1578947368421)*x99))+(((-1.0)*x100*x101))+((x101*x99))+(((13.1578947368421)*x100))+(((-226.964147749891)*x98))+(((113.482073874945)*pp)));
j5eval[1]=((((7.25787693146396e-15)*(IKabs(((10471381743954.5)+(((-137781338736243.0)*x100))+(((137781338736243.0)*x99)))))))+(IKabs(npz)));
j5eval[2]=IKsign(((-11017544862440.5)+(((-144967695558428.0)*x99))+(((-1.0)*x100*x102))+((x102*x99))+(((1.25029384e+15)*pp))+(((144967695558428.0)*x100))+(((-2.50058768e+15)*x98))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x103=(npy*sj6);
IkReal x104=(cj6*npx);
IkReal x105=((1.64512347368421e+16)*pp);
CheckValue<IkReal> x106=IKPowWithIntegerCheck(IKsign(((-11017544862440.5)+(((-1.0)*x104*x105))+(((-144967695558428.0)*x103))+(((-2.50058768e+15)*(npz*npz)))+((x103*x105))+(((1.25029384e+15)*pp))+(((144967695558428.0)*x104)))),-1);
if(!x106.valid){
continue;
}
CheckValue<IkReal> x107 = IKatan2WithCheck(IkReal(((10471381743954.5)+(((-137781338736243.0)*x104))+(((137781338736243.0)*x103)))),IkReal(((-137781338736243.0)*npz)),IKFAST_ATAN2_MAGTHRESH);
if(!x107.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x106.value)))+(x107.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x108=IKcos(j5);
IkReal x109=IKsin(j5);
IkReal x110=((6.57894736842105)*pp);
IkReal x111=(npz*x109);
IkReal x112=((1.0)*x108);
evalcond[0]=((((0.0579734502884649)*x108))+x111+(((-1.0)*x108*x110)));
evalcond[1]=((0.0550995831252928)+(((0.0579734502884649)*x109))+(((-1.0)*npz*x112))+(((-1.0)*x109*x110)));
evalcond[2]=((((0.076)*x108))+(((-1.0)*cj6*npx*x112))+x111+((npy*sj6*x108)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x651 = IKatan2WithCheck(IkReal(((7987659591903.73)+(((-906456175896336.0)*pp)))),IkReal(((-137781338736243.0)*npz)),IKFAST_ATAN2_MAGTHRESH);
if(!x651.valid){
continue;
}
CheckValue<IkReal> x652=IKPowWithIntegerCheck(IKsign(((-8404277491889.83)+(((1.90746967840037e+15)*pp))+(((-2.50058768e+15)*(npz*npz)))+(((-1.08231807479224e+17)*(pp*pp))))),-1);
if(!x652.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x651.value)+(((1.5707963267949)*(x652.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x653=IKcos(j5);
IkReal x654=IKsin(j5);
IkReal x655=((6.57894736842105)*pp);
IkReal x656=(npz*x654);
IkReal x657=((1.0)*x653);
evalcond[0]=((((0.0579734502884649)*x653))+x656+(((-1.0)*x653*x655)));
evalcond[1]=((0.0550995831252928)+(((0.0579734502884649)*x654))+(((-1.0)*npz*x657))+(((-1.0)*x654*x655)));
evalcond[2]=((((0.076)*x653))+((npy*sj6*x653))+x656+(((-1.0)*cj6*npx*x657)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[3];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
sj4=-1.0;
cj4=0;
j4=-1.5707963267949;
IkReal x658=pp*pp;
IkReal x659=npz*npz;
j5eval[0]=((-1.0)+(((-12878.1810909586)*x658))+(((226.964147749891)*pp))+(((-297.537495925507)*x659)));
j5eval[1]=((((7.25787693146396e-15)*(IKabs(((7987659591903.73)+(((-906456175896336.0)*pp)))))))+(IKabs(npz)));
j5eval[2]=IKsign(((-8404277491889.83)+(((-2.50058768e+15)*x659))+(((1.90746967840037e+15)*pp))+(((-1.08231807479224e+17)*x658))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
sj4=-1.0;
cj4=0;
j4=-1.5707963267949;
IkReal x660=npz*npz;
IkReal x661=(npy*sj6);
IkReal x662=(cj6*npx);
IkReal x663=((1493.18518256507)*pp);
IkReal x664=((1.64512347368421e+16)*pp);
j5eval[0]=((1.0)+(((226.964147749891)*x660))+(((13.1578947368421)*x661))+((x662*x663))+(((-1.0)*x661*x663))+(((-113.482073874945)*pp))+(((-13.1578947368421)*x662)));
j5eval[1]=((IKabs(npz))+(((7.25787693146396e-15)*(IKabs(((-10471381743954.5)+(((-137781338736243.0)*x661))+(((137781338736243.0)*x662))))))));
j5eval[2]=IKsign(((11017544862440.5)+(((-1.25029384e+15)*pp))+((x662*x664))+(((-1.0)*x661*x664))+(((2.50058768e+15)*x660))+(((144967695558428.0)*x661))+(((-144967695558428.0)*x662))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x665=(npy*sj6);
IkReal x666=(cj6*npx);
IkReal x667=((1.64512347368421e+16)*pp);
CheckValue<IkReal> x668 = IKatan2WithCheck(IkReal(((-10471381743954.5)+(((-137781338736243.0)*x665))+(((137781338736243.0)*x666)))),IkReal(((137781338736243.0)*npz)),IKFAST_ATAN2_MAGTHRESH);
if(!x668.valid){
continue;
}
CheckValue<IkReal> x669=IKPowWithIntegerCheck(IKsign(((11017544862440.5)+(((-1.0)*x665*x667))+(((-1.25029384e+15)*pp))+((x666*x667))+(((2.50058768e+15)*(npz*npz)))+(((144967695558428.0)*x665))+(((-144967695558428.0)*x666)))),-1);
if(!x669.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x668.value)+(((1.5707963267949)*(x669.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x670=IKcos(j5);
IkReal x671=IKsin(j5);
IkReal x672=((6.57894736842105)*pp);
IkReal x673=(npz*x671);
IkReal x674=((1.0)*x670);
evalcond[0]=((((0.0579734502884649)*x670))+(((-1.0)*x670*x672))+x673);
evalcond[1]=((0.0550995831252928)+(((0.0579734502884649)*x671))+(((-1.0)*npz*x674))+(((-1.0)*x671*x672)));
evalcond[2]=(((cj6*npx*x670))+(((-1.0)*x673))+(((-1.0)*npy*sj6*x674))+(((-0.076)*x670)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x675 = IKatan2WithCheck(IkReal(((7987659591903.73)+(((-906456175896336.0)*pp)))),IkReal(((-137781338736243.0)*npz)),IKFAST_ATAN2_MAGTHRESH);
if(!x675.valid){
continue;
}
CheckValue<IkReal> x676=IKPowWithIntegerCheck(IKsign(((-8404277491889.83)+(((1.90746967840037e+15)*pp))+(((-2.50058768e+15)*(npz*npz)))+(((-1.08231807479224e+17)*(pp*pp))))),-1);
if(!x676.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x675.value)+(((1.5707963267949)*(x676.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x677=IKcos(j5);
IkReal x678=IKsin(j5);
IkReal x679=((6.57894736842105)*pp);
IkReal x680=(npz*x678);
IkReal x681=((1.0)*x677);
evalcond[0]=((((0.0579734502884649)*x677))+x680+(((-1.0)*x677*x679)));
evalcond[1]=((0.0550995831252928)+(((-1.0)*npz*x681))+(((-1.0)*x678*x679))+(((0.0579734502884649)*x678)));
evalcond[2]=(((cj6*npx*x677))+(((-1.0)*npy*sj6*x681))+(((-1.0)*x680))+(((-0.076)*x677)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j4))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[2];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
sj4=0;
cj4=1.0;
j4=0;
IkReal x682=pp*pp;
IkReal x683=npz*npz;
j5eval[0]=((-1.0)+(((-12878.1810909586)*x682))+(((-297.537495925507)*x683))+(((226.964147749891)*pp)));
j5eval[1]=IKsign(((-50425664951339.0)+(((1.14448180704022e+16)*pp))+(((-1.500352608e+16)*x683))+(((-6.49390844875346e+17)*x682))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
sj4=0;
cj4=1.0;
j4=0;
IkReal x684=npz*npz;
IkReal x685=(cj6*npx);
IkReal x686=(npy*sj6);
IkReal x687=((1493.18518256507)*pp);
IkReal x688=((9.87074084210526e+16)*pp);
j5eval[0]=((-1.0)+(((-1.0)*x685*x687))+((x686*x687))+(((-13.1578947368421)*x686))+(((113.482073874945)*pp))+(((-226.964147749891)*x684))+(((13.1578947368421)*x685)));
j5eval[1]=IKsign(((-66105269174643.1)+(((-1.0)*x685*x688))+((x686*x688))+(((-1.500352608e+16)*x684))+(((-869806173350567.0)*x686))+(((869806173350567.0)*x685))+(((7.50176304e+15)*pp))));
j5eval[2]=((IKabs(((62828290463726.8)+(((292435.0)*npz))+(((-826688032417458.0)*x685))+(((826688032417458.0)*x686)))))+(IKabs(((16953.4659351072)+(((-1923914.47368421)*pp))+(((-826688032417458.0)*npz))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x689=(cj6*npx);
IkReal x690=(npy*sj6);
IkReal x691=((9.87074084210526e+16)*pp);
CheckValue<IkReal> x692=IKPowWithIntegerCheck(IKsign(((-66105269174643.1)+((x690*x691))+(((-869806173350567.0)*x690))+(((869806173350567.0)*x689))+(((7.50176304e+15)*pp))+(((-1.500352608e+16)*(npz*npz)))+(((-1.0)*x689*x691)))),-1);
if(!x692.valid){
continue;
}
CheckValue<IkReal> x693 = IKatan2WithCheck(IkReal(((62828290463726.8)+(((292435.0)*npz))+(((-826688032417458.0)*x689))+(((826688032417458.0)*x690)))),IkReal(((16953.4659351072)+(((-1923914.47368421)*pp))+(((-826688032417458.0)*npz)))),IKFAST_ATAN2_MAGTHRESH);
if(!x693.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x692.value)))+(x693.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x694=IKcos(j5);
IkReal x695=IKsin(j5);
IkReal x696=((6.57894736842105)*pp);
IkReal x697=(npz*x695);
IkReal x698=((1.0)*x694);
IkReal x699=((1.94910848583668e-11)+x697);
evalcond[0]=((((0.0579734502884649)*x694))+x699+(((-1.0)*x694*x696)));
evalcond[1]=((0.0550995831252928)+(((0.0579734502884649)*x695))+(((-1.0)*x695*x696))+(((-1.0)*npz*x698)));
evalcond[2]=((((-1.0)*cj6*npx*x698))+(((0.076)*x694))+x699+((npy*sj6*x694)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x700=IKPowWithIntegerCheck(IKsign(((-50425664951339.0)+(((1.14448180704022e+16)*pp))+(((-6.49390844875346e+17)*(pp*pp)))+(((-1.500352608e+16)*(npz*npz))))),-1);
if(!x700.valid){
continue;
}
CheckValue<IkReal> x701 = IKatan2WithCheck(IkReal(((47925957551422.4)+(((-5.43873705537801e+15)*pp))+(((292435.0)*npz)))),IkReal(((16953.4659351072)+(((-1923914.47368421)*pp))+(((-826688032417458.0)*npz)))),IKFAST_ATAN2_MAGTHRESH);
if(!x701.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x700.value)))+(x701.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x702=IKcos(j5);
IkReal x703=IKsin(j5);
IkReal x704=((6.57894736842105)*pp);
IkReal x705=(npz*x703);
IkReal x706=((1.0)*x702);
IkReal x707=((1.94910848583668e-11)+x705);
evalcond[0]=((((-1.0)*x702*x704))+(((0.0579734502884649)*x702))+x707);
evalcond[1]=((0.0550995831252928)+(((0.0579734502884649)*x703))+(((-1.0)*x703*x704))+(((-1.0)*npz*x706)));
evalcond[2]=((((-1.0)*cj6*npx*x706))+(((0.076)*x702))+x707+((npy*sj6*x702)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[2];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
sj4=0;
cj4=-1.0;
j4=3.14159265358979;
IkReal x708=pp*pp;
IkReal x709=npz*npz;
j5eval[0]=((1.0)+(((12878.1810909586)*x708))+(((297.537495925507)*x709))+(((-226.964147749891)*pp)));
j5eval[1]=IKsign(((50425664951339.0)+(((6.49390844875346e+17)*x708))+(((-1.14448180704022e+16)*pp))+(((1.500352608e+16)*x709))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.426566392851566;
cj3=0.90445625;
j3=-0.440691687069409;
sj4=0;
cj4=-1.0;
j4=3.14159265358979;
IkReal x710=npz*npz;
IkReal x711=(cj6*npx);
IkReal x712=(npy*sj6);
IkReal x713=((1493.18518256507)*pp);
IkReal x714=((9.87074084210526e+16)*pp);
j5eval[0]=((-1.0)+(((-1.0)*x711*x713))+(((-226.964147749891)*x710))+(((-13.1578947368421)*x712))+(((13.1578947368421)*x711))+(((113.482073874945)*pp))+((x712*x713)));
j5eval[1]=IKsign(((-66105269174643.1)+(((869806173350567.0)*x711))+(((-1.500352608e+16)*x710))+(((-1.0)*x711*x714))+(((7.50176304e+15)*pp))+(((-869806173350567.0)*x712))+((x712*x714))));
j5eval[2]=((IKabs(((62828290463726.8)+(((826688032417458.0)*x712))+(((-292435.0)*npz))+(((-826688032417458.0)*x711)))))+(IKabs(((-16953.4659351072)+(((-826688032417458.0)*npz))+(((1923914.47368421)*pp))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x715=(cj6*npx);
IkReal x716=(npy*sj6);
IkReal x717=((9.87074084210526e+16)*pp);
CheckValue<IkReal> x718=IKPowWithIntegerCheck(IKsign(((-66105269174643.1)+(((869806173350567.0)*x715))+(((7.50176304e+15)*pp))+(((-1.500352608e+16)*(npz*npz)))+((x716*x717))+(((-869806173350567.0)*x716))+(((-1.0)*x715*x717)))),-1);
if(!x718.valid){
continue;
}
CheckValue<IkReal> x719 = IKatan2WithCheck(IkReal(((62828290463726.8)+(((826688032417458.0)*x716))+(((-292435.0)*npz))+(((-826688032417458.0)*x715)))),IkReal(((-16953.4659351072)+(((-826688032417458.0)*npz))+(((1923914.47368421)*pp)))),IKFAST_ATAN2_MAGTHRESH);
if(!x719.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x718.value)))+(x719.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x720=IKcos(j5);
IkReal x721=IKsin(j5);
IkReal x722=((6.57894736842105)*pp);
IkReal x723=(npz*x721);
IkReal x724=((1.0)*x720);
evalcond[0]=((-1.94910848583668e-11)+(((0.0579734502884649)*x720))+(((-1.0)*x720*x722))+x723);
evalcond[1]=((0.0550995831252928)+(((0.0579734502884649)*x721))+(((-1.0)*npz*x724))+(((-1.0)*x721*x722)));
evalcond[2]=((1.94910848583668e-11)+(((-0.076)*x720))+((cj6*npx*x720))+(((-1.0)*npy*sj6*x724))+(((-1.0)*x723)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x725 = IKatan2WithCheck(IkReal(((-47925957551422.4)+(((292435.0)*npz))+(((5.43873705537801e+15)*pp)))),IkReal(((16953.4659351072)+(((826688032417458.0)*npz))+(((-1923914.47368421)*pp)))),IKFAST_ATAN2_MAGTHRESH);
if(!x725.valid){
continue;
}
CheckValue<IkReal> x726=IKPowWithIntegerCheck(IKsign(((50425664951339.0)+(((-1.14448180704022e+16)*pp))+(((6.49390844875346e+17)*(pp*pp)))+(((1.500352608e+16)*(npz*npz))))),-1);
if(!x726.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x725.value)+(((1.5707963267949)*(x726.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x727=IKcos(j5);
IkReal x728=IKsin(j5);
IkReal x729=((6.57894736842105)*pp);
IkReal x730=(npz*x728);
IkReal x731=((1.0)*x727);
evalcond[0]=((-1.94910848583668e-11)+(((-1.0)*x727*x729))+(((0.0579734502884649)*x727))+x730);
evalcond[1]=((0.0550995831252928)+(((0.0579734502884649)*x728))+(((-1.0)*npz*x731))+(((-1.0)*x728*x729)));
evalcond[2]=((1.94910848583668e-11)+(((-0.076)*x727))+((cj6*npx*x727))+(((-1.0)*npy*sj6*x731))+(((-1.0)*x730)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j5]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x732=(npx*sj6);
IkReal x733=(cj6*npy);
IkReal x734=((144967695558428.0)*sj4);
IkReal x735=(cj6*npx);
IkReal x736=(npy*sj6);
IkReal x737=((137781338736243.0)*cj4);
IkReal x738=((144967695558428.0)*cj4);
IkReal x739=((1.64512347368421e+16)*pp*sj4);
IkReal x740=((1.64512347368421e+16)*cj4*pp);
IkReal x741=((2.50058768e+15)*npz*sj4);
CheckValue<IkReal> x742=IKPowWithIntegerCheck(IKsign(((((-1.0)*x736*x738))+((x735*x738))+(((-11017544862440.5)*cj4))+(((1.25029384e+15)*cj4*pp))+(((-1.0)*x735*x740))+(((-2.50058768e+15)*cj4*(npz*npz)))+((x736*x740)))),-1);
if(!x742.valid){
continue;
}
CheckValue<IkReal> x743 = IKatan2WithCheck(IkReal(((((-1.0)*x735*x737))+((x736*x737))+((x732*x741))+(((48739.1666666667)*npz))+((x733*x741))+(((10471381743954.5)*cj4)))),IkReal(((2825.57765585121)+(((-1.0)*x732*x739))+(((-1.0)*npz*x737))+(((-1.0)*x733*x739))+((x733*x734))+(((-320652.412280702)*pp))+((x732*x734)))),IKFAST_ATAN2_MAGTHRESH);
if(!x743.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x742.value)))+(x743.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x744=IKsin(j5);
IkReal x745=IKcos(j5);
IkReal x746=(sj4*sj6);
IkReal x747=((1.0)*cj4);
IkReal x748=(cj6*npy);
IkReal x749=((6.57894736842105)*pp);
IkReal x750=(npy*x745);
IkReal x751=(npz*x744);
IkReal x752=((0.076)*x745);
IkReal x753=(cj6*npx*x745);
evalcond[0]=((0.0550995831252928)+(((0.0579734502884649)*x744))+(((-1.0)*npz*x745))+(((-1.0)*x744*x749)));
evalcond[1]=((((-1.0)*x745*x749))+(((0.0579734502884649)*x745))+(((1.94910848583668e-11)*cj4))+x751);
evalcond[2]=((1.94910848583668e-11)+((sj4*x748))+((npx*x746))+((cj4*sj6*x750))+(((-1.0)*x747*x753))+((cj4*x752))+((cj4*x751)));
evalcond[3]=(((sj4*x752))+((sj4*x751))+(((-1.0)*x747*x748))+((x746*x750))+(((-1.0)*sj4*x753))+(((-1.0)*npx*sj6*x747)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x754=((144967695558428.0)*npy);
IkReal x755=(cj4*cj6);
IkReal x756=((137781338736243.0)*sj4);
IkReal x757=(npy*sj6);
IkReal x758=(cj6*npx);
IkReal x759=((1.64512347368421e+16)*pp);
IkReal x760=((2.50058768e+15)*npz);
IkReal x761=(cj4*npx*sj6);
CheckValue<IkReal> x762 = IKatan2WithCheck(IkReal(((((-1.0)*npy*x755*x760))+((x756*x757))+(((10471381743954.5)*sj4))+(((-1.0)*x756*x758))+(((-1.0)*x760*x761)))),IkReal(((((-1.0)*npz*x756))+(((-1.0)*x754*x755))+((npy*x755*x759))+(((-144967695558428.0)*x761))+((x759*x761)))),IKFAST_ATAN2_MAGTHRESH);
if(!x762.valid){
continue;
}
CheckValue<IkReal> x763=IKPowWithIntegerCheck(IKsign(((((-1.0)*sj4*sj6*x754))+(((-11017544862440.5)*sj4))+(((144967695558428.0)*sj4*x758))+(((1.25029384e+15)*pp*sj4))+(((-1.0)*sj4*x758*x759))+(((-1.0)*npz*sj4*x760))+((sj4*x757*x759)))),-1);
if(!x763.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x762.value)+(((1.5707963267949)*(x763.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x764=IKsin(j5);
IkReal x765=IKcos(j5);
IkReal x766=(sj4*sj6);
IkReal x767=((1.0)*cj4);
IkReal x768=(cj6*npy);
IkReal x769=((6.57894736842105)*pp);
IkReal x770=(npy*x765);
IkReal x771=(npz*x764);
IkReal x772=((0.076)*x765);
IkReal x773=(cj6*npx*x765);
evalcond[0]=((0.0550995831252928)+(((-1.0)*x764*x769))+(((-1.0)*npz*x765))+(((0.0579734502884649)*x764)));
evalcond[1]=((((-1.0)*x765*x769))+(((1.94910848583668e-11)*cj4))+x771+(((0.0579734502884649)*x765)));
evalcond[2]=((1.94910848583668e-11)+(((-1.0)*x767*x773))+((cj4*sj6*x770))+((sj4*x768))+((cj4*x771))+((cj4*x772))+((npx*x766)));
evalcond[3]=(((sj4*x771))+((sj4*x772))+(((-1.0)*sj4*x773))+(((-1.0)*x767*x768))+((x766*x770))+(((-1.0)*npx*sj6*x767)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x774 = IKatan2WithCheck(IkReal(((47925957551422.4)+(((-5.43873705537801e+15)*pp))+(((292435.0)*cj4*npz)))),IkReal(((((-826688032417458.0)*npz))+(((-1923914.47368421)*cj4*pp))+(((16953.4659351072)*cj4)))),IKFAST_ATAN2_MAGTHRESH);
if(!x774.valid){
continue;
}
CheckValue<IkReal> x775=IKPowWithIntegerCheck(IKsign(((-50425664951339.0)+(((1.14448180704022e+16)*pp))+(((-6.49390844875346e+17)*(pp*pp)))+(((-1.500352608e+16)*(npz*npz))))),-1);
if(!x775.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x774.value)+(((1.5707963267949)*(x775.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x776=IKsin(j5);
IkReal x777=IKcos(j5);
IkReal x778=(sj4*sj6);
IkReal x779=((1.0)*cj4);
IkReal x780=(cj6*npy);
IkReal x781=((6.57894736842105)*pp);
IkReal x782=(npy*x777);
IkReal x783=(npz*x776);
IkReal x784=((0.076)*x777);
IkReal x785=(cj6*npx*x777);
evalcond[0]=((0.0550995831252928)+(((0.0579734502884649)*x776))+(((-1.0)*npz*x777))+(((-1.0)*x776*x781)));
evalcond[1]=((((1.94910848583668e-11)*cj4))+(((0.0579734502884649)*x777))+(((-1.0)*x777*x781))+x783);
evalcond[2]=((1.94910848583668e-11)+((cj4*sj6*x782))+((npx*x778))+((sj4*x780))+((cj4*x783))+((cj4*x784))+(((-1.0)*x779*x785)));
evalcond[3]=((((-1.0)*npx*sj6*x779))+((sj4*x783))+((sj4*x784))+((x778*x782))+(((-1.0)*sj4*x785))+(((-1.0)*x779*x780)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.2276868576215)+j3)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
sj4array[0]=((((819672131.147541)*npx*sj6))+(((819672131.147541)*cj6*npy)));
if( sj4array[0] >= -1-IKFAST_SINCOS_THRESH && sj4array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j4valid[0] = j4valid[1] = true;
    j4array[0] = IKasin(sj4array[0]);
    cj4array[0] = IKcos(j4array[0]);
    sj4array[1] = sj4array[0];
    j4array[1] = j4array[0] > 0 ? (IKPI-j4array[0]) : (-IKPI-j4array[0]);
    cj4array[1] = -cj4array[0];
}
else if( isnan(sj4array[0]) )
{
    // probably any value will work
    j4valid[0] = true;
    cj4array[0] = 1; sj4array[0] = 0; j4array[0] = 0;
}
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
IkReal j5eval[3];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
IkReal x786=npz*npz;
IkReal x787=pp*pp;
j5eval[0]=((7.01380424724537)+(((-34.8468261014262)*pp))+(((43.2825484764543)*x787))+x786);
j5eval[1]=IKsign(((1402760849449.07)+(((-6969365220285.23)*pp))+(((200000000000.0)*x786))+(((8656509695290.86)*x787))));
j5eval[2]=((IKabs(((((-1605.26315789474)*cj4*pp))+(((646.199543224847)*cj4))+(((125980083303.0)*npz)))))+(IKabs(((-333640460187.815)+(((244.0)*cj4*npz))+(((828816337519.737)*pp))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
IkReal x788=npz*npz;
IkReal x789=((529671756741.678)*sj6);
IkReal x790=(cj4*npx);
IkReal x791=((125980083303.0)*sj4);
IkReal x792=(cj6*npx);
IkReal x793=(npy*sj4);
IkReal x794=((1315789473684.21)*pp);
IkReal x795=((200000000000.0)*npz);
IkReal x796=(pp*sj4);
IkReal x797=(cj4*cj6*npy);
IkReal x798=(sj4*x788);
j5eval[0]=((((-1.0)*sj4))+(((2.48416015567529)*x796))+(((-4.96832031135058)*x798))+(((32.6863178378328)*pp*sj6*x793))+(((13.1578947368421)*sj4*x792))+(((-13.1578947368421)*sj6*x793))+(((-32.6863178378328)*x792*x796)));
j5eval[1]=((IKabs((((npy*sj6*x791))+(((-1.0)*x795*x797))+(((9574486331.028)*sj4))+(((-1.0)*x791*x792))+(((-1.0)*sj6*x790*x795)))))+(IKabs(((((-1.0)*npz*x791))+(((-529671756741.678)*x797))+(((-1.0)*x789*x790))+((x794*x797))+((sj6*x790*x794))))));
j5eval[2]=IKsign((((sj6*x793*x794))+(((-200000000000.0)*x798))+(((100000000000.0)*x796))+(((-40255053512.3675)*sj4))+(((-1.0)*sj4*x792*x794))+(((-1.0)*x789*x793))+(((529671756741.678)*sj4*x792))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
IkReal x799=npz*npz;
IkReal x800=((32.6863178378328)*pp);
IkReal x801=(cj4*pp);
IkReal x802=((1315789473684.21)*pp);
IkReal x803=(cj4*npy*sj6);
IkReal x804=(cj4*cj6*npx);
IkReal x805=(cj4*x799);
j5eval[0]=((((2.48416015567529)*x801))+(((-4.96832031135058)*x805))+(((13.1578947368421)*x804))+(((-13.1578947368421)*x803))+((x800*x803))+(((-1.0)*x800*x804))+(((-1.0)*cj4)));
j5eval[1]=IKsign(((((100000000000.0)*x801))+(((-200000000000.0)*x805))+(((529671756741.678)*x804))+(((-529671756741.678)*x803))+(((-40255053512.3675)*cj4))+(((1315789473684.21)*npy*sj6*x801))+(((-1315789473684.21)*cj6*npx*x801))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[3];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
sj4=1.0;
cj4=0;
j4=1.5707963267949;
IkReal x806=npz*npz;
IkReal x807=pp*pp;
j5eval[0]=((-7.01380424724537)+(((-1.0)*x806))+(((-43.2825484764543)*x807))+(((34.8468261014262)*pp)));
j5eval[1]=IKsign(((-1402760849449.07)+(((-200000000000.0)*x806))+(((6969365220285.23)*pp))+(((-8656509695290.86)*x807))));
j5eval[2]=((IKabs(npz))+(((7.9377626509014e-12)*(IKabs(((333640460187.815)+(((-828816337519.737)*pp))))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
sj4=1.0;
cj4=0;
j4=1.5707963267949;
IkReal x808=npz*npz;
IkReal x809=((1315789473684.21)*pp);
IkReal x810=(cj6*npx);
IkReal x811=(npy*sj6);
IkReal x812=((32.6863178378328)*pp);
j5eval[0]=((-1.0)+(((-4.96832031135058)*x808))+(((2.48416015567529)*pp))+(((-1.0)*x810*x812))+((x811*x812))+(((13.1578947368421)*x810))+(((-13.1578947368421)*x811)));
j5eval[1]=((IKabs(npz))+(((7.9377626509014e-12)*(IKabs(((9574486331.028)+(((125980083303.0)*x811))+(((-125980083303.0)*x810))))))));
j5eval[2]=IKsign(((-40255053512.3675)+(((-529671756741.678)*x811))+(((-200000000000.0)*x808))+((x809*x811))+(((100000000000.0)*pp))+(((-1.0)*x809*x810))+(((529671756741.678)*x810))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x813=((1315789473684.21)*pp);
IkReal x814=(npy*sj6);
IkReal x815=(cj6*npx);
CheckValue<IkReal> x816=IKPowWithIntegerCheck(IKsign(((-40255053512.3675)+(((-529671756741.678)*x814))+(((100000000000.0)*pp))+((x813*x814))+(((-1.0)*x813*x815))+(((529671756741.678)*x815))+(((-200000000000.0)*(npz*npz))))),-1);
if(!x816.valid){
continue;
}
CheckValue<IkReal> x817 = IKatan2WithCheck(IkReal(((9574486331.028)+(((125980083303.0)*x814))+(((-125980083303.0)*x815)))),IkReal(((-125980083303.0)*npz)),IKFAST_ATAN2_MAGTHRESH);
if(!x817.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x816.value)))+(x817.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x818=IKcos(j5);
IkReal x819=IKsin(j5);
IkReal x820=((6.57894736842105)*pp);
IkReal x821=(npz*x819);
IkReal x822=((1.0)*x818);
evalcond[0]=((((-1.0)*x818*x820))+(((2.64835878370839)*x818))+x821);
evalcond[1]=((0.629900416515)+(((-1.0)*npz*x822))+(((-1.0)*x819*x820))+(((2.64835878370839)*x819)));
evalcond[2]=((((-1.0)*cj6*npx*x822))+(((0.076)*x818))+((npy*sj6*x818))+x821);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x823 = IKatan2WithCheck(IkReal(((333640460187.815)+(((-828816337519.737)*pp)))),IkReal(((-125980083303.0)*npz)),IKFAST_ATAN2_MAGTHRESH);
if(!x823.valid){
continue;
}
CheckValue<IkReal> x824=IKPowWithIntegerCheck(IKsign(((-1402760849449.07)+(((6969365220285.23)*pp))+(((-8656509695290.86)*(pp*pp)))+(((-200000000000.0)*(npz*npz))))),-1);
if(!x824.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x823.value)+(((1.5707963267949)*(x824.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x825=IKcos(j5);
IkReal x826=IKsin(j5);
IkReal x827=((6.57894736842105)*pp);
IkReal x828=(npz*x826);
IkReal x829=((1.0)*x825);
evalcond[0]=((((2.64835878370839)*x825))+x828+(((-1.0)*x825*x827)));
evalcond[1]=((0.629900416515)+(((-1.0)*npz*x829))+(((-1.0)*x826*x827))+(((2.64835878370839)*x826)));
evalcond[2]=((((0.076)*x825))+(((-1.0)*cj6*npx*x829))+x828+((npy*sj6*x825)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[3];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
sj4=-1.0;
cj4=0;
j4=-1.5707963267949;
IkReal x830=npz*npz;
IkReal x831=pp*pp;
j5eval[0]=((-7.01380424724537)+(((34.8468261014262)*pp))+(((-1.0)*x830))+(((-43.2825484764543)*x831)));
j5eval[1]=IKsign(((-1402760849449.07)+(((6969365220285.23)*pp))+(((-200000000000.0)*x830))+(((-8656509695290.86)*x831))));
j5eval[2]=((IKabs(npz))+(((7.9377626509014e-12)*(IKabs(((333640460187.815)+(((-828816337519.737)*pp))))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
sj4=-1.0;
cj4=0;
j4=-1.5707963267949;
IkReal x832=npz*npz;
IkReal x833=((1315789473684.21)*pp);
IkReal x834=(cj6*npx);
IkReal x835=(npy*sj6);
IkReal x836=((32.6863178378328)*pp);
j5eval[0]=((1.0)+(((13.1578947368421)*x835))+(((4.96832031135058)*x832))+(((-1.0)*x835*x836))+(((-2.48416015567529)*pp))+((x834*x836))+(((-13.1578947368421)*x834)));
j5eval[1]=IKsign(((40255053512.3675)+(((-529671756741.678)*x834))+(((-1.0)*x833*x835))+((x833*x834))+(((-100000000000.0)*pp))+(((200000000000.0)*x832))+(((529671756741.678)*x835))));
j5eval[2]=((((7.9377626509014e-12)*(IKabs(((-9574486331.028)+(((-125980083303.0)*x835))+(((125980083303.0)*x834)))))))+(IKabs(npz)));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x837=(npy*sj6);
IkReal x838=((1315789473684.21)*pp);
IkReal x839=(cj6*npx);
CheckValue<IkReal> x840=IKPowWithIntegerCheck(IKsign(((40255053512.3675)+(((200000000000.0)*(npz*npz)))+((x838*x839))+(((-529671756741.678)*x839))+(((-1.0)*x837*x838))+(((-100000000000.0)*pp))+(((529671756741.678)*x837)))),-1);
if(!x840.valid){
continue;
}
CheckValue<IkReal> x841 = IKatan2WithCheck(IkReal(((-9574486331.028)+(((-125980083303.0)*x837))+(((125980083303.0)*x839)))),IkReal(((125980083303.0)*npz)),IKFAST_ATAN2_MAGTHRESH);
if(!x841.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x840.value)))+(x841.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x842=IKcos(j5);
IkReal x843=IKsin(j5);
IkReal x844=((6.57894736842105)*pp);
IkReal x845=(npz*x843);
IkReal x846=((1.0)*x842);
evalcond[0]=((((2.64835878370839)*x842))+(((-1.0)*x842*x844))+x845);
evalcond[1]=((0.629900416515)+(((2.64835878370839)*x843))+(((-1.0)*npz*x846))+(((-1.0)*x843*x844)));
evalcond[2]=((((-0.076)*x842))+((cj6*npx*x842))+(((-1.0)*npy*sj6*x846))+(((-1.0)*x845)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x847 = IKatan2WithCheck(IkReal(((333640460187.815)+(((-828816337519.737)*pp)))),IkReal(((-125980083303.0)*npz)),IKFAST_ATAN2_MAGTHRESH);
if(!x847.valid){
continue;
}
CheckValue<IkReal> x848=IKPowWithIntegerCheck(IKsign(((-1402760849449.07)+(((6969365220285.23)*pp))+(((-8656509695290.86)*(pp*pp)))+(((-200000000000.0)*(npz*npz))))),-1);
if(!x848.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x847.value)+(((1.5707963267949)*(x848.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x849=IKcos(j5);
IkReal x850=IKsin(j5);
IkReal x851=((6.57894736842105)*pp);
IkReal x852=(npz*x850);
IkReal x853=((1.0)*x849);
evalcond[0]=((((2.64835878370839)*x849))+(((-1.0)*x849*x851))+x852);
evalcond[1]=((0.629900416515)+(((-1.0)*x850*x851))+(((2.64835878370839)*x850))+(((-1.0)*npz*x853)));
evalcond[2]=((((-0.076)*x849))+((cj6*npx*x849))+(((-1.0)*x852))+(((-1.0)*npy*sj6*x853)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j4))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[2];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
sj4=0;
cj4=1.0;
j4=0;
IkReal x854=npz*npz;
IkReal x855=pp*pp;
j5eval[0]=((7.01380424724537)+(((-34.8468261014262)*pp))+(((43.2825484764543)*x855))+x854);
j5eval[1]=IKsign(((1402760849449.07)+(((-6969365220285.23)*pp))+(((200000000000.0)*x854))+(((8656509695290.86)*x855))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
sj4=0;
cj4=1.0;
j4=0;
IkReal x856=npz*npz;
IkReal x857=((1315789473684.21)*pp);
IkReal x858=(cj6*npx);
IkReal x859=(npy*sj6);
IkReal x860=((32.6863178378328)*pp);
j5eval[0]=((1.0)+(((13.1578947368421)*x859))+(((4.96832031135058)*x856))+((x858*x860))+(((-13.1578947368421)*x858))+(((-2.48416015567529)*pp))+(((-1.0)*x859*x860)));
j5eval[1]=IKsign(((40255053512.3675)+(((200000000000.0)*x856))+(((529671756741.678)*x859))+(((-100000000000.0)*pp))+(((-529671756741.678)*x858))+((x857*x858))+(((-1.0)*x857*x859))));
j5eval[2]=((IKabs(((-9574486331.028)+(((244.0)*npz))+(((125980083303.0)*x858))+(((-125980083303.0)*x859)))))+(IKabs(((646.199543224847)+(((-1605.26315789474)*pp))+(((125980083303.0)*npz))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x861=(npy*sj6);
IkReal x862=((1315789473684.21)*pp);
IkReal x863=(cj6*npx);
CheckValue<IkReal> x864=IKPowWithIntegerCheck(IKsign(((40255053512.3675)+(((200000000000.0)*(npz*npz)))+(((-1.0)*x861*x862))+(((-100000000000.0)*pp))+(((529671756741.678)*x861))+((x862*x863))+(((-529671756741.678)*x863)))),-1);
if(!x864.valid){
continue;
}
CheckValue<IkReal> x865 = IKatan2WithCheck(IkReal(((-9574486331.028)+(((244.0)*npz))+(((125980083303.0)*x863))+(((-125980083303.0)*x861)))),IkReal(((646.199543224847)+(((-1605.26315789474)*pp))+(((125980083303.0)*npz)))),IKFAST_ATAN2_MAGTHRESH);
if(!x865.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x864.value)))+(x865.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x866=IKcos(j5);
IkReal x867=IKsin(j5);
IkReal x868=((6.57894736842105)*pp);
IkReal x869=(npz*x867);
IkReal x870=((1.0)*x866);
IkReal x871=((-1.22e-9)+x869);
evalcond[0]=((((-1.0)*x866*x868))+(((2.64835878370839)*x866))+x871);
evalcond[1]=((0.629900416515)+(((2.64835878370839)*x867))+(((-1.0)*npz*x870))+(((-1.0)*x867*x868)));
evalcond[2]=((((0.076)*x866))+((npy*sj6*x866))+x871+(((-1.0)*cj6*npx*x870)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x872 = IKatan2WithCheck(IkReal(((-333640460187.815)+(((244.0)*npz))+(((828816337519.737)*pp)))),IkReal(((646.199543224847)+(((-1605.26315789474)*pp))+(((125980083303.0)*npz)))),IKFAST_ATAN2_MAGTHRESH);
if(!x872.valid){
continue;
}
CheckValue<IkReal> x873=IKPowWithIntegerCheck(IKsign(((1402760849449.07)+(((200000000000.0)*(npz*npz)))+(((-6969365220285.23)*pp))+(((8656509695290.86)*(pp*pp))))),-1);
if(!x873.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x872.value)+(((1.5707963267949)*(x873.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x874=IKcos(j5);
IkReal x875=IKsin(j5);
IkReal x876=((6.57894736842105)*pp);
IkReal x877=(npz*x875);
IkReal x878=((1.0)*x874);
IkReal x879=((-1.22e-9)+x877);
evalcond[0]=((((2.64835878370839)*x874))+(((-1.0)*x874*x876))+x879);
evalcond[1]=((0.629900416515)+(((2.64835878370839)*x875))+(((-1.0)*npz*x878))+(((-1.0)*x875*x876)));
evalcond[2]=((((0.076)*x874))+((npy*sj6*x874))+x879+(((-1.0)*cj6*npx*x878)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[2];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
sj4=0;
cj4=-1.0;
j4=3.14159265358979;
IkReal x880=npz*npz;
IkReal x881=pp*pp;
j5eval[0]=((-7.01380424724537)+(((-43.2825484764543)*x881))+(((-1.0)*x880))+(((34.8468261014262)*pp)));
j5eval[1]=IKsign(((-1402760849449.07)+(((6969365220285.23)*pp))+(((-200000000000.0)*x880))+(((-8656509695290.86)*x881))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[3];
sj3=-0.08598789;
cj3=-0.99629618;
j3=3.22768551701203;
sj4=0;
cj4=-1.0;
j4=3.14159265358979;
IkReal x882=npz*npz;
IkReal x883=((1315789473684.21)*pp);
IkReal x884=(cj6*npx);
IkReal x885=(npy*sj6);
IkReal x886=((32.6863178378328)*pp);
j5eval[0]=((-1.0)+(((2.48416015567529)*pp))+(((-4.96832031135058)*x882))+(((-13.1578947368421)*x885))+((x885*x886))+(((13.1578947368421)*x884))+(((-1.0)*x884*x886)));
j5eval[1]=IKsign(((-40255053512.3675)+(((-1.0)*x883*x884))+(((-529671756741.678)*x885))+((x883*x885))+(((100000000000.0)*pp))+(((-200000000000.0)*x882))+(((529671756741.678)*x884))));
j5eval[2]=((IKabs(((646.199543224847)+(((-1605.26315789474)*pp))+(((-125980083303.0)*npz)))))+(IKabs(((9574486331.028)+(((125980083303.0)*x885))+(((244.0)*npz))+(((-125980083303.0)*x884))))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  || IKabs(j5eval[2]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x887=((1315789473684.21)*pp);
IkReal x888=(npy*sj6);
IkReal x889=(cj6*npx);
CheckValue<IkReal> x890=IKPowWithIntegerCheck(IKsign(((-40255053512.3675)+(((-1.0)*x887*x889))+((x887*x888))+(((-529671756741.678)*x888))+(((100000000000.0)*pp))+(((-200000000000.0)*(npz*npz)))+(((529671756741.678)*x889)))),-1);
if(!x890.valid){
continue;
}
CheckValue<IkReal> x891 = IKatan2WithCheck(IkReal(((9574486331.028)+(((125980083303.0)*x888))+(((244.0)*npz))+(((-125980083303.0)*x889)))),IkReal(((646.199543224847)+(((-1605.26315789474)*pp))+(((-125980083303.0)*npz)))),IKFAST_ATAN2_MAGTHRESH);
if(!x891.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x890.value)))+(x891.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x892=IKcos(j5);
IkReal x893=IKsin(j5);
IkReal x894=((6.57894736842105)*pp);
IkReal x895=(npz*x893);
IkReal x896=((1.0)*x892);
evalcond[0]=((1.22e-9)+(((-1.0)*x892*x894))+(((2.64835878370839)*x892))+x895);
evalcond[1]=((0.629900416515)+(((-1.0)*npz*x896))+(((-1.0)*x893*x894))+(((2.64835878370839)*x893)));
evalcond[2]=((-1.22e-9)+((cj6*npx*x892))+(((-1.0)*x895))+(((-1.0)*npy*sj6*x896))+(((-0.076)*x892)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x897=IKPowWithIntegerCheck(IKsign(((-1402760849449.07)+(((6969365220285.23)*pp))+(((-8656509695290.86)*(pp*pp)))+(((-200000000000.0)*(npz*npz))))),-1);
if(!x897.valid){
continue;
}
CheckValue<IkReal> x898 = IKatan2WithCheck(IkReal(((333640460187.815)+(((244.0)*npz))+(((-828816337519.737)*pp)))),IkReal(((646.199543224847)+(((-1605.26315789474)*pp))+(((-125980083303.0)*npz)))),IKFAST_ATAN2_MAGTHRESH);
if(!x898.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x897.value)))+(x898.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[3];
IkReal x899=IKcos(j5);
IkReal x900=IKsin(j5);
IkReal x901=((6.57894736842105)*pp);
IkReal x902=(npz*x900);
IkReal x903=((1.0)*x899);
evalcond[0]=((1.22e-9)+(((2.64835878370839)*x899))+x902+(((-1.0)*x899*x901)));
evalcond[1]=((0.629900416515)+(((-1.0)*x900*x901))+(((-1.0)*npz*x903))+(((2.64835878370839)*x900)));
evalcond[2]=((-1.22e-9)+((cj6*npx*x899))+(((-1.0)*npy*sj6*x903))+(((-1.0)*x902))+(((-0.076)*x899)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j5]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x904=((200000000000.0)*npz);
IkReal x905=((529671756741.678)*cj4);
IkReal x906=(npy*sj6);
IkReal x907=((125980083303.0)*cj4);
IkReal x908=(cj6*npx);
IkReal x909=((1315789473684.21)*pp);
IkReal x910=(npx*sj4*sj6);
IkReal x911=(cj6*npy*sj4);
CheckValue<IkReal> x912 = IKatan2WithCheck(IkReal(((((9574486331.028)*cj4))+((x906*x907))+(((-244.0)*npz))+((x904*x910))+((x904*x911))+(((-1.0)*x907*x908)))),IkReal(((-646.199543224847)+(((1605.26315789474)*pp))+(((-1.0)*npz*x907))+(((529671756741.678)*x910))+(((529671756741.678)*x911))+(((-1.0)*x909*x910))+(((-1.0)*x909*x911)))),IKFAST_ATAN2_MAGTHRESH);
if(!x912.valid){
continue;
}
CheckValue<IkReal> x913=IKPowWithIntegerCheck(IKsign((((x905*x908))+((cj4*x906*x909))+(((-40255053512.3675)*cj4))+(((-1.0)*cj4*x908*x909))+(((100000000000.0)*cj4*pp))+(((-1.0)*x905*x906))+(((-1.0)*cj4*npz*x904)))),-1);
if(!x913.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x912.value)+(((1.5707963267949)*(x913.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x914=IKsin(j5);
IkReal x915=IKcos(j5);
IkReal x916=(sj4*sj6);
IkReal x917=((1.0)*cj4);
IkReal x918=(cj6*npy);
IkReal x919=((6.57894736842105)*pp);
IkReal x920=(npy*x915);
IkReal x921=(npz*x914);
IkReal x922=((0.076)*x915);
IkReal x923=(cj6*npx*x915);
evalcond[0]=((0.629900416515)+(((2.64835878370839)*x914))+(((-1.0)*npz*x915))+(((-1.0)*x914*x919)));
evalcond[1]=((((2.64835878370839)*x915))+(((-1.22e-9)*cj4))+x921+(((-1.0)*x915*x919)));
evalcond[2]=((-1.22e-9)+((npx*x916))+((cj4*sj6*x920))+((cj4*x922))+((cj4*x921))+(((-1.0)*x917*x923))+((sj4*x918)));
evalcond[3]=(((x916*x920))+(((-1.0)*npx*sj6*x917))+(((-1.0)*sj4*x923))+((sj4*x922))+((sj4*x921))+(((-1.0)*x917*x918)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x924=((125980083303.0)*sj4);
IkReal x925=(cj6*npx);
IkReal x926=(cj6*npy);
IkReal x927=(npx*sj6);
IkReal x928=(npy*sj6);
IkReal x929=((529671756741.678)*sj4);
IkReal x930=(pp*sj4);
IkReal x931=((529671756741.678)*cj4);
IkReal x932=((200000000000.0)*cj4*npz);
IkReal x933=((1315789473684.21)*cj4*pp);
CheckValue<IkReal> x934 = IKatan2WithCheck(IkReal((((x924*x928))+(((9574486331.028)*sj4))+(((-1.0)*x924*x925))+(((-1.0)*x927*x932))+(((-1.0)*x926*x932)))),IkReal((((x927*x933))+(((-1.0)*x927*x931))+(((-1.0)*x926*x931))+((x926*x933))+(((-1.0)*npz*x924)))),IKFAST_ATAN2_MAGTHRESH);
if(!x934.valid){
continue;
}
CheckValue<IkReal> x935=IKPowWithIntegerCheck(IKsign(((((-200000000000.0)*sj4*(npz*npz)))+(((-1.0)*x928*x929))+(((100000000000.0)*x930))+(((-40255053512.3675)*sj4))+((x925*x929))+(((1315789473684.21)*x928*x930))+(((-1315789473684.21)*x925*x930)))),-1);
if(!x935.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x934.value)+(((1.5707963267949)*(x935.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x936=IKsin(j5);
IkReal x937=IKcos(j5);
IkReal x938=(sj4*sj6);
IkReal x939=((1.0)*cj4);
IkReal x940=(cj6*npy);
IkReal x941=((6.57894736842105)*pp);
IkReal x942=(npy*x937);
IkReal x943=(npz*x936);
IkReal x944=((0.076)*x937);
IkReal x945=(cj6*npx*x937);
evalcond[0]=((0.629900416515)+(((-1.0)*npz*x937))+(((2.64835878370839)*x936))+(((-1.0)*x936*x941)));
evalcond[1]=((((-1.22e-9)*cj4))+(((2.64835878370839)*x937))+(((-1.0)*x937*x941))+x943);
evalcond[2]=((-1.22e-9)+((cj4*sj6*x942))+(((-1.0)*x939*x945))+((sj4*x940))+((npx*x938))+((cj4*x944))+((cj4*x943)));
evalcond[3]=((((-1.0)*npx*sj6*x939))+(((-1.0)*sj4*x945))+(((-1.0)*x939*x940))+((x938*x942))+((sj4*x944))+((sj4*x943)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
CheckValue<IkReal> x946=IKPowWithIntegerCheck(IKsign(((1402760849449.07)+(((200000000000.0)*(npz*npz)))+(((-6969365220285.23)*pp))+(((8656509695290.86)*(pp*pp))))),-1);
if(!x946.valid){
continue;
}
CheckValue<IkReal> x947 = IKatan2WithCheck(IkReal(((-333640460187.815)+(((244.0)*cj4*npz))+(((828816337519.737)*pp)))),IkReal(((((-1605.26315789474)*cj4*pp))+(((646.199543224847)*cj4))+(((125980083303.0)*npz)))),IKFAST_ATAN2_MAGTHRESH);
if(!x947.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x946.value)))+(x947.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x948=IKsin(j5);
IkReal x949=IKcos(j5);
IkReal x950=(sj4*sj6);
IkReal x951=((1.0)*cj4);
IkReal x952=(cj6*npy);
IkReal x953=((6.57894736842105)*pp);
IkReal x954=(npy*x949);
IkReal x955=(npz*x948);
IkReal x956=((0.076)*x949);
IkReal x957=(cj6*npx*x949);
evalcond[0]=((0.629900416515)+(((-1.0)*npz*x949))+(((-1.0)*x948*x953))+(((2.64835878370839)*x948)));
evalcond[1]=((((-1.22e-9)*cj4))+(((-1.0)*x949*x953))+x955+(((2.64835878370839)*x949)));
evalcond[2]=((-1.22e-9)+((cj4*sj6*x954))+((npx*x950))+((sj4*x952))+(((-1.0)*x951*x957))+((cj4*x956))+((cj4*x955)));
evalcond[3]=(((sj4*x956))+((sj4*x955))+(((-1.0)*sj4*x957))+(((-1.0)*npx*sj6*x951))+(((-1.0)*x951*x952))+((x950*x954)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4, j5]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[2], cj4array[2], sj4array[2];
bool j4valid[2]={false};
_nj4 = 2;
CheckValue<IkReal> x958=IKPowWithIntegerCheck(((0.0775)+(((0.0525)*cj3))+(((0.293)*sj3))),-1);
if(!x958.valid){
continue;
}
sj4array[0]=((-1.0)*(x958.value)*((((cj6*npy))+((npx*sj6)))));
if( sj4array[0] >= -1-IKFAST_SINCOS_THRESH && sj4array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j4valid[0] = j4valid[1] = true;
    j4array[0] = IKasin(sj4array[0]);
    cj4array[0] = IKcos(j4array[0]);
    sj4array[1] = sj4array[0];
    j4array[1] = j4array[0] > 0 ? (IKPI-j4array[0]) : (-IKPI-j4array[0]);
    cj4array[1] = -cj4array[0];
}
else if( isnan(sj4array[0]) )
{
    // probably any value will work
    j4valid[0] = true;
    cj4array[0] = 1; sj4array[0] = 0; j4array[0] = 0;
}
for(int ij4 = 0; ij4 < 2; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 2; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];

{
IkReal j5eval[2];
IkReal x959=cj6*cj6;
IkReal x960=npy*npy;
IkReal x961=npz*npz;
IkReal x962=npx*npx;
IkReal x963=(npy*sj6);
IkReal x964=(cj6*npx);
IkReal x965=((173.130193905817)*x960);
IkReal x966=(x959*x962);
j5eval[0]=((-1.0)+(((-173.130193905817)*x961))+(((-173.130193905817)*x966))+(((-26.3157894736842)*x963))+(((26.3157894736842)*x964))+((x959*x965))+(((346.260387811634)*x963*x964))+(((-1.0)*x965)));
j5eval[1]=IKsign(((-0.005776)+(((0.152)*x964))+(((-1.0)*x966))+(((-1.0)*x960))+(((-1.0)*x961))+((x959*x960))+(((2.0)*x963*x964))+(((-0.152)*x963))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
IkReal x967=cj6*cj6;
IkReal x968=npy*npy;
IkReal x969=npx*npx;
IkReal x970=npz*npz;
IkReal x971=((1.0)*sj4);
IkReal x972=((173.130193905817)*sj4);
IkReal x973=(cj6*npx*sj4);
IkReal x974=(npy*sj4*sj6);
IkReal x975=(x967*x969);
IkReal x976=(x967*x968);
IkReal x977=(npy*sj6*x973);
j5eval[0]=((((-26.3157894736842)*x974))+(((26.3157894736842)*x973))+(((346.260387811634)*x977))+(((-1.0)*x970*x972))+(((-1.0)*x972*x975))+(((-1.0)*x968*x972))+(((-1.0)*x971))+((x972*x976)));
j5eval[1]=IKsign(((((-0.152)*x974))+(((0.152)*x973))+(((-1.0)*x971*x975))+((sj4*x976))+(((2.0)*x977))+(((-0.005776)*sj4))+(((-1.0)*x970*x971))+(((-1.0)*x968*x971))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal j5eval[2];
IkReal x978=cj6*cj6;
IkReal x979=npy*npy;
IkReal x980=npz*npz;
IkReal x981=npx*npx;
IkReal x982=((1.0)*cj4);
IkReal x983=((26.3157894736842)*cj4);
IkReal x984=(cj6*npx);
IkReal x985=(npy*sj6);
IkReal x986=((0.152)*cj4);
IkReal x987=((173.130193905817)*cj4);
IkReal x988=(x978*x981);
IkReal x989=(cj4*x984*x985);
IkReal x990=(cj4*x978*x979);
j5eval[0]=(((x983*x984))+(((-1.0)*x982))+((x978*x979*x987))+(((346.260387811634)*x989))+(((-1.0)*x987*x988))+(((-1.0)*x980*x987))+(((-1.0)*x979*x987))+(((-1.0)*x983*x985)));
j5eval[1]=IKsign(((((-0.005776)*cj4))+(((-1.0)*x982*x988))+(((2.0)*x989))+(((-1.0)*x980*x982))+((x984*x986))+(((-1.0)*x979*x982))+x990+(((-1.0)*x985*x986))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[2];
sj4=1.0;
cj4=0;
j4=1.5707963267949;
IkReal x991=cj6*cj6;
IkReal x992=npy*npy;
IkReal x993=npz*npz;
IkReal x994=npx*npx;
IkReal x995=(npy*sj6);
IkReal x996=(cj6*npx);
IkReal x997=((173.130193905817)*x992);
IkReal x998=(x991*x994);
j5eval[0]=((-1.0)+(((-173.130193905817)*x993))+(((-173.130193905817)*x998))+((x991*x997))+(((26.3157894736842)*x996))+(((346.260387811634)*x995*x996))+(((-1.0)*x997))+(((-26.3157894736842)*x995)));
j5eval[1]=IKsign(((-0.005776)+(((-1.0)*x993))+(((-1.0)*x992))+(((-1.0)*x998))+((x991*x992))+(((-0.152)*x995))+(((2.0)*x995*x996))+(((0.152)*x996))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x999=cj6*cj6;
IkReal x1000=npy*npy;
IkReal x1001=(cj6*npx);
IkReal x1002=(npy*sj6);
IkReal x1003=((0.0525)*sj3);
IkReal x1004=((0.293)*cj3);
CheckValue<IkReal> x1005=IKPowWithIntegerCheck(IKsign(((-0.005776)+(((-0.152)*x1002))+((x1000*x999))+(((-1.0)*x1000))+(((0.152)*x1001))+(((2.0)*x1001*x1002))+(((-1.0)*(npz*npz)))+(((-1.0)*x999*(npx*npx))))),-1);
if(!x1005.valid){
continue;
}
CheckValue<IkReal> x1006 = IKatan2WithCheck(IkReal(((0.02603)+(((0.3425)*x1002))+(((-1.0)*x1002*x1004))+(((0.00399)*sj3))+(((-1.0)*x1001*x1003))+((x1002*x1003))+((x1001*x1004))+(((-0.022268)*cj3))+(((-0.3425)*x1001)))),IkReal(((((-0.3425)*npz))+((npz*x1004))+(((-1.0)*npz*x1003)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1006.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1005.value)))+(x1006.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[2];
IkReal x1007=IKcos(j5);
IkReal x1008=IKsin(j5);
IkReal x1009=(npy*sj6);
IkReal x1010=((1.0)*cj6*npx);
evalcond[0]=(((npz*x1008))+((x1007*x1009))+(((0.076)*x1007))+(((-1.0)*x1007*x1010)));
evalcond[1]=((0.3425)+((x1008*x1009))+(((-1.0)*x1008*x1010))+(((-0.293)*cj3))+(((-1.0)*npz*x1007))+(((0.076)*x1008))+(((0.0525)*sj3)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[2];
sj4=-1.0;
cj4=0;
j4=-1.5707963267949;
IkReal x1011=cj6*cj6;
IkReal x1012=npy*npy;
IkReal x1013=npz*npz;
IkReal x1014=npx*npx;
IkReal x1015=(npy*sj6);
IkReal x1016=(cj6*npx);
IkReal x1017=((173.130193905817)*x1012);
IkReal x1018=(x1011*x1014);
j5eval[0]=((-1.0)+(((-26.3157894736842)*x1015))+(((26.3157894736842)*x1016))+(((-1.0)*x1017))+(((346.260387811634)*x1015*x1016))+(((-173.130193905817)*x1018))+(((-173.130193905817)*x1013))+((x1011*x1017)));
j5eval[1]=IKsign(((-0.005776)+(((-0.152)*x1015))+(((0.152)*x1016))+(((2.0)*x1015*x1016))+((x1011*x1012))+(((-1.0)*x1012))+(((-1.0)*x1013))+(((-1.0)*x1018))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1019=cj6*cj6;
IkReal x1020=npy*npy;
IkReal x1021=(cj6*npx);
IkReal x1022=(npy*sj6);
IkReal x1023=((0.0525)*sj3);
IkReal x1024=((0.293)*cj3);
CheckValue<IkReal> x1025=IKPowWithIntegerCheck(IKsign(((-0.005776)+(((-1.0)*x1020))+(((-1.0)*(npz*npz)))+(((-1.0)*x1019*(npx*npx)))+(((0.152)*x1021))+((x1019*x1020))+(((2.0)*x1021*x1022))+(((-0.152)*x1022)))),-1);
if(!x1025.valid){
continue;
}
CheckValue<IkReal> x1026 = IKatan2WithCheck(IkReal(((0.02603)+(((-1.0)*x1022*x1024))+(((0.3425)*x1022))+(((-0.3425)*x1021))+(((-1.0)*x1021*x1023))+((x1022*x1023))+((x1021*x1024))+(((0.00399)*sj3))+(((-0.022268)*cj3)))),IkReal((((npz*x1024))+(((-0.3425)*npz))+(((-1.0)*npz*x1023)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1026.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1025.value)))+(x1026.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[2];
IkReal x1027=IKcos(j5);
IkReal x1028=IKsin(j5);
IkReal x1029=(npy*sj6);
IkReal x1030=((1.0)*cj6*npx);
evalcond[0]=((((-1.0)*x1027*x1030))+(((0.076)*x1027))+((npz*x1028))+((x1027*x1029)));
evalcond[1]=((0.3425)+(((0.076)*x1028))+(((-1.0)*x1028*x1030))+((x1028*x1029))+(((-0.293)*cj3))+(((-1.0)*npz*x1027))+(((0.0525)*sj3)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j4))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[2];
sj4=0;
cj4=1.0;
j4=0;
IkReal x1031=cj6*cj6;
IkReal x1032=npy*npy;
IkReal x1033=npz*npz;
IkReal x1034=npx*npx;
IkReal x1035=(npy*sj6);
IkReal x1036=(cj6*npx);
IkReal x1037=((173.130193905817)*x1032);
IkReal x1038=(x1031*x1034);
j5eval[0]=((-1.0)+((x1031*x1037))+(((-1.0)*x1037))+(((26.3157894736842)*x1036))+(((-26.3157894736842)*x1035))+(((-173.130193905817)*x1033))+(((-173.130193905817)*x1038))+(((346.260387811634)*x1035*x1036)));
j5eval[1]=IKsign(((-0.005776)+((x1031*x1032))+(((-0.152)*x1035))+(((0.152)*x1036))+(((-1.0)*x1032))+(((-1.0)*x1033))+(((-1.0)*x1038))+(((2.0)*x1035*x1036))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1039=cj6*cj6;
IkReal x1040=npy*npy;
IkReal x1041=(npy*sj6);
IkReal x1042=((0.0525)*cj3);
IkReal x1043=(cj6*npx);
IkReal x1044=((0.293)*cj3);
IkReal x1045=((0.293)*sj3);
IkReal x1046=((0.0525)*sj3);
CheckValue<IkReal> x1047=IKPowWithIntegerCheck(IKsign(((-0.005776)+(((0.152)*x1043))+((x1039*x1040))+(((2.0)*x1041*x1043))+(((-1.0)*(npz*npz)))+(((-0.152)*x1041))+(((-1.0)*x1040))+(((-1.0)*x1039*(npx*npx))))),-1);
if(!x1047.valid){
continue;
}
CheckValue<IkReal> x1048 = IKatan2WithCheck(IkReal(((0.02603)+((x1043*x1044))+((x1041*x1046))+(((-1.0)*x1043*x1046))+(((-1.0)*x1041*x1044))+(((0.0775)*npz))+(((0.3425)*x1041))+(((0.00399)*sj3))+(((-0.3425)*x1043))+(((-0.022268)*cj3))+((npz*x1045))+((npz*x1042)))),IkReal(((0.00589)+(((0.00399)*cj3))+(((-1.0)*npz*x1046))+((x1041*x1042))+((x1041*x1045))+(((-1.0)*x1043*x1045))+(((-0.3425)*npz))+(((0.0775)*x1041))+(((-1.0)*x1042*x1043))+(((-0.0775)*x1043))+(((0.022268)*sj3))+((npz*x1044)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1048.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1047.value)))+(x1048.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[2];
IkReal x1049=IKcos(j5);
IkReal x1050=IKsin(j5);
IkReal x1051=(npy*sj6);
IkReal x1052=((1.0)*cj6*npx);
evalcond[0]=((0.0775)+(((0.0525)*cj3))+(((0.293)*sj3))+((x1049*x1051))+(((0.076)*x1049))+(((-1.0)*x1049*x1052))+((npz*x1050)));
evalcond[1]=((0.3425)+(((-1.0)*npz*x1049))+((x1050*x1051))+(((-0.293)*cj3))+(((0.076)*x1050))+(((0.0525)*sj3))+(((-1.0)*x1050*x1052)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j4)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j5eval[2];
sj4=0;
cj4=-1.0;
j4=3.14159265358979;
IkReal x1053=cj6*cj6;
IkReal x1054=npy*npy;
IkReal x1055=npz*npz;
IkReal x1056=npx*npx;
IkReal x1057=(npy*sj6);
IkReal x1058=(cj6*npx);
IkReal x1059=((173.130193905817)*x1054);
IkReal x1060=(x1053*x1056);
j5eval[0]=((-1.0)+(((-1.0)*x1059))+(((-173.130193905817)*x1055))+(((-173.130193905817)*x1060))+(((-26.3157894736842)*x1057))+((x1053*x1059))+(((346.260387811634)*x1057*x1058))+(((26.3157894736842)*x1058)));
j5eval[1]=IKsign(((-0.005776)+(((0.152)*x1058))+(((2.0)*x1057*x1058))+(((-0.152)*x1057))+((x1053*x1054))+(((-1.0)*x1054))+(((-1.0)*x1055))+(((-1.0)*x1060))));
if( IKabs(j5eval[0]) < 0.0000010000000000  || IKabs(j5eval[1]) < 0.0000010000000000  )
{
continue; // no branches [j5]

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1061=cj6*cj6;
IkReal x1062=npy*npy;
IkReal x1063=(npy*sj6);
IkReal x1064=((0.0525)*cj3);
IkReal x1065=(cj6*npx);
IkReal x1066=((0.293)*cj3);
IkReal x1067=((0.293)*sj3);
IkReal x1068=((0.0525)*sj3);
CheckValue<IkReal> x1069=IKPowWithIntegerCheck(IKsign(((-0.005776)+((x1061*x1062))+(((-0.152)*x1063))+(((-1.0)*(npz*npz)))+(((2.0)*x1063*x1065))+(((-1.0)*x1061*(npx*npx)))+(((0.152)*x1065))+(((-1.0)*x1062)))),-1);
if(!x1069.valid){
continue;
}
CheckValue<IkReal> x1070 = IKatan2WithCheck(IkReal(((0.02603)+(((-1.0)*x1063*x1066))+(((-0.0775)*npz))+(((0.00399)*sj3))+((x1063*x1068))+(((0.3425)*x1063))+(((-0.3425)*x1065))+(((-1.0)*x1065*x1068))+(((-1.0)*npz*x1067))+(((-1.0)*npz*x1064))+(((-0.022268)*cj3))+((x1065*x1066)))),IkReal(((-0.00589)+(((-1.0)*x1063*x1067))+(((-1.0)*x1063*x1064))+(((-0.0775)*x1063))+(((-0.00399)*cj3))+(((-0.3425)*npz))+(((-0.022268)*sj3))+(((0.0775)*x1065))+((npz*x1066))+((x1064*x1065))+(((-1.0)*npz*x1068))+((x1065*x1067)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1070.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1069.value)))+(x1070.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[2];
IkReal x1071=IKcos(j5);
IkReal x1072=IKsin(j5);
IkReal x1073=(npy*sj6);
IkReal x1074=((1.0)*cj6*npx);
evalcond[0]=((-0.0775)+(((-0.0525)*cj3))+(((-1.0)*x1071*x1074))+((x1071*x1073))+(((0.076)*x1071))+((npz*x1072))+(((-0.293)*sj3)));
evalcond[1]=((0.3425)+(((-1.0)*npz*x1071))+(((-0.293)*cj3))+(((0.076)*x1072))+((x1072*x1073))+(((-1.0)*x1072*x1074))+(((0.0525)*sj3)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j5]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1075=cj6*cj6;
IkReal x1076=npy*npy;
IkReal x1077=npx*npx;
IkReal x1078=(npy*sj6);
IkReal x1079=((0.152)*cj4);
IkReal x1080=(cj4*sj3);
IkReal x1081=((0.0525)*npz);
IkReal x1082=((0.0525)*cj3);
IkReal x1083=((0.3425)*cj4);
IkReal x1084=(cj6*npx);
IkReal x1085=(cj6*sj4);
IkReal x1086=(npx*sj4);
IkReal x1087=((0.293)*sj3);
IkReal x1088=((1.0)*cj4);
IkReal x1089=(cj4*x1076);
IkReal x1090=((0.293)*cj3*cj4);
CheckValue<IkReal> x1091 = IKatan2WithCheck(IkReal(((((-0.022268)*cj3*cj4))+((x1084*x1090))+((npz*x1087))+((x1078*x1083))+(((-0.0525)*x1080*x1084))+((npy*npz*x1085))+(((0.0775)*npz))+(((0.0525)*x1078*x1080))+(((0.00399)*x1080))+((cj3*x1081))+(((0.02603)*cj4))+((npz*sj6*x1086))+(((-1.0)*x1083*x1084))+(((-1.0)*x1078*x1090)))),IkReal(((0.00589)+(((0.00399)*cj3))+((npy*x1086))+(((0.076)*npy*x1085))+((x1078*x1082))+((x1078*x1087))+(((-1.0)*sj6*x1077*x1085))+((npz*x1090))+(((-1.0)*x1080*x1081))+(((-1.0)*npz*x1083))+(((-0.0775)*x1084))+(((-2.0)*npy*x1075*x1086))+(((-1.0)*x1082*x1084))+(((0.0775)*x1078))+(((-1.0)*x1084*x1087))+(((0.076)*sj6*x1086))+(((0.022268)*sj3))+((sj6*x1076*x1085)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1091.valid){
continue;
}
CheckValue<IkReal> x1092=IKPowWithIntegerCheck(IKsign((((x1079*x1084))+(((-1.0)*x1078*x1079))+((x1075*x1089))+(((-0.005776)*cj4))+(((2.0)*cj4*x1078*x1084))+(((-1.0)*x1076*x1088))+(((-1.0)*x1075*x1077*x1088))+(((-1.0)*x1088*(npz*npz))))),-1);
if(!x1092.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x1091.value)+(((1.5707963267949)*(x1092.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x1093=IKsin(j5);
IkReal x1094=IKcos(j5);
IkReal x1095=((0.293)*sj3);
IkReal x1096=(npx*sj4);
IkReal x1097=((0.0525)*cj3);
IkReal x1098=((1.0)*cj4);
IkReal x1099=(cj6*npy);
IkReal x1100=(cj6*npx);
IkReal x1101=(npz*x1093);
IkReal x1102=((1.0)*x1094);
IkReal x1103=((0.076)*x1094);
IkReal x1104=(npy*sj6*x1094);
evalcond[0]=((0.3425)+(((0.076)*x1093))+(((-1.0)*x1093*x1100))+(((-0.293)*cj3))+(((-1.0)*npz*x1102))+((npy*sj6*x1093))+(((0.0525)*sj3)));
evalcond[1]=(x1104+x1103+x1101+((cj4*x1095))+((cj4*x1097))+(((-1.0)*x1100*x1102))+(((0.0775)*cj4)));
evalcond[2]=((((-1.0)*npx*sj6*x1098))+((sj4*x1101))+((sj4*x1104))+((sj4*x1103))+(((-1.0)*cj6*x1096*x1102))+(((-1.0)*x1098*x1099)));
evalcond[3]=((0.0775)+x1095+x1097+((cj4*x1101))+((cj4*x1104))+((cj4*x1103))+((sj4*x1099))+(((-1.0)*x1094*x1098*x1100))+((sj6*x1096)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1105=npy*npy;
IkReal x1106=cj6*cj6;
IkReal x1107=npx*npx;
IkReal x1108=(cj3*sj4);
IkReal x1109=((1.0)*sj4);
IkReal x1110=(npy*sj6);
IkReal x1111=(cj6*npx);
IkReal x1112=((0.3425)*sj4);
IkReal x1113=((1.0)*npz);
IkReal x1114=((0.152)*sj4);
IkReal x1115=(cj4*npx);
IkReal x1116=(cj4*cj6*sj6);
IkReal x1117=((1.0)*x1105);
IkReal x1118=((0.0525)*sj3*sj4);
IkReal x1119=(cj4*cj6*npy);
CheckValue<IkReal> x1120 = IKatan2WithCheck(IkReal(((((-1.0)*sj6*x1113*x1115))+((x1110*x1112))+((x1110*x1118))+(((0.293)*x1108*x1111))+(((-0.293)*x1108*x1110))+(((-1.0)*x1113*x1119))+(((-0.022268)*x1108))+(((0.00399)*sj3*sj4))+(((0.02603)*sj4))+(((-1.0)*x1111*x1118))+(((-1.0)*x1111*x1112)))),IkReal(((((-0.076)*sj6*x1115))+(((-1.0)*npy*x1115))+(((-1.0)*x1116*x1117))+(((-1.0)*npz*x1112))+(((-1.0)*npz*x1118))+(((2.0)*npy*x1106*x1115))+(((-0.076)*x1119))+((x1107*x1116))+(((0.293)*npz*x1108)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1120.valid){
continue;
}
CheckValue<IkReal> x1121=IKPowWithIntegerCheck(IKsign((((x1111*x1114))+((sj4*x1105*x1106))+(((-0.005776)*sj4))+(((2.0)*sj4*x1110*x1111))+(((-1.0)*x1110*x1114))+(((-1.0)*x1109*(npz*npz)))+(((-1.0)*x1105*x1109))+(((-1.0)*x1106*x1107*x1109)))),-1);
if(!x1121.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(x1120.value)+(((1.5707963267949)*(x1121.value))));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x1122=IKsin(j5);
IkReal x1123=IKcos(j5);
IkReal x1124=((0.293)*sj3);
IkReal x1125=(npx*sj4);
IkReal x1126=((0.0525)*cj3);
IkReal x1127=((1.0)*cj4);
IkReal x1128=(cj6*npy);
IkReal x1129=(cj6*npx);
IkReal x1130=(npz*x1122);
IkReal x1131=((1.0)*x1123);
IkReal x1132=((0.076)*x1123);
IkReal x1133=(npy*sj6*x1123);
evalcond[0]=((0.3425)+(((-1.0)*npz*x1131))+(((0.076)*x1122))+((npy*sj6*x1122))+(((-0.293)*cj3))+(((-1.0)*x1122*x1129))+(((0.0525)*sj3)));
evalcond[1]=(x1133+x1132+x1130+((cj4*x1126))+((cj4*x1124))+(((-1.0)*x1129*x1131))+(((0.0775)*cj4)));
evalcond[2]=(((sj4*x1130))+((sj4*x1132))+((sj4*x1133))+(((-1.0)*x1127*x1128))+(((-1.0)*npx*sj6*x1127))+(((-1.0)*cj6*x1125*x1131)));
evalcond[3]=((0.0775)+x1124+x1126+((sj4*x1128))+((cj4*x1130))+((cj4*x1132))+((cj4*x1133))+(((-1.0)*x1123*x1127*x1129))+((sj6*x1125)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j5array[1], cj5array[1], sj5array[1];
bool j5valid[1]={false};
_nj5 = 1;
IkReal x1134=cj6*cj6;
IkReal x1135=npy*npy;
IkReal x1136=(cj4*sj3);
IkReal x1137=(cj3*cj4);
IkReal x1138=(npy*sj6);
IkReal x1139=(cj6*npx);
IkReal x1140=((0.293)*cj3);
IkReal x1141=((0.0775)*cj4);
IkReal x1142=((0.0525)*npz);
IkReal x1143=((0.0525)*sj3);
CheckValue<IkReal> x1144=IKPowWithIntegerCheck(IKsign(((-0.005776)+(((-1.0)*x1134*(npx*npx)))+(((-1.0)*(npz*npz)))+(((0.152)*x1139))+((x1134*x1135))+(((-0.152)*x1138))+(((-1.0)*x1135))+(((2.0)*x1138*x1139)))),-1);
if(!x1144.valid){
continue;
}
CheckValue<IkReal> x1145 = IKatan2WithCheck(IkReal(((0.02603)+((x1137*x1142))+((npz*x1141))+(((0.3425)*x1138))+(((0.00399)*sj3))+((x1139*x1140))+(((-1.0)*x1139*x1143))+(((0.293)*npz*x1136))+(((-0.3425)*x1139))+((x1138*x1143))+(((-0.022268)*cj3))+(((-1.0)*x1138*x1140)))),IkReal((((npz*x1140))+(((0.293)*x1136*x1138))+(((-0.3425)*npz))+(((-0.293)*x1136*x1139))+(((0.022268)*x1136))+(((0.0525)*x1137*x1138))+(((0.00589)*cj4))+(((0.00399)*x1137))+(((-1.0)*x1139*x1141))+(((-1.0)*sj3*x1142))+((x1138*x1141))+(((-0.0525)*x1137*x1139)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1145.valid){
continue;
}
j5array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1144.value)))+(x1145.value));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
for(int ij5 = 0; ij5 < 1; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 1; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];
{
IkReal evalcond[4];
IkReal x1146=IKsin(j5);
IkReal x1147=IKcos(j5);
IkReal x1148=((0.293)*sj3);
IkReal x1149=(npx*sj4);
IkReal x1150=((0.0525)*cj3);
IkReal x1151=((1.0)*cj4);
IkReal x1152=(cj6*npy);
IkReal x1153=(cj6*npx);
IkReal x1154=(npz*x1146);
IkReal x1155=((1.0)*x1147);
IkReal x1156=((0.076)*x1147);
IkReal x1157=(npy*sj6*x1147);
evalcond[0]=((0.3425)+(((0.076)*x1146))+(((-1.0)*npz*x1155))+(((-0.293)*cj3))+((npy*sj6*x1146))+(((0.0525)*sj3))+(((-1.0)*x1146*x1153)));
evalcond[1]=(x1154+x1157+x1156+((cj4*x1150))+((cj4*x1148))+(((-1.0)*x1153*x1155))+(((0.0775)*cj4)));
evalcond[2]=((((-1.0)*npx*sj6*x1151))+(((-1.0)*x1151*x1152))+((sj4*x1156))+((sj4*x1157))+((sj4*x1154))+(((-1.0)*cj6*x1149*x1155)));
evalcond[3]=((0.0775)+((sj6*x1149))+x1148+x1150+(((-1.0)*x1147*x1151*x1153))+((cj4*x1156))+((cj4*x1157))+((cj4*x1154))+((sj4*x1152)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}

} else
{
{
IkReal j5array[2], cj5array[2], sj5array[2];
bool j5valid[2]={false};
_nj5 = 2;
IkReal x1158=((0.076)+(((-1.0)*cj6*npx))+((npy*sj6)));
CheckValue<IkReal> x1161 = IKatan2WithCheck(IkReal(((-1.0)*npz)),IkReal(x1158),IKFAST_ATAN2_MAGTHRESH);
if(!x1161.valid){
continue;
}
IkReal x1159=((1.0)*(x1161.value));
if((((x1158*x1158)+(npz*npz))) < -0.00001)
continue;
CheckValue<IkReal> x1162=IKPowWithIntegerCheck(IKabs(IKsqrt(((x1158*x1158)+(npz*npz)))),-1);
if(!x1162.valid){
continue;
}
if( (((x1162.value)*(((0.3425)+(((-0.293)*cj3))+(((0.0525)*sj3)))))) < -1-IKFAST_SINCOS_THRESH || (((x1162.value)*(((0.3425)+(((-0.293)*cj3))+(((0.0525)*sj3)))))) > 1+IKFAST_SINCOS_THRESH )
    continue;
IkReal x1160=IKasin(((x1162.value)*(((0.3425)+(((-0.293)*cj3))+(((0.0525)*sj3))))));
j5array[0]=((((-1.0)*x1159))+(((-1.0)*x1160)));
sj5array[0]=IKsin(j5array[0]);
cj5array[0]=IKcos(j5array[0]);
j5array[1]=((3.14159265358979)+x1160+(((-1.0)*x1159)));
sj5array[1]=IKsin(j5array[1]);
cj5array[1]=IKcos(j5array[1]);
if( j5array[0] > IKPI )
{
    j5array[0]-=IK2PI;
}
else if( j5array[0] < -IKPI )
{    j5array[0]+=IK2PI;
}
j5valid[0] = true;
if( j5array[1] > IKPI )
{
    j5array[1]-=IK2PI;
}
else if( j5array[1] < -IKPI )
{    j5array[1]+=IK2PI;
}
j5valid[1] = true;
for(int ij5 = 0; ij5 < 2; ++ij5)
{
if( !j5valid[ij5] )
{
    continue;
}
_ij5[0] = ij5; _ij5[1] = -1;
for(int iij5 = ij5+1; iij5 < 2; ++iij5)
{
if( j5valid[iij5] && IKabs(cj5array[ij5]-cj5array[iij5]) < IKFAST_SOLUTION_THRESH && IKabs(sj5array[ij5]-sj5array[iij5]) < IKFAST_SOLUTION_THRESH )
{
    j5valid[iij5]=false; _ij5[1] = iij5; break; 
}
}
j5 = j5array[ij5]; cj5 = cj5array[ij5]; sj5 = sj5array[ij5];

{
IkReal j4eval[3];
IkReal x1163=((1.0)*npy);
j4eval[0]=((1.47619047619048)+cj3+(((5.58095238095238)*sj3)));
j4eval[1]=((IKabs(((((-1.0)*cj6*x1163))+(((-1.0)*npx*sj6)))))+(IKabs(((((-0.076)*cj5))+(((-1.0)*cj5*sj6*x1163))+((cj5*cj6*npx))+(((-1.0)*npz*sj5))))));
j4eval[2]=IKsign(((0.0775)+(((0.0525)*cj3))+(((0.293)*sj3))));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  || IKabs(j4eval[2]) < 0.0000010000000000  )
{
{
IkReal j4eval[2];
IkReal x1164=(npx*sj6);
IkReal x1165=((5.58095238095238)*sj3);
IkReal x1166=(cj6*npy);
IkReal x1167=((1.0)*cj3);
j4eval[0]=((1.47619047619048)+x1165+cj3);
j4eval[1]=((((-1.0)*x1166*x1167))+(((-1.47619047619048)*x1166))+(((-1.47619047619048)*x1164))+(((-1.0)*x1164*x1165))+(((-1.0)*x1164*x1167))+(((-1.0)*x1165*x1166)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  )
{
{
IkReal j4eval[2];
IkReal x1168=((73.4335839598997)*sj3);
IkReal x1169=((13.1578947368421)*cj3);
IkReal x1170=((5.58095238095238)*sj3);
IkReal x1171=(npz*sj5);
IkReal x1172=(cj5*cj6*npx);
IkReal x1173=(cj5*npy*sj6);
j4eval[0]=((1.47619047619048)+x1170+cj3);
j4eval[1]=((((-1.0)*x1168*x1172))+((x1168*x1171))+((x1168*x1173))+((cj3*cj5))+((cj5*x1170))+(((-19.4235588972431)*x1172))+(((-1.0)*x1169*x1172))+(((19.4235588972431)*x1173))+(((19.4235588972431)*x1171))+((x1169*x1173))+((x1169*x1171))+(((1.47619047619048)*cj5)));
if( IKabs(j4eval[0]) < 0.0000010000000000  || IKabs(j4eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((0.440693041647182)+j3)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1174=((51305507480.2948)*cj5);
if( IKabs(((((-51305507480.2948)*npx*sj6))+(((-51305507480.2948)*cj6*npy)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-51305507480.2948)*npz*sj5))+(((-1.0)*npy*sj6*x1174))+((cj6*npx*x1174))+(((-3899218568.5024)*cj5)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-51305507480.2948)*npx*sj6))+(((-51305507480.2948)*cj6*npy))))+IKsqr(((((-51305507480.2948)*npz*sj5))+(((-1.0)*npy*sj6*x1174))+((cj6*npx*x1174))+(((-3899218568.5024)*cj5))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j4array[0]=IKatan2(((((-51305507480.2948)*npx*sj6))+(((-51305507480.2948)*cj6*npy))), ((((-51305507480.2948)*npz*sj5))+(((-1.0)*npy*sj6*x1174))+((cj6*npx*x1174))+(((-3899218568.5024)*cj5))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1175=IKsin(j4);
IkReal x1176=IKcos(j4);
IkReal x1177=(npy*sj6);
IkReal x1178=(npz*sj5);
IkReal x1179=((0.076)*cj5);
IkReal x1180=(cj6*npy);
IkReal x1181=(npx*sj6);
IkReal x1182=(cj5*x1176);
IkReal x1183=((1.0)*x1176);
IkReal x1184=((1.0)*cj5*cj6*npx);
evalcond[0]=(x1180+x1181+(((1.94910848583668e-11)*x1175)));
evalcond[1]=(x1179+x1178+(((1.94910848583668e-11)*x1176))+((cj5*x1177))+(((-1.0)*x1184)));
evalcond[2]=((1.94910848583668e-11)+(((-1.0)*cj6*npx*x1182))+((x1175*x1180))+((x1175*x1181))+((x1177*x1182))+((x1176*x1178))+((x1176*x1179)));
evalcond[3]=((((-1.0)*x1181*x1183))+(((-1.0)*x1175*x1184))+((x1175*x1178))+((x1175*x1179))+(((-1.0)*x1180*x1183))+((cj5*x1175*x1177)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.2276868576215)+j3)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1185=((819672131.147541)*sj6);
IkReal x1186=((819672131.147541)*cj6);
if( IKabs((((npy*x1186))+((npx*x1185)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((((cj5*npy*x1185))+(((-1.0)*cj5*npx*x1186))+(((62295081.9672131)*cj5))+(((819672131.147541)*npz*sj5)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((npy*x1186))+((npx*x1185))))+IKsqr((((cj5*npy*x1185))+(((-1.0)*cj5*npx*x1186))+(((62295081.9672131)*cj5))+(((819672131.147541)*npz*sj5))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j4array[0]=IKatan2((((npy*x1186))+((npx*x1185))), (((cj5*npy*x1185))+(((-1.0)*cj5*npx*x1186))+(((62295081.9672131)*cj5))+(((819672131.147541)*npz*sj5))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1187=IKsin(j4);
IkReal x1188=IKcos(j4);
IkReal x1189=((1.0)*cj6);
IkReal x1190=(cj5*sj6);
IkReal x1191=(npz*sj5);
IkReal x1192=(cj6*npy);
IkReal x1193=(npx*sj6);
IkReal x1194=(cj5*x1187);
IkReal x1195=(npy*x1188);
IkReal x1196=(cj5*x1188);
evalcond[0]=(x1193+x1192+(((-1.22e-9)*x1187)));
evalcond[1]=(x1191+(((-1.22e-9)*x1188))+(((-1.0)*cj5*npx*x1189))+((npy*x1190))+(((0.076)*cj5)));
evalcond[2]=((-1.22e-9)+((x1187*x1193))+((x1187*x1192))+(((0.076)*x1196))+((x1188*x1191))+(((-1.0)*npx*x1189*x1196))+((x1190*x1195)));
evalcond[3]=((((-1.0)*x1189*x1195))+((x1187*x1191))+(((0.076)*x1194))+(((-1.0)*npx*x1189*x1194))+(((-1.0)*x1188*x1193))+((npy*x1187*x1190)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j4]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1197=cj6*cj6;
IkReal x1198=npx*npx;
IkReal x1199=((0.0525)*cj3);
IkReal x1200=(npz*sj5);
IkReal x1201=(npx*sj6);
IkReal x1202=(cj6*npy);
IkReal x1203=((0.293)*sj3);
IkReal x1204=(cj5*npy*sj6);
IkReal x1205=(cj5*cj6*npx);
CheckValue<IkReal> x1206=IKPowWithIntegerCheck(((0.0775)+x1199+x1203),-1);
if(!x1206.valid){
continue;
}
CheckValue<IkReal> x1207=IKPowWithIntegerCheck(((((-1.0)*x1199*x1205))+(((0.0775)*x1204))+(((0.0775)*x1200))+(((-0.0775)*x1205))+(((0.00399)*cj3*cj5))+((x1199*x1204))+((x1199*x1200))+(((0.022268)*cj5*sj3))+((x1200*x1203))+((x1203*x1204))+(((-1.0)*x1203*x1205))+(((0.00589)*cj5))),-1);
if(!x1207.valid){
continue;
}
if( IKabs(((x1206.value)*(((((-1.0)*x1202))+(((-1.0)*x1201)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((x1207.value)*(((-0.09185525)+(((-0.045415)*sj3))+x1198+(((-0.0081375)*cj3))+(((-1.0)*x1197*x1198))+(((2.0)*x1201*x1202))+(((-0.030765)*cj3*sj3))+((x1197*(npy*npy)))+(((0.08309275)*(cj3*cj3))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x1206.value)*(((((-1.0)*x1202))+(((-1.0)*x1201))))))+IKsqr(((x1207.value)*(((-0.09185525)+(((-0.045415)*sj3))+x1198+(((-0.0081375)*cj3))+(((-1.0)*x1197*x1198))+(((2.0)*x1201*x1202))+(((-0.030765)*cj3*sj3))+((x1197*(npy*npy)))+(((0.08309275)*(cj3*cj3)))))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j4array[0]=IKatan2(((x1206.value)*(((((-1.0)*x1202))+(((-1.0)*x1201))))), ((x1207.value)*(((-0.09185525)+(((-0.045415)*sj3))+x1198+(((-0.0081375)*cj3))+(((-1.0)*x1197*x1198))+(((2.0)*x1201*x1202))+(((-0.030765)*cj3*sj3))+((x1197*(npy*npy)))+(((0.08309275)*(cj3*cj3)))))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1208=IKsin(j4);
IkReal x1209=IKcos(j4);
IkReal x1210=((0.293)*sj3);
IkReal x1211=(npx*sj6);
IkReal x1212=((0.0525)*cj3);
IkReal x1213=(cj5*sj6);
IkReal x1214=(npz*sj5);
IkReal x1215=(cj6*npy);
IkReal x1216=((0.076)*cj5);
IkReal x1217=((1.0)*x1209);
IkReal x1218=(npy*x1208);
IkReal x1219=(cj5*x1209);
IkReal x1220=(cj5*cj6*npx);
evalcond[0]=((((0.0775)*x1208))+x1211+x1215+((x1208*x1212))+((x1208*x1210)));
evalcond[1]=((((0.0775)*x1209))+((npy*x1213))+x1216+x1214+(((-1.0)*x1220))+((x1209*x1210))+((x1209*x1212)));
evalcond[2]=((((-1.0)*x1215*x1217))+((x1213*x1218))+(((-1.0)*x1211*x1217))+(((-1.0)*x1208*x1220))+((x1208*x1216))+((x1208*x1214)));
evalcond[3]=((0.0775)+x1212+x1210+((npy*x1209*x1213))+(((-1.0)*x1217*x1220))+((x1209*x1214))+((x1209*x1216))+((x1208*x1215))+((x1208*x1211)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1221=(npx*sj6);
IkReal x1222=((0.293)*sj3);
IkReal x1223=(cj6*npy);
IkReal x1224=((0.076)*cj5);
IkReal x1225=(npz*sj5);
IkReal x1226=((0.0525)*cj3);
IkReal x1227=(cj5*npx*npy);
IkReal x1228=(cj5*cj6*sj6);
CheckValue<IkReal> x1229=IKPowWithIntegerCheck(((0.0775)+x1226+x1222),-1);
if(!x1229.valid){
continue;
}
CheckValue<IkReal> x1230=IKPowWithIntegerCheck(((((-1.0)*x1221*x1226))+(((-1.0)*x1221*x1222))+(((-1.0)*x1222*x1223))+(((-0.0775)*x1221))+(((-0.0775)*x1223))+(((-1.0)*x1223*x1226))),-1);
if(!x1230.valid){
continue;
}
if( IKabs(((x1229.value)*(((((-1.0)*x1221))+(((-1.0)*x1223)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((x1230.value)*((x1227+((x1223*x1225))+((x1223*x1224))+(((-2.0)*cj5*cj6*npx*x1223))+((x1221*x1225))+((x1221*x1224))+(((-1.0)*cj5*cj6*npx*x1221))+((cj5*npy*sj6*x1223)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x1229.value)*(((((-1.0)*x1221))+(((-1.0)*x1223))))))+IKsqr(((x1230.value)*((x1227+((x1223*x1225))+((x1223*x1224))+(((-2.0)*cj5*cj6*npx*x1223))+((x1221*x1225))+((x1221*x1224))+(((-1.0)*cj5*cj6*npx*x1221))+((cj5*npy*sj6*x1223))))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j4array[0]=IKatan2(((x1229.value)*(((((-1.0)*x1221))+(((-1.0)*x1223))))), ((x1230.value)*((x1227+((x1223*x1225))+((x1223*x1224))+(((-2.0)*cj5*cj6*npx*x1223))+((x1221*x1225))+((x1221*x1224))+(((-1.0)*cj5*cj6*npx*x1221))+((cj5*npy*sj6*x1223))))));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1231=IKsin(j4);
IkReal x1232=IKcos(j4);
IkReal x1233=((0.293)*sj3);
IkReal x1234=(npx*sj6);
IkReal x1235=((0.0525)*cj3);
IkReal x1236=(cj5*sj6);
IkReal x1237=(npz*sj5);
IkReal x1238=(cj6*npy);
IkReal x1239=((0.076)*cj5);
IkReal x1240=((1.0)*x1232);
IkReal x1241=(npy*x1231);
IkReal x1242=(cj5*x1232);
IkReal x1243=(cj5*cj6*npx);
evalcond[0]=(x1234+x1238+(((0.0775)*x1231))+((x1231*x1235))+((x1231*x1233)));
evalcond[1]=(x1237+x1239+((npy*x1236))+(((-1.0)*x1243))+(((0.0775)*x1232))+((x1232*x1233))+((x1232*x1235)));
evalcond[2]=(((x1236*x1241))+(((-1.0)*x1238*x1240))+((x1231*x1239))+((x1231*x1237))+(((-1.0)*x1234*x1240))+(((-1.0)*x1231*x1243)));
evalcond[3]=((0.0775)+x1235+x1233+((npy*x1232*x1236))+(((-1.0)*x1240*x1243))+((x1231*x1238))+((x1231*x1234))+((x1232*x1237))+((x1232*x1239)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}

} else
{
{
IkReal j4array[1], cj4array[1], sj4array[1];
bool j4valid[1]={false};
_nj4 = 1;
IkReal x1244=((1.0)*npy);
CheckValue<IkReal> x1245=IKPowWithIntegerCheck(IKsign(((0.0775)+(((0.0525)*cj3))+(((0.293)*sj3)))),-1);
if(!x1245.valid){
continue;
}
CheckValue<IkReal> x1246 = IKatan2WithCheck(IkReal(((((-1.0)*npx*sj6))+(((-1.0)*cj6*x1244)))),IkReal(((((-1.0)*cj5*sj6*x1244))+(((-0.076)*cj5))+((cj5*cj6*npx))+(((-1.0)*npz*sj5)))),IKFAST_ATAN2_MAGTHRESH);
if(!x1246.valid){
continue;
}
j4array[0]=((-1.5707963267949)+(((1.5707963267949)*(x1245.value)))+(x1246.value));
sj4array[0]=IKsin(j4array[0]);
cj4array[0]=IKcos(j4array[0]);
if( j4array[0] > IKPI )
{
    j4array[0]-=IK2PI;
}
else if( j4array[0] < -IKPI )
{    j4array[0]+=IK2PI;
}
j4valid[0] = true;
for(int ij4 = 0; ij4 < 1; ++ij4)
{
if( !j4valid[ij4] )
{
    continue;
}
_ij4[0] = ij4; _ij4[1] = -1;
for(int iij4 = ij4+1; iij4 < 1; ++iij4)
{
if( j4valid[iij4] && IKabs(cj4array[ij4]-cj4array[iij4]) < IKFAST_SOLUTION_THRESH && IKabs(sj4array[ij4]-sj4array[iij4]) < IKFAST_SOLUTION_THRESH )
{
    j4valid[iij4]=false; _ij4[1] = iij4; break; 
}
}
j4 = j4array[ij4]; cj4 = cj4array[ij4]; sj4 = sj4array[ij4];
{
IkReal evalcond[4];
IkReal x1247=IKsin(j4);
IkReal x1248=IKcos(j4);
IkReal x1249=((0.293)*sj3);
IkReal x1250=(npx*sj6);
IkReal x1251=((0.0525)*cj3);
IkReal x1252=(cj5*sj6);
IkReal x1253=(npz*sj5);
IkReal x1254=(cj6*npy);
IkReal x1255=((0.076)*cj5);
IkReal x1256=((1.0)*x1248);
IkReal x1257=(npy*x1247);
IkReal x1258=(cj5*x1248);
IkReal x1259=(cj5*cj6*npx);
evalcond[0]=(((x1247*x1251))+x1254+x1250+((x1247*x1249))+(((0.0775)*x1247)));
evalcond[1]=(x1255+x1253+((x1248*x1249))+((npy*x1252))+(((0.0775)*x1248))+(((-1.0)*x1259))+((x1248*x1251)));
evalcond[2]=(((x1247*x1253))+((x1247*x1255))+(((-1.0)*x1247*x1259))+((x1252*x1257))+(((-1.0)*x1254*x1256))+(((-1.0)*x1250*x1256)));
evalcond[3]=((0.0775)+((x1247*x1254))+((x1247*x1250))+x1249+x1251+((npy*x1248*x1252))+(((-1.0)*x1256*x1259))+((x1248*x1255))+((x1248*x1253)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

rotationfunction0(solutions);
}
}

}

}
}
}

}

}
}
}
}
return solutions.GetNumSolutions()>0;
}
inline void rotationfunction0(IkSolutionListBase<IkReal>& solutions) {
for(int rotationiter = 0; rotationiter < 1; ++rotationiter) {
IkReal x113=((1.0)*sj4);
IkReal x114=((1.0)*sj5);
IkReal x115=((1.0)*sj3);
IkReal x116=((1.0)*cj5);
IkReal x117=((1.0)*cj4);
IkReal x118=((1.0)*sj6);
IkReal x119=((-1.0)*sj4);
IkReal x120=((-1.0)*sj5);
IkReal x121=((-1.0)*cj5);
IkReal x122=((((-1.0)*r01*x118))+((cj6*r00)));
IkReal x123=(((r00*sj6))+((cj6*r01)));
IkReal x124=((((-1.0)*r11*x118))+((cj6*r10)));
IkReal x125=(((r10*sj6))+((cj6*r11)));
IkReal x126=((((-1.0)*r21*x118))+((cj6*r20)));
IkReal x127=(((cj6*r21))+((r20*sj6)));
IkReal x128=(cj5*x122);
IkReal x129=(cj5*x124);
IkReal x130=(cj5*x126);
IkReal x131=(x128+(((-1.0)*r02*x114)));
IkReal x132=((((-1.0)*r12*x114))+x129);
IkReal x133=(x130+(((-1.0)*r22*x114)));
new_r00=(((cj3*(((((-1.0)*x113*x123))+((cj4*x131))))))+(((-1.0)*x115*((((r02*x121))+((x120*x122)))))));
new_r01=((((-1.0)*x113*x131))+(((-1.0)*x117*x123)));
new_r02=(((cj3*(((((-1.0)*x114*x122))+(((-1.0)*r02*x116))))))+((sj3*((((x119*x123))+((cj4*((((r02*x120))+x128)))))))));
new_r10=(((cj3*(((((-1.0)*x113*x125))+((cj4*x132))))))+(((-1.0)*x115*((((r12*x121))+((x120*x124)))))));
new_r11=((((-1.0)*x113*x132))+(((-1.0)*x117*x125)));
new_r12=(((sj3*((((cj4*((x129+((r12*x120))))))+((x119*x125))))))+((cj3*(((((-1.0)*r12*x116))+(((-1.0)*x114*x124)))))));
new_r20=(((cj3*((((cj4*((((r22*x120))+x130))))+((x119*x127))))))+(((-1.0)*x115*(((((-1.0)*r22*x116))+(((-1.0)*x114*x126)))))));
new_r21=((((-1.0)*x113*x133))+(((-1.0)*x117*x127)));
new_r22=(((sj3*(((((-1.0)*x113*x127))+((cj4*x133))))))+((cj3*((((r22*x121))+((x120*x126)))))));
{
IkReal j1array[2], cj1array[2], sj1array[2];
bool j1valid[2]={false};
_nj1 = 2;
cj1array[0]=new_r22;
if( cj1array[0] >= -1-IKFAST_SINCOS_THRESH && cj1array[0] <= 1+IKFAST_SINCOS_THRESH )
{
    j1valid[0] = j1valid[1] = true;
    j1array[0] = IKacos(cj1array[0]);
    sj1array[0] = IKsin(j1array[0]);
    cj1array[1] = cj1array[0];
    j1array[1] = -j1array[0];
    sj1array[1] = -sj1array[0];
}
else if( isnan(cj1array[0]) )
{
    // probably any value will work
    j1valid[0] = true;
    cj1array[0] = 1; sj1array[0] = 0; j1array[0] = 0;
}
for(int ij1 = 0; ij1 < 2; ++ij1)
{
if( !j1valid[ij1] )
{
    continue;
}
_ij1[0] = ij1; _ij1[1] = -1;
for(int iij1 = ij1+1; iij1 < 2; ++iij1)
{
if( j1valid[iij1] && IKabs(cj1array[ij1]-cj1array[iij1]) < IKFAST_SOLUTION_THRESH && IKabs(sj1array[ij1]-sj1array[iij1]) < IKFAST_SOLUTION_THRESH )
{
    j1valid[iij1]=false; _ij1[1] = iij1; break; 
}
}
j1 = j1array[ij1]; cj1 = cj1array[ij1]; sj1 = sj1array[ij1];

{
IkReal j0eval[3];
j0eval[0]=sj1;
j0eval[1]=((IKabs(new_r12))+(IKabs(new_r02)));
j0eval[2]=IKsign(sj1);
if( IKabs(j0eval[0]) < 0.0000010000000000  || IKabs(j0eval[1]) < 0.0000010000000000  || IKabs(j0eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[3];
j2eval[0]=sj1;
j2eval[1]=IKsign(sj1);
j2eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j0eval[2];
j0eval[0]=new_r12;
j0eval[1]=sj1;
if( IKabs(j0eval[0]) < 0.0000010000000000  || IKabs(j0eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[5];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j1))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
IkReal j2mul = 1;
j2=0;
j0mul=-1.0;
if( IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r01))+IKsqr(new_r00)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j0=IKatan2(((-1.0)*new_r01), new_r00);
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].fmul = j0mul;
vinfos[0].freeind = 0;
vinfos[0].maxsolutions = 0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].fmul = j2mul;
vinfos[2].freeind = 0;
vinfos[2].maxsolutions = 0;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(1);
vfree[0] = 2;
solutions.AddSolution(vinfos,vfree);
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j1)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
IkReal j2mul = 1;
j2=0;
j0mul=1.0;
if( IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r01))+IKsqr(((-1.0)*new_r00))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j0=IKatan2(((-1.0)*new_r01), ((-1.0)*new_r00));
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].fmul = j0mul;
vinfos[0].freeind = 0;
vinfos[0].maxsolutions = 0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].fmul = j2mul;
vinfos[2].freeind = 0;
vinfos[2].maxsolutions = 0;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(1);
vfree[0] = 2;
solutions.AddSolution(vinfos,vfree);
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r12))+(IKabs(new_r02)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j0eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
IkReal x134=new_r22*new_r22;
IkReal x135=((16.0)*new_r10);
IkReal x136=((16.0)*new_r01);
IkReal x137=((16.0)*new_r22);
IkReal x138=((8.0)*new_r11);
IkReal x139=((8.0)*new_r00);
IkReal x140=(x134*x135);
IkReal x141=(x134*x136);
j0eval[0]=((IKabs((x140+(((-1.0)*x135)))))+(IKabs((((new_r22*x138))+(((-1.0)*x139)))))+(IKabs(((((16.0)*new_r00))+(((-32.0)*new_r00*x134))+((new_r11*x137)))))+(IKabs((x141+(((-1.0)*x136)))))+(IKabs(((((-1.0)*x140))+x135)))+(IKabs(((((32.0)*new_r11))+(((-1.0)*new_r00*x137))+(((-16.0)*new_r11*x134)))))+(IKabs(((((-1.0)*x141))+x136)))+(IKabs((((x134*x138))+(((-1.0)*new_r22*x139))))));
if( IKabs(j0eval[0]) < 0.0000000100000000  )
{
continue; // no branches [j0, j2]

} else
{
IkReal op[4+1], zeror[4];
int numroots;
IkReal j0evalpoly[1];
IkReal x142=new_r22*new_r22;
IkReal x143=((16.0)*new_r10);
IkReal x144=(new_r11*new_r22);
IkReal x145=(x142*x143);
IkReal x146=((((8.0)*x144))+(((-8.0)*new_r00)));
op[0]=x146;
op[1]=((((-1.0)*x145))+x143);
op[2]=((((16.0)*new_r00))+(((16.0)*x144))+(((-32.0)*new_r00*x142)));
op[3]=((((-1.0)*x143))+x145);
op[4]=x146;
polyroots4(op,zeror,numroots);
IkReal j0array[4], cj0array[4], sj0array[4], tempj0array[1];
int numsolutions = 0;
for(int ij0 = 0; ij0 < numroots; ++ij0)
{
IkReal htj0 = zeror[ij0];
tempj0array[0]=((2.0)*(atan(htj0)));
for(int kj0 = 0; kj0 < 1; ++kj0)
{
j0array[numsolutions] = tempj0array[kj0];
if( j0array[numsolutions] > IKPI )
{
    j0array[numsolutions]-=IK2PI;
}
else if( j0array[numsolutions] < -IKPI )
{
    j0array[numsolutions]+=IK2PI;
}
sj0array[numsolutions] = IKsin(j0array[numsolutions]);
cj0array[numsolutions] = IKcos(j0array[numsolutions]);
numsolutions++;
}
}
bool j0valid[4]={true,true,true,true};
_nj0 = 4;
for(int ij0 = 0; ij0 < numsolutions; ++ij0)
    {
if( !j0valid[ij0] )
{
    continue;
}
    j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
htj0 = IKtan(j0/2);

IkReal x147=((16.0)*new_r01);
IkReal x148=new_r22*new_r22;
IkReal x149=(new_r00*new_r22);
IkReal x150=((8.0)*x149);
IkReal x151=(new_r11*x148);
IkReal x152=(x147*x148);
IkReal x153=((8.0)*x151);
j0evalpoly[0]=((((-1.0)*x150))+((htj0*(((((-1.0)*x152))+x147))))+(((htj0*htj0*htj0)*(((((-1.0)*x147))+x152))))+x153+(((htj0*htj0*htj0*htj0)*(((((-1.0)*x150))+x153))))+(((htj0*htj0)*(((((32.0)*new_r11))+(((-16.0)*x151))+(((-16.0)*x149)))))));
if( IKabs(j0evalpoly[0]) > 0.0000001000000000  )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < numsolutions; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
{
IkReal j2eval[3];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
IkReal x154=cj0*cj0;
IkReal x155=(cj0*new_r22);
IkReal x156=((-1.0)+x154+(((-1.0)*x154*(new_r22*new_r22))));
j2eval[0]=x156;
j2eval[1]=IKsign(x156);
j2eval[2]=((IKabs((((new_r01*x155))+((new_r00*sj0)))))+(IKabs((((new_r01*sj0))+(((-1.0)*new_r00*x155))))));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
j2eval[0]=new_r22;
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal j2eval[2];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
IkReal x157=new_r22*new_r22;
j2eval[0]=(((cj0*x157))+(((-1.0)*cj0)));
j2eval[1]=((((-1.0)*sj0))+((sj0*x157)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[1];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j0)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r01))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r01));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x158=IKsin(j2);
IkReal x159=IKcos(j2);
evalcond[0]=x158;
evalcond[1]=((-1.0)*x159);
evalcond[2]=((((-1.0)*x158))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*x159))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j0)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r00)+IKsqr(new_r01)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r00, new_r01);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x160=IKsin(j2);
IkReal x161=IKcos(j2);
evalcond[0]=x160;
evalcond[1]=((-1.0)*x161);
evalcond[2]=((((-1.0)*x160))+new_r00);
evalcond[3]=((((-1.0)*x161))+new_r01);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j0))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r10)+IKsqr(new_r11)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r10, new_r11);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x162=IKsin(j2);
IkReal x163=IKcos(j2);
evalcond[0]=x162;
evalcond[1]=((-1.0)*x163);
evalcond[2]=((((-1.0)*x162))+new_r10);
evalcond[3]=((((-1.0)*x163))+new_r11);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j0)))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r10), ((-1.0)*new_r11));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x164=IKsin(j2);
IkReal x165=IKcos(j2);
evalcond[0]=x164;
evalcond[1]=((-1.0)*x165);
evalcond[2]=((((-1.0)*x164))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x165))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
CheckValue<IkReal> x166=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x166.valid){
continue;
}
if((x166.value) < -0.00001)
continue;
IkReal gconst0=((-1.0)*(IKsqrt(x166.value)));
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs((cj0+(((-1.0)*gconst0)))))+(IKabs(((-1.0)+(IKsign(sj0)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
if((((1.0)+(((-1.0)*(gconst0*gconst0))))) < -0.00001)
continue;
sj0=IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0)))));
cj0=gconst0;
if( (gconst0) < -1-IKFAST_SINCOS_THRESH || (gconst0) > 1+IKFAST_SINCOS_THRESH )
    continue;
j0=IKacos(gconst0);
CheckValue<IkReal> x167=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x167.valid){
continue;
}
if((x167.value) < -0.00001)
continue;
IkReal gconst0=((-1.0)*(IKsqrt(x167.value)));
j2eval[0]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if((((1.0)+(((-1.0)*(gconst0*gconst0))))) < -0.00001)
continue;
CheckValue<IkReal> x168=IKPowWithIntegerCheck(gconst0,-1);
if(!x168.valid){
continue;
}
if( IKabs(((((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0))))))))+((gconst0*new_r10)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r11*(x168.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0))))))))+((gconst0*new_r10))))+IKsqr((new_r11*(x168.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0))))))))+((gconst0*new_r10))), (new_r11*(x168.value)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x169=IKcos(j2);
IkReal x170=IKsin(j2);
IkReal x171=((1.0)*gconst0);
if((((1.0)+(((-1.0)*gconst0*x171)))) < -0.00001)
continue;
IkReal x172=IKsqrt(((1.0)+(((-1.0)*gconst0*x171))));
IkReal x173=((1.0)*x172);
evalcond[0]=x170;
evalcond[1]=((-1.0)*x169);
evalcond[2]=(new_r11+(((-1.0)*x169*x171)));
evalcond[3]=((((-1.0)*x170*x171))+new_r10);
evalcond[4]=(((x169*x172))+new_r01);
evalcond[5]=(((x170*x172))+new_r00);
evalcond[6]=((((-1.0)*x170))+((gconst0*new_r10))+(((-1.0)*new_r00*x173)));
evalcond[7]=((((-1.0)*x169))+((gconst0*new_r11))+(((-1.0)*new_r01*x173)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x174=IKPowWithIntegerCheck(IKsign(gconst0),-1);
if(!x174.valid){
continue;
}
CheckValue<IkReal> x175 = IKatan2WithCheck(IkReal(new_r10),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x175.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x174.value)))+(x175.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x176=IKcos(j2);
IkReal x177=IKsin(j2);
IkReal x178=((1.0)*gconst0);
if((((1.0)+(((-1.0)*gconst0*x178)))) < -0.00001)
continue;
IkReal x179=IKsqrt(((1.0)+(((-1.0)*gconst0*x178))));
IkReal x180=((1.0)*x179);
evalcond[0]=x177;
evalcond[1]=((-1.0)*x176);
evalcond[2]=((((-1.0)*x176*x178))+new_r11);
evalcond[3]=((((-1.0)*x177*x178))+new_r10);
evalcond[4]=(((x176*x179))+new_r01);
evalcond[5]=(new_r00+((x177*x179)));
evalcond[6]=((((-1.0)*x177))+((gconst0*new_r10))+(((-1.0)*new_r00*x180)));
evalcond[7]=((((-1.0)*x176))+(((-1.0)*new_r01*x180))+((gconst0*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
CheckValue<IkReal> x181=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x181.valid){
continue;
}
if((x181.value) < -0.00001)
continue;
IkReal gconst0=((-1.0)*(IKsqrt(x181.value)));
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.0)+(IKsign(sj0)))))+(IKabs((cj0+(((-1.0)*gconst0)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
if((((1.0)+(((-1.0)*(gconst0*gconst0))))) < -0.00001)
continue;
sj0=((-1.0)*(IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0)))))));
cj0=gconst0;
if( (gconst0) < -1-IKFAST_SINCOS_THRESH || (gconst0) > 1+IKFAST_SINCOS_THRESH )
    continue;
j0=((-1.0)*(IKacos(gconst0)));
CheckValue<IkReal> x182=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x182.valid){
continue;
}
if((x182.value) < -0.00001)
continue;
IkReal gconst0=((-1.0)*(IKsqrt(x182.value)));
j2eval[0]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if((((1.0)+(((-1.0)*(gconst0*gconst0))))) < -0.00001)
continue;
CheckValue<IkReal> x183=IKPowWithIntegerCheck(gconst0,-1);
if(!x183.valid){
continue;
}
if( IKabs((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0))))))))+((gconst0*new_r10)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r11*(x183.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0))))))))+((gconst0*new_r10))))+IKsqr((new_r11*(x183.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0))))))))+((gconst0*new_r10))), (new_r11*(x183.value)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x184=IKcos(j2);
IkReal x185=IKsin(j2);
IkReal x186=((1.0)*x185);
IkReal x187=((1.0)*x184);
if((((1.0)+(((-1.0)*(gconst0*gconst0))))) < -0.00001)
continue;
IkReal x188=IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0)))));
evalcond[0]=x185;
evalcond[1]=((-1.0)*x184);
evalcond[2]=(new_r11+(((-1.0)*gconst0*x187)));
evalcond[3]=(new_r10+(((-1.0)*gconst0*x186)));
evalcond[4]=((((-1.0)*x187*x188))+new_r01);
evalcond[5]=((((-1.0)*x186*x188))+new_r00);
evalcond[6]=(((new_r00*x188))+(((-1.0)*x186))+((gconst0*new_r10)));
evalcond[7]=(((new_r01*x188))+(((-1.0)*x187))+((gconst0*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x189=IKPowWithIntegerCheck(IKsign(gconst0),-1);
if(!x189.valid){
continue;
}
CheckValue<IkReal> x190 = IKatan2WithCheck(IkReal(new_r10),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x190.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x189.value)))+(x190.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x191=IKcos(j2);
IkReal x192=IKsin(j2);
IkReal x193=((1.0)*x192);
IkReal x194=((1.0)*x191);
if((((1.0)+(((-1.0)*(gconst0*gconst0))))) < -0.00001)
continue;
IkReal x195=IKsqrt(((1.0)+(((-1.0)*(gconst0*gconst0)))));
evalcond[0]=x192;
evalcond[1]=((-1.0)*x191);
evalcond[2]=((((-1.0)*gconst0*x194))+new_r11);
evalcond[3]=((((-1.0)*gconst0*x193))+new_r10);
evalcond[4]=((((-1.0)*x194*x195))+new_r01);
evalcond[5]=(new_r00+(((-1.0)*x193*x195)));
evalcond[6]=(((new_r00*x195))+(((-1.0)*x193))+((gconst0*new_r10)));
evalcond[7]=(((new_r01*x195))+(((-1.0)*x194))+((gconst0*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
CheckValue<IkReal> x196=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x196.valid){
continue;
}
if((x196.value) < -0.00001)
continue;
IkReal gconst1=IKsqrt(x196.value);
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs((cj0+(((-1.0)*gconst1)))))+(IKabs(((-1.0)+(IKsign(sj0)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
if((((1.0)+(((-1.0)*(gconst1*gconst1))))) < -0.00001)
continue;
sj0=IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1)))));
cj0=gconst1;
if( (gconst1) < -1-IKFAST_SINCOS_THRESH || (gconst1) > 1+IKFAST_SINCOS_THRESH )
    continue;
j0=IKacos(gconst1);
CheckValue<IkReal> x197=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x197.valid){
continue;
}
if((x197.value) < -0.00001)
continue;
IkReal gconst1=IKsqrt(x197.value);
j2eval[0]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if((((1.0)+(((-1.0)*(gconst1*gconst1))))) < -0.00001)
continue;
CheckValue<IkReal> x198=IKPowWithIntegerCheck(gconst1,-1);
if(!x198.valid){
continue;
}
if( IKabs((((gconst1*new_r10))+(((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1)))))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r11*(x198.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((gconst1*new_r10))+(((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1))))))))))+IKsqr((new_r11*(x198.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((((gconst1*new_r10))+(((-1.0)*new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1))))))))), (new_r11*(x198.value)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x199=IKcos(j2);
IkReal x200=IKsin(j2);
IkReal x201=((1.0)*gconst1);
if((((1.0)+(((-1.0)*gconst1*x201)))) < -0.00001)
continue;
IkReal x202=IKsqrt(((1.0)+(((-1.0)*gconst1*x201))));
IkReal x203=((1.0)*x202);
evalcond[0]=x200;
evalcond[1]=((-1.0)*x199);
evalcond[2]=((((-1.0)*x199*x201))+new_r11);
evalcond[3]=((((-1.0)*x200*x201))+new_r10);
evalcond[4]=(new_r01+((x199*x202)));
evalcond[5]=(((x200*x202))+new_r00);
evalcond[6]=((((-1.0)*new_r00*x203))+((gconst1*new_r10))+(((-1.0)*x200)));
evalcond[7]=(((gconst1*new_r11))+(((-1.0)*x199))+(((-1.0)*new_r01*x203)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x204=IKPowWithIntegerCheck(IKsign(gconst1),-1);
if(!x204.valid){
continue;
}
CheckValue<IkReal> x205 = IKatan2WithCheck(IkReal(new_r10),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x205.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x204.value)))+(x205.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x206=IKcos(j2);
IkReal x207=IKsin(j2);
IkReal x208=((1.0)*gconst1);
if((((1.0)+(((-1.0)*gconst1*x208)))) < -0.00001)
continue;
IkReal x209=IKsqrt(((1.0)+(((-1.0)*gconst1*x208))));
IkReal x210=((1.0)*x209);
evalcond[0]=x207;
evalcond[1]=((-1.0)*x206);
evalcond[2]=((((-1.0)*x206*x208))+new_r11);
evalcond[3]=((((-1.0)*x207*x208))+new_r10);
evalcond[4]=(new_r01+((x206*x209)));
evalcond[5]=(new_r00+((x207*x209)));
evalcond[6]=(((gconst1*new_r10))+(((-1.0)*new_r00*x210))+(((-1.0)*x207)));
evalcond[7]=(((gconst1*new_r11))+(((-1.0)*new_r01*x210))+(((-1.0)*x206)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
CheckValue<IkReal> x211=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x211.valid){
continue;
}
if((x211.value) < -0.00001)
continue;
IkReal gconst1=IKsqrt(x211.value);
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.0)+(IKsign(sj0)))))+(IKabs((cj0+(((-1.0)*gconst1)))))), 6.28318530717959)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
if((((1.0)+(((-1.0)*(gconst1*gconst1))))) < -0.00001)
continue;
sj0=((-1.0)*(IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1)))))));
cj0=gconst1;
if( (gconst1) < -1-IKFAST_SINCOS_THRESH || (gconst1) > 1+IKFAST_SINCOS_THRESH )
    continue;
j0=((-1.0)*(IKacos(gconst1)));
CheckValue<IkReal> x212=IKPowWithIntegerCheck(((1.0)+(((-1.0)*(new_r22*new_r22)))),-1);
if(!x212.valid){
continue;
}
if((x212.value) < -0.00001)
continue;
IkReal gconst1=IKsqrt(x212.value);
j2eval[0]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if((((1.0)+(((-1.0)*(gconst1*gconst1))))) < -0.00001)
continue;
CheckValue<IkReal> x213=IKPowWithIntegerCheck(gconst1,-1);
if(!x213.valid){
continue;
}
if( IKabs((((gconst1*new_r10))+((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1)))))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r11*(x213.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((((gconst1*new_r10))+((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1))))))))))+IKsqr((new_r11*(x213.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((((gconst1*new_r10))+((new_r00*(IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1))))))))), (new_r11*(x213.value)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x214=IKcos(j2);
IkReal x215=IKsin(j2);
IkReal x216=((1.0)*x214);
IkReal x217=((1.0)*x215);
if((((1.0)+(((-1.0)*(gconst1*gconst1))))) < -0.00001)
continue;
IkReal x218=IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1)))));
evalcond[0]=x215;
evalcond[1]=((-1.0)*x214);
evalcond[2]=((((-1.0)*gconst1*x216))+new_r11);
evalcond[3]=((((-1.0)*gconst1*x217))+new_r10);
evalcond[4]=(new_r01+(((-1.0)*x216*x218)));
evalcond[5]=((((-1.0)*x217*x218))+new_r00);
evalcond[6]=(((gconst1*new_r10))+((new_r00*x218))+(((-1.0)*x217)));
evalcond[7]=(((gconst1*new_r11))+((new_r01*x218))+(((-1.0)*x216)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x219=IKPowWithIntegerCheck(IKsign(gconst1),-1);
if(!x219.valid){
continue;
}
CheckValue<IkReal> x220 = IKatan2WithCheck(IkReal(new_r10),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x220.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x219.value)))+(x220.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x221=IKcos(j2);
IkReal x222=IKsin(j2);
IkReal x223=((1.0)*x221);
IkReal x224=((1.0)*x222);
if((((1.0)+(((-1.0)*(gconst1*gconst1))))) < -0.00001)
continue;
IkReal x225=IKsqrt(((1.0)+(((-1.0)*(gconst1*gconst1)))));
evalcond[0]=x222;
evalcond[1]=((-1.0)*x221);
evalcond[2]=((((-1.0)*gconst1*x223))+new_r11);
evalcond[3]=((((-1.0)*gconst1*x224))+new_r10);
evalcond[4]=((((-1.0)*x223*x225))+new_r01);
evalcond[5]=((((-1.0)*x224*x225))+new_r00);
evalcond[6]=(((new_r00*x225))+((gconst1*new_r10))+(((-1.0)*x224)));
evalcond[7]=(((new_r01*x225))+((gconst1*new_r11))+(((-1.0)*x223)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j2]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
IkReal x226=new_r22*new_r22;
CheckValue<IkReal> x227=IKPowWithIntegerCheck((((cj0*x226))+(((-1.0)*cj0))),-1);
if(!x227.valid){
continue;
}
CheckValue<IkReal> x228=IKPowWithIntegerCheck(((((-1.0)*sj0))+((sj0*x226))),-1);
if(!x228.valid){
continue;
}
if( IKabs(((x227.value)*(((((-1.0)*new_r01*new_r22))+(((-1.0)*new_r10)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((x228.value)*((((new_r10*new_r22))+new_r01)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x227.value)*(((((-1.0)*new_r01*new_r22))+(((-1.0)*new_r10))))))+IKsqr(((x228.value)*((((new_r10*new_r22))+new_r01))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((x227.value)*(((((-1.0)*new_r01*new_r22))+(((-1.0)*new_r10))))), ((x228.value)*((((new_r10*new_r22))+new_r01))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[10];
IkReal x229=IKsin(j2);
IkReal x230=IKcos(j2);
IkReal x231=(cj0*new_r22);
IkReal x232=(new_r22*sj0);
IkReal x233=((1.0)*sj0);
IkReal x234=((1.0)*x230);
IkReal x235=((1.0)*x229);
evalcond[0]=(((new_r11*sj0))+((new_r22*x229))+((cj0*new_r01)));
evalcond[1]=(((new_r11*x232))+((new_r01*x231))+x229);
evalcond[2]=((((-1.0)*x235))+((cj0*new_r10))+(((-1.0)*new_r00*x233)));
evalcond[3]=((((-1.0)*x234))+(((-1.0)*new_r01*x233))+((cj0*new_r11)));
evalcond[4]=(((x229*x231))+((sj0*x230))+new_r01);
evalcond[5]=((((-1.0)*new_r22*x234))+((new_r10*sj0))+((cj0*new_r00)));
evalcond[6]=(((sj0*x229))+(((-1.0)*x231*x234))+new_r00);
evalcond[7]=(((x229*x232))+(((-1.0)*cj0*x234))+new_r11);
evalcond[8]=(((new_r10*x232))+((new_r00*x231))+(((-1.0)*x234)));
evalcond[9]=((((-1.0)*cj0*x235))+new_r10+(((-1.0)*x232*x234)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
IkReal x236=((1.0)*new_r01);
CheckValue<IkReal> x237=IKPowWithIntegerCheck(new_r22,-1);
if(!x237.valid){
continue;
}
if( IKabs(((x237.value)*(((((-1.0)*cj0*x236))+(((-1.0)*new_r11*sj0)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs((((cj0*new_r11))+(((-1.0)*sj0*x236)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((x237.value)*(((((-1.0)*cj0*x236))+(((-1.0)*new_r11*sj0))))))+IKsqr((((cj0*new_r11))+(((-1.0)*sj0*x236))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((x237.value)*(((((-1.0)*cj0*x236))+(((-1.0)*new_r11*sj0))))), (((cj0*new_r11))+(((-1.0)*sj0*x236))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[10];
IkReal x238=IKsin(j2);
IkReal x239=IKcos(j2);
IkReal x240=(cj0*new_r22);
IkReal x241=(new_r22*sj0);
IkReal x242=((1.0)*sj0);
IkReal x243=((1.0)*x239);
IkReal x244=((1.0)*x238);
evalcond[0]=(((new_r11*sj0))+((new_r22*x238))+((cj0*new_r01)));
evalcond[1]=(x238+((new_r01*x240))+((new_r11*x241)));
evalcond[2]=((((-1.0)*new_r00*x242))+(((-1.0)*x244))+((cj0*new_r10)));
evalcond[3]=((((-1.0)*new_r01*x242))+(((-1.0)*x243))+((cj0*new_r11)));
evalcond[4]=(((sj0*x239))+((x238*x240))+new_r01);
evalcond[5]=(((new_r10*sj0))+(((-1.0)*new_r22*x243))+((cj0*new_r00)));
evalcond[6]=(((sj0*x238))+(((-1.0)*x240*x243))+new_r00);
evalcond[7]=(((x238*x241))+(((-1.0)*cj0*x243))+new_r11);
evalcond[8]=(((new_r00*x240))+(((-1.0)*x243))+((new_r10*x241)));
evalcond[9]=((((-1.0)*x241*x243))+(((-1.0)*cj0*x244))+new_r10);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
IkReal x245=cj0*cj0;
IkReal x246=(cj0*new_r22);
CheckValue<IkReal> x247 = IKatan2WithCheck(IkReal((((new_r00*sj0))+((new_r01*x246)))),IkReal(((((-1.0)*new_r00*x246))+((new_r01*sj0)))),IKFAST_ATAN2_MAGTHRESH);
if(!x247.valid){
continue;
}
CheckValue<IkReal> x248=IKPowWithIntegerCheck(IKsign(((-1.0)+x245+(((-1.0)*x245*(new_r22*new_r22))))),-1);
if(!x248.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(x247.value)+(((1.5707963267949)*(x248.value))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[10];
IkReal x249=IKsin(j2);
IkReal x250=IKcos(j2);
IkReal x251=(cj0*new_r22);
IkReal x252=(new_r22*sj0);
IkReal x253=((1.0)*sj0);
IkReal x254=((1.0)*x250);
IkReal x255=((1.0)*x249);
evalcond[0]=(((new_r11*sj0))+((new_r22*x249))+((cj0*new_r01)));
evalcond[1]=(((new_r01*x251))+x249+((new_r11*x252)));
evalcond[2]=(((cj0*new_r10))+(((-1.0)*new_r00*x253))+(((-1.0)*x255)));
evalcond[3]=((((-1.0)*new_r01*x253))+((cj0*new_r11))+(((-1.0)*x254)));
evalcond[4]=(((x249*x251))+((sj0*x250))+new_r01);
evalcond[5]=(((new_r10*sj0))+((cj0*new_r00))+(((-1.0)*new_r22*x254)));
evalcond[6]=((((-1.0)*x251*x254))+new_r00+((sj0*x249)));
evalcond[7]=(((x249*x252))+(((-1.0)*cj0*x254))+new_r11);
evalcond[8]=(((new_r00*x251))+((new_r10*x252))+(((-1.0)*x254)));
evalcond[9]=((((-1.0)*x252*x254))+(((-1.0)*cj0*x255))+new_r10);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
    }

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j0, j2]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}

} else
{
{
IkReal j0array[1], cj0array[1], sj0array[1];
bool j0valid[1]={false};
_nj0 = 1;
CheckValue<IkReal> x257=IKPowWithIntegerCheck(sj1,-1);
if(!x257.valid){
continue;
}
IkReal x256=x257.value;
CheckValue<IkReal> x258=IKPowWithIntegerCheck(new_r12,-1);
if(!x258.valid){
continue;
}
if( IKabs((x256*(x258.value)*(((1.0)+(((-1.0)*(new_r02*new_r02)))+(((-1.0)*(cj1*cj1))))))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r02*x256)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x256*(x258.value)*(((1.0)+(((-1.0)*(new_r02*new_r02)))+(((-1.0)*(cj1*cj1)))))))+IKsqr((new_r02*x256))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j0array[0]=IKatan2((x256*(x258.value)*(((1.0)+(((-1.0)*(new_r02*new_r02)))+(((-1.0)*(cj1*cj1)))))), (new_r02*x256));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
for(int ij0 = 0; ij0 < 1; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 1; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[8];
IkReal x259=IKcos(j0);
IkReal x260=IKsin(j0);
IkReal x261=((1.0)*cj1);
IkReal x262=((1.0)*sj1);
IkReal x263=(new_r12*x260);
IkReal x264=(new_r02*x259);
evalcond[0]=((((-1.0)*x259*x262))+new_r02);
evalcond[1]=(new_r12+(((-1.0)*x260*x262)));
evalcond[2]=((((-1.0)*new_r02*x260))+((new_r12*x259)));
evalcond[3]=(x264+x263+(((-1.0)*x262)));
evalcond[4]=(((cj1*x264))+((cj1*x263))+(((-1.0)*new_r22*x262)));
evalcond[5]=((((-1.0)*new_r00*x259*x262))+(((-1.0)*new_r20*x261))+(((-1.0)*new_r10*x260*x262)));
evalcond[6]=((((-1.0)*new_r11*x260*x262))+(((-1.0)*new_r21*x261))+(((-1.0)*new_r01*x259*x262)));
evalcond[7]=((1.0)+(((-1.0)*x262*x264))+(((-1.0)*x262*x263))+(((-1.0)*new_r22*x261)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j2eval[3];
j2eval[0]=sj1;
j2eval[1]=IKsign(sj1);
j2eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[2];
j2eval[0]=sj0;
j2eval[1]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  )
{
{
IkReal j2eval[3];
j2eval[0]=cj0;
j2eval[1]=cj1;
j2eval[2]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[5];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j0)))), 6.28318530717959)));
evalcond[1]=new_r02;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[3];
sj0=1.0;
cj0=0;
j0=1.5707963267949;
j2eval[0]=sj1;
j2eval[1]=IKsign(sj1);
j2eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[3];
sj0=1.0;
cj0=0;
j0=1.5707963267949;
j2eval[0]=cj1;
j2eval[1]=IKsign(cj1);
j2eval[2]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[1];
sj0=1.0;
cj0=0;
j0=1.5707963267949;
j2eval[0]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[4];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j1))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r12;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r11))+IKsqr(new_r10)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r11), new_r10);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x265=IKsin(j2);
IkReal x266=((1.0)*(IKcos(j2)));
evalcond[0]=(x265+new_r11);
evalcond[1]=(new_r10+(((-1.0)*x266)));
evalcond[2]=((((-1.0)*x265))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x266)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j1)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r12;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r11)+IKsqr(((-1.0)*new_r10))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r11, ((-1.0)*new_r10));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x267=IKcos(j2);
IkReal x268=((1.0)*(IKsin(j2)));
evalcond[0]=(x267+new_r10);
evalcond[1]=(new_r11+(((-1.0)*x268)));
evalcond[2]=((((-1.0)*new_r00))+(((-1.0)*x268)));
evalcond[3]=((((-1.0)*x267))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r11;
evalcond[3]=new_r10;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x269=IKcos(j2);
IkReal x270=((1.0)*(IKsin(j2)));
evalcond[0]=(x269+new_r20);
evalcond[1]=((((-1.0)*x270))+new_r21);
evalcond[2]=((((-1.0)*x270))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*x269))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r11;
evalcond[3]=new_r10;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x271=IKsin(j2);
IkReal x272=((1.0)*(IKcos(j2)));
evalcond[0]=(x271+new_r21);
evalcond[1]=((((-1.0)*x272))+new_r20);
evalcond[2]=((((-1.0)*x271))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*x272))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r01))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r01));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[6];
IkReal x273=IKsin(j2);
IkReal x274=IKcos(j2);
IkReal x275=((-1.0)*x274);
evalcond[0]=x273;
evalcond[1]=(new_r22*x273);
evalcond[2]=x275;
evalcond[3]=(new_r22*x275);
evalcond[4]=((((-1.0)*x273))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x274))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j2]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x276=IKPowWithIntegerCheck(sj1,-1);
if(!x276.valid){
continue;
}
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*(x276.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r20*(x276.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r20*(x276.value)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x277=IKsin(j2);
IkReal x278=IKcos(j2);
IkReal x279=((1.0)*sj1);
IkReal x280=((1.0)*x278);
evalcond[0]=(new_r20+((sj1*x278)));
evalcond[1]=(((cj1*x277))+new_r11);
evalcond[2]=((((-1.0)*x277*x279))+new_r21);
evalcond[3]=((((-1.0)*cj1*x280))+new_r10);
evalcond[4]=((((-1.0)*x277))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x280))+(((-1.0)*new_r01)));
evalcond[6]=(((cj1*new_r11))+(((-1.0)*new_r21*x279))+x277);
evalcond[7]=(((cj1*new_r10))+(((-1.0)*new_r20*x279))+(((-1.0)*x280)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x281=IKPowWithIntegerCheck(IKsign(cj1),-1);
if(!x281.valid){
continue;
}
CheckValue<IkReal> x282 = IKatan2WithCheck(IkReal(((-1.0)*new_r11)),IkReal(new_r10),IKFAST_ATAN2_MAGTHRESH);
if(!x282.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x281.value)))+(x282.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x283=IKsin(j2);
IkReal x284=IKcos(j2);
IkReal x285=((1.0)*sj1);
IkReal x286=((1.0)*x284);
evalcond[0]=(new_r20+((sj1*x284)));
evalcond[1]=(new_r11+((cj1*x283)));
evalcond[2]=(new_r21+(((-1.0)*x283*x285)));
evalcond[3]=((((-1.0)*cj1*x286))+new_r10);
evalcond[4]=((((-1.0)*x283))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x286))+(((-1.0)*new_r01)));
evalcond[6]=(((cj1*new_r11))+x283+(((-1.0)*new_r21*x285)));
evalcond[7]=(((cj1*new_r10))+(((-1.0)*x286))+(((-1.0)*new_r20*x285)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x287=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x287.valid){
continue;
}
CheckValue<IkReal> x288 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x288.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x287.value)))+(x288.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x289=IKsin(j2);
IkReal x290=IKcos(j2);
IkReal x291=((1.0)*sj1);
IkReal x292=((1.0)*x290);
evalcond[0]=(((sj1*x290))+new_r20);
evalcond[1]=(new_r11+((cj1*x289)));
evalcond[2]=((((-1.0)*x289*x291))+new_r21);
evalcond[3]=((((-1.0)*cj1*x292))+new_r10);
evalcond[4]=((((-1.0)*x289))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x292))+(((-1.0)*new_r01)));
evalcond[6]=((((-1.0)*new_r21*x291))+((cj1*new_r11))+x289);
evalcond[7]=((((-1.0)*new_r20*x291))+(((-1.0)*x292))+((cj1*new_r10)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j0)))), 6.28318530717959)));
evalcond[1]=new_r02;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r00)+IKsqr(new_r01)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r00, new_r01);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x293=IKcos(j2);
IkReal x294=IKsin(j2);
IkReal x295=((1.0)*sj1);
IkReal x296=((1.0)*new_r11);
IkReal x297=((1.0)*new_r10);
IkReal x298=((1.0)*x293);
evalcond[0]=(((sj1*x293))+new_r20);
evalcond[1]=((((-1.0)*x294))+new_r00);
evalcond[2]=((((-1.0)*x298))+new_r01);
evalcond[3]=(new_r21+(((-1.0)*x294*x295)));
evalcond[4]=(((cj1*x294))+(((-1.0)*x296)));
evalcond[5]=((((-1.0)*cj1*x298))+(((-1.0)*x297)));
evalcond[6]=((((-1.0)*cj1*x296))+(((-1.0)*new_r21*x295))+x294);
evalcond[7]=((((-1.0)*cj1*x297))+(((-1.0)*new_r20*x295))+(((-1.0)*x298)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x299=IKcos(j2);
IkReal x300=IKsin(j2);
IkReal x301=((1.0)*sj0);
IkReal x302=((1.0)*x300);
IkReal x303=((1.0)*x299);
evalcond[0]=(x299+new_r20);
evalcond[1]=(new_r21+(((-1.0)*x302)));
evalcond[2]=(((sj0*x299))+new_r01);
evalcond[3]=(((sj0*x300))+new_r00);
evalcond[4]=((((-1.0)*cj0*x303))+new_r11);
evalcond[5]=((((-1.0)*new_r02*x302))+new_r10);
evalcond[6]=((((-1.0)*new_r00*x301))+((cj0*new_r10))+(((-1.0)*x302)));
evalcond[7]=((((-1.0)*new_r01*x301))+((cj0*new_r11))+(((-1.0)*x303)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x304=IKcos(j2);
IkReal x305=IKsin(j2);
IkReal x306=((1.0)*sj0);
IkReal x307=((1.0)*x304);
evalcond[0]=(x305+new_r21);
evalcond[1]=(new_r20+(((-1.0)*x307)));
evalcond[2]=(((sj0*x304))+new_r01);
evalcond[3]=(((sj0*x305))+new_r00);
evalcond[4]=(((new_r02*x305))+new_r10);
evalcond[5]=((((-1.0)*cj0*x307))+new_r11);
evalcond[6]=((((-1.0)*x305))+(((-1.0)*new_r00*x306))+((cj0*new_r10)));
evalcond[7]=((((-1.0)*new_r01*x306))+((cj0*new_r11))+(((-1.0)*x307)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j1))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
IkReal x308=((1.0)*new_r01);
if( IKabs(((((-1.0)*cj0*x308))+(((-1.0)*new_r00*sj0)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-1.0)*sj0*x308))+((cj0*new_r00)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*cj0*x308))+(((-1.0)*new_r00*sj0))))+IKsqr(((((-1.0)*sj0*x308))+((cj0*new_r00))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((((-1.0)*cj0*x308))+(((-1.0)*new_r00*sj0))), ((((-1.0)*sj0*x308))+((cj0*new_r00))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x309=IKsin(j2);
IkReal x310=IKcos(j2);
IkReal x311=((1.0)*sj0);
IkReal x312=((1.0)*x310);
IkReal x313=(sj0*x309);
IkReal x314=(cj0*x309);
IkReal x315=(cj0*x312);
evalcond[0]=(((new_r11*sj0))+x309+((cj0*new_r01)));
evalcond[1]=(((sj0*x310))+x314+new_r01);
evalcond[2]=(((new_r10*sj0))+(((-1.0)*x312))+((cj0*new_r00)));
evalcond[3]=((((-1.0)*new_r00*x311))+(((-1.0)*x309))+((cj0*new_r10)));
evalcond[4]=((((-1.0)*x312))+((cj0*new_r11))+(((-1.0)*new_r01*x311)));
evalcond[5]=(x313+(((-1.0)*x315))+new_r00);
evalcond[6]=(x313+(((-1.0)*x315))+new_r11);
evalcond[7]=((((-1.0)*x310*x311))+(((-1.0)*x314))+new_r10);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j1)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
IkReal x316=((1.0)*sj0);
if( IKabs(((((-1.0)*new_r00*x316))+((cj0*new_r01)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-1.0)*cj0*new_r00))+(((-1.0)*new_r01*x316)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*new_r00*x316))+((cj0*new_r01))))+IKsqr(((((-1.0)*cj0*new_r00))+(((-1.0)*new_r01*x316))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((((-1.0)*new_r00*x316))+((cj0*new_r01))), ((((-1.0)*cj0*new_r00))+(((-1.0)*new_r01*x316))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x317=IKsin(j2);
IkReal x318=IKcos(j2);
IkReal x319=((1.0)*sj0);
IkReal x320=((1.0)*x317);
IkReal x321=(sj0*x318);
IkReal x322=((1.0)*x318);
IkReal x323=(cj0*x320);
evalcond[0]=(((new_r10*sj0))+x318+((cj0*new_r00)));
evalcond[1]=(((new_r11*sj0))+(((-1.0)*x320))+((cj0*new_r01)));
evalcond[2]=(((sj0*x317))+((cj0*x318))+new_r00);
evalcond[3]=((((-1.0)*new_r00*x319))+(((-1.0)*x320))+((cj0*new_r10)));
evalcond[4]=((((-1.0)*x322))+((cj0*new_r11))+(((-1.0)*new_r01*x319)));
evalcond[5]=((((-1.0)*x323))+x321+new_r01);
evalcond[6]=((((-1.0)*x323))+x321+new_r10);
evalcond[7]=((((-1.0)*cj0*x322))+new_r11+(((-1.0)*x317*x319)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j0))), 6.28318530717959)));
evalcond[1]=new_r12;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r10)+IKsqr(new_r11)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r10, new_r11);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x324=IKcos(j2);
IkReal x325=IKsin(j2);
IkReal x326=((1.0)*sj1);
IkReal x327=((1.0)*x324);
evalcond[0]=(((sj1*x324))+new_r20);
evalcond[1]=((((-1.0)*x325))+new_r10);
evalcond[2]=((((-1.0)*x327))+new_r11);
evalcond[3]=(((cj1*x325))+new_r01);
evalcond[4]=((((-1.0)*x325*x326))+new_r21);
evalcond[5]=((((-1.0)*cj1*x327))+new_r00);
evalcond[6]=(((cj1*new_r01))+x325+(((-1.0)*new_r21*x326)));
evalcond[7]=((((-1.0)*x327))+((cj1*new_r00))+(((-1.0)*new_r20*x326)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j0)))), 6.28318530717959)));
evalcond[1]=new_r12;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[3];
sj0=0;
cj0=-1.0;
j0=3.14159265358979;
j2eval[0]=sj1;
j2eval[1]=IKsign(sj1);
j2eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[1];
sj0=0;
cj0=-1.0;
j0=3.14159265358979;
j2eval[0]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal j2eval[2];
sj0=0;
cj0=-1.0;
j0=3.14159265358979;
j2eval[0]=cj1;
j2eval[1]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[4];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r01;
evalcond[3]=new_r00;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x328=IKcos(j2);
IkReal x329=((1.0)*(IKsin(j2)));
evalcond[0]=(x328+new_r20);
evalcond[1]=((((-1.0)*x329))+new_r21);
evalcond[2]=((((-1.0)*x329))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x328))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r01;
evalcond[3]=new_r00;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x330=IKsin(j2);
IkReal x331=((1.0)*(IKcos(j2)));
evalcond[0]=(x330+new_r21);
evalcond[1]=((((-1.0)*x331))+new_r20);
evalcond[2]=((((-1.0)*x330))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x331)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j1))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r01)+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r01, ((-1.0)*new_r11));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x332=IKsin(j2);
IkReal x333=((1.0)*(IKcos(j2)));
evalcond[0]=(x332+(((-1.0)*new_r01)));
evalcond[1]=((((-1.0)*x332))+(((-1.0)*new_r10)));
evalcond[2]=((((-1.0)*new_r11))+(((-1.0)*x333)));
evalcond[3]=((((-1.0)*x333))+(((-1.0)*new_r00)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j1)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(new_r00)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r10), new_r00);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x334=IKcos(j2);
IkReal x335=((1.0)*(IKsin(j2)));
evalcond[0]=(x334+(((-1.0)*new_r00)));
evalcond[1]=((((-1.0)*new_r10))+(((-1.0)*x335)));
evalcond[2]=((((-1.0)*x334))+(((-1.0)*new_r11)));
evalcond[3]=((((-1.0)*x335))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r10), ((-1.0)*new_r11));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[6];
IkReal x336=IKsin(j2);
IkReal x337=IKcos(j2);
IkReal x338=((-1.0)*x337);
evalcond[0]=x336;
evalcond[1]=(new_r22*x336);
evalcond[2]=x338;
evalcond[3]=(new_r22*x338);
evalcond[4]=((((-1.0)*x336))+(((-1.0)*new_r10)));
evalcond[5]=((((-1.0)*x337))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j2]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x339=IKPowWithIntegerCheck(cj1,-1);
if(!x339.valid){
continue;
}
CheckValue<IkReal> x340=IKPowWithIntegerCheck(sj1,-1);
if(!x340.valid){
continue;
}
if( IKabs((new_r01*(x339.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*(x340.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((new_r01*(x339.value)))+IKsqr(((-1.0)*new_r20*(x340.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((new_r01*(x339.value)), ((-1.0)*new_r20*(x340.value)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x341=IKsin(j2);
IkReal x342=IKcos(j2);
IkReal x343=((1.0)*new_r00);
IkReal x344=((1.0)*sj1);
IkReal x345=((1.0)*new_r01);
IkReal x346=((1.0)*x342);
evalcond[0]=(((sj1*x342))+new_r20);
evalcond[1]=((((-1.0)*x341*x344))+new_r21);
evalcond[2]=((((-1.0)*new_r10))+(((-1.0)*x341)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x346)));
evalcond[4]=(((cj1*x341))+(((-1.0)*x345)));
evalcond[5]=((((-1.0)*cj1*x346))+(((-1.0)*x343)));
evalcond[6]=((((-1.0)*cj1*x345))+x341+(((-1.0)*new_r21*x344)));
evalcond[7]=((((-1.0)*cj1*x343))+(((-1.0)*new_r20*x344))+(((-1.0)*x346)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x347=IKPowWithIntegerCheck(sj1,-1);
if(!x347.valid){
continue;
}
if( IKabs((new_r21*(x347.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((new_r21*(x347.value)))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((new_r21*(x347.value)), ((-1.0)*new_r11));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x348=IKsin(j2);
IkReal x349=IKcos(j2);
IkReal x350=((1.0)*new_r00);
IkReal x351=((1.0)*sj1);
IkReal x352=((1.0)*new_r01);
IkReal x353=((1.0)*x349);
evalcond[0]=(((sj1*x349))+new_r20);
evalcond[1]=((((-1.0)*x348*x351))+new_r21);
evalcond[2]=((((-1.0)*new_r10))+(((-1.0)*x348)));
evalcond[3]=((((-1.0)*x353))+(((-1.0)*new_r11)));
evalcond[4]=((((-1.0)*x352))+((cj1*x348)));
evalcond[5]=((((-1.0)*x350))+(((-1.0)*cj1*x353)));
evalcond[6]=((((-1.0)*cj1*x352))+x348+(((-1.0)*new_r21*x351)));
evalcond[7]=((((-1.0)*x353))+(((-1.0)*cj1*x350))+(((-1.0)*new_r20*x351)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x354=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x354.valid){
continue;
}
CheckValue<IkReal> x355 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x355.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x354.value)))+(x355.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x356=IKsin(j2);
IkReal x357=IKcos(j2);
IkReal x358=((1.0)*new_r00);
IkReal x359=((1.0)*sj1);
IkReal x360=((1.0)*new_r01);
IkReal x361=((1.0)*x357);
evalcond[0]=(((sj1*x357))+new_r20);
evalcond[1]=((((-1.0)*x356*x359))+new_r21);
evalcond[2]=((((-1.0)*x356))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x361))+(((-1.0)*new_r11)));
evalcond[4]=(((cj1*x356))+(((-1.0)*x360)));
evalcond[5]=((((-1.0)*x358))+(((-1.0)*cj1*x361)));
evalcond[6]=((((-1.0)*cj1*x360))+x356+(((-1.0)*new_r21*x359)));
evalcond[7]=((((-1.0)*x361))+(((-1.0)*cj1*x358))+(((-1.0)*new_r20*x359)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[1];
new_r21=0;
new_r20=0;
new_r02=0;
new_r12=0;
j2eval[0]=IKabs(new_r22);
if( IKabs(j2eval[0]) < 0.0000000100000000  )
{
continue; // no branches [j2]

} else
{
IkReal op[2+1], zeror[2];
int numroots;
op[0]=new_r22;
op[1]=0;
op[2]=((-1.0)*new_r22);
polyroots2(op,zeror,numroots);
IkReal j2array[2], cj2array[2], sj2array[2], tempj2array[1];
int numsolutions = 0;
for(int ij2 = 0; ij2 < numroots; ++ij2)
{
IkReal htj2 = zeror[ij2];
tempj2array[0]=((2.0)*(atan(htj2)));
for(int kj2 = 0; kj2 < 1; ++kj2)
{
j2array[numsolutions] = tempj2array[kj2];
if( j2array[numsolutions] > IKPI )
{
    j2array[numsolutions]-=IK2PI;
}
else if( j2array[numsolutions] < -IKPI )
{
    j2array[numsolutions]+=IK2PI;
}
sj2array[numsolutions] = IKsin(j2array[numsolutions]);
cj2array[numsolutions] = IKcos(j2array[numsolutions]);
numsolutions++;
}
}
bool j2valid[2]={true,true};
_nj2 = 2;
for(int ij2 = 0; ij2 < numsolutions; ++ij2)
    {
if( !j2valid[ij2] )
{
    continue;
}
    j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
htj2 = IKtan(j2/2);

_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < numsolutions; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
    }

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j2]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}
}
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x363=IKPowWithIntegerCheck(sj1,-1);
if(!x363.valid){
continue;
}
IkReal x362=x363.value;
CheckValue<IkReal> x364=IKPowWithIntegerCheck(cj0,-1);
if(!x364.valid){
continue;
}
CheckValue<IkReal> x365=IKPowWithIntegerCheck(cj1,-1);
if(!x365.valid){
continue;
}
if( IKabs((x362*(x364.value)*(x365.value)*((((new_r20*sj0))+(((-1.0)*new_r01*sj1)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*x362)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x362*(x364.value)*(x365.value)*((((new_r20*sj0))+(((-1.0)*new_r01*sj1))))))+IKsqr(((-1.0)*new_r20*x362))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((x362*(x364.value)*(x365.value)*((((new_r20*sj0))+(((-1.0)*new_r01*sj1))))), ((-1.0)*new_r20*x362));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[12];
IkReal x366=IKsin(j2);
IkReal x367=IKcos(j2);
IkReal x368=((1.0)*sj1);
IkReal x369=((1.0)*sj0);
IkReal x370=(cj0*new_r00);
IkReal x371=(cj0*cj1);
IkReal x372=(new_r11*sj0);
IkReal x373=(new_r10*sj0);
IkReal x374=((1.0)*x367);
IkReal x375=(cj1*x366);
IkReal x376=((1.0)*x366);
evalcond[0]=(((sj1*x367))+new_r20);
evalcond[1]=((((-1.0)*x366*x368))+new_r21);
evalcond[2]=(x375+x372+((cj0*new_r01)));
evalcond[3]=((((-1.0)*new_r00*x369))+(((-1.0)*x376))+((cj0*new_r10)));
evalcond[4]=((((-1.0)*x374))+((cj0*new_r11))+(((-1.0)*new_r01*x369)));
evalcond[5]=(((sj0*x367))+((x366*x371))+new_r01);
evalcond[6]=((((-1.0)*cj1*x374))+x373+x370);
evalcond[7]=(((sj0*x366))+(((-1.0)*x371*x374))+new_r00);
evalcond[8]=(((sj0*x375))+new_r11+(((-1.0)*cj0*x374)));
evalcond[9]=((((-1.0)*cj1*x367*x369))+new_r10+(((-1.0)*cj0*x376)));
evalcond[10]=(((new_r01*x371))+x366+((cj1*x372))+(((-1.0)*new_r21*x368)));
evalcond[11]=((((-1.0)*new_r20*x368))+(((-1.0)*x374))+((cj1*x373))+((cj1*x370)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x378=IKPowWithIntegerCheck(sj1,-1);
if(!x378.valid){
continue;
}
IkReal x377=x378.value;
CheckValue<IkReal> x379=IKPowWithIntegerCheck(sj0,-1);
if(!x379.valid){
continue;
}
if( IKabs((x377*(x379.value)*(((((-1.0)*cj0*cj1*new_r20))+(((-1.0)*new_r00*sj1)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*x377)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x377*(x379.value)*(((((-1.0)*cj0*cj1*new_r20))+(((-1.0)*new_r00*sj1))))))+IKsqr(((-1.0)*new_r20*x377))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((x377*(x379.value)*(((((-1.0)*cj0*cj1*new_r20))+(((-1.0)*new_r00*sj1))))), ((-1.0)*new_r20*x377));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[12];
IkReal x380=IKsin(j2);
IkReal x381=IKcos(j2);
IkReal x382=((1.0)*sj1);
IkReal x383=((1.0)*sj0);
IkReal x384=(cj0*new_r00);
IkReal x385=(cj0*cj1);
IkReal x386=(new_r11*sj0);
IkReal x387=(new_r10*sj0);
IkReal x388=((1.0)*x381);
IkReal x389=(cj1*x380);
IkReal x390=((1.0)*x380);
evalcond[0]=(new_r20+((sj1*x381)));
evalcond[1]=((((-1.0)*x380*x382))+new_r21);
evalcond[2]=(x386+x389+((cj0*new_r01)));
evalcond[3]=((((-1.0)*x390))+(((-1.0)*new_r00*x383))+((cj0*new_r10)));
evalcond[4]=((((-1.0)*new_r01*x383))+(((-1.0)*x388))+((cj0*new_r11)));
evalcond[5]=(new_r01+((sj0*x381))+((x380*x385)));
evalcond[6]=((((-1.0)*cj1*x388))+x387+x384);
evalcond[7]=(new_r00+((sj0*x380))+(((-1.0)*x385*x388)));
evalcond[8]=((((-1.0)*cj0*x388))+new_r11+((sj0*x389)));
evalcond[9]=((((-1.0)*cj0*x390))+(((-1.0)*cj1*x381*x383))+new_r10);
evalcond[10]=(((cj1*x386))+((new_r01*x385))+x380+(((-1.0)*new_r21*x382)));
evalcond[11]=(((cj1*x387))+((cj1*x384))+(((-1.0)*new_r20*x382))+(((-1.0)*x388)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x391=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x391.valid){
continue;
}
CheckValue<IkReal> x392 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x392.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x391.value)))+(x392.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[12];
IkReal x393=IKsin(j2);
IkReal x394=IKcos(j2);
IkReal x395=((1.0)*sj1);
IkReal x396=((1.0)*sj0);
IkReal x397=(cj0*new_r00);
IkReal x398=(cj0*cj1);
IkReal x399=(new_r11*sj0);
IkReal x400=(new_r10*sj0);
IkReal x401=((1.0)*x394);
IkReal x402=(cj1*x393);
IkReal x403=((1.0)*x393);
evalcond[0]=(((sj1*x394))+new_r20);
evalcond[1]=(new_r21+(((-1.0)*x393*x395)));
evalcond[2]=(x399+x402+((cj0*new_r01)));
evalcond[3]=((((-1.0)*new_r00*x396))+((cj0*new_r10))+(((-1.0)*x403)));
evalcond[4]=((((-1.0)*new_r01*x396))+((cj0*new_r11))+(((-1.0)*x401)));
evalcond[5]=(((sj0*x394))+new_r01+((x393*x398)));
evalcond[6]=((((-1.0)*cj1*x401))+x397+x400);
evalcond[7]=(((sj0*x393))+(((-1.0)*x398*x401))+new_r00);
evalcond[8]=(((sj0*x402))+(((-1.0)*cj0*x401))+new_r11);
evalcond[9]=((((-1.0)*cj1*x394*x396))+(((-1.0)*cj0*x403))+new_r10);
evalcond[10]=((((-1.0)*new_r21*x395))+x393+((new_r01*x398))+((cj1*x399)));
evalcond[11]=(((cj1*x397))+((cj1*x400))+(((-1.0)*x401))+(((-1.0)*new_r20*x395)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x404=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x404.valid){
continue;
}
CheckValue<IkReal> x405 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x405.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x404.value)))+(x405.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[2];
evalcond[0]=(((sj1*(IKcos(j2))))+new_r20);
evalcond[1]=((((-1.0)*sj1*(IKsin(j2))))+new_r21);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j0eval[3];
j0eval[0]=sj1;
j0eval[1]=((IKabs(new_r12))+(IKabs(new_r02)));
j0eval[2]=IKsign(sj1);
if( IKabs(j0eval[0]) < 0.0000010000000000  || IKabs(j0eval[1]) < 0.0000010000000000  || IKabs(j0eval[2]) < 0.0000010000000000  )
{
{
IkReal j0eval[2];
j0eval[0]=cj2;
j0eval[1]=sj1;
if( IKabs(j0eval[0]) < 0.0000010000000000  || IKabs(j0eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[5];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j2)))), 6.28318530717959)));
evalcond[1]=new_r20;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j0array[1], cj0array[1], sj0array[1];
bool j0valid[1]={false};
_nj0 = 1;
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(new_r10)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j0array[0]=IKatan2(((-1.0)*new_r00), new_r10);
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
for(int ij0 = 0; ij0 < 1; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 1; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[18];
IkReal x406=IKsin(j0);
IkReal x407=IKcos(j0);
IkReal x408=((1.0)*sj1);
IkReal x409=((1.0)*cj1);
IkReal x410=(new_r10*x406);
IkReal x411=(new_r01*x407);
IkReal x412=(new_r00*x407);
IkReal x413=((1.0)*x406);
IkReal x414=(new_r11*x406);
IkReal x415=(new_r12*x406);
IkReal x416=(cj1*x407);
IkReal x417=(new_r02*x407);
evalcond[0]=(x406+new_r00);
evalcond[1]=(x416+new_r01);
evalcond[2]=(new_r11+((cj1*x406)));
evalcond[3]=(new_r10+(((-1.0)*x407)));
evalcond[4]=((((-1.0)*x407*x408))+new_r02);
evalcond[5]=((((-1.0)*x406*x408))+new_r12);
evalcond[6]=(x412+x410);
evalcond[7]=(((new_r12*x407))+(((-1.0)*new_r02*x413)));
evalcond[8]=(((new_r11*x407))+(((-1.0)*new_r01*x413)));
evalcond[9]=(cj1+x411+x414);
evalcond[10]=((-1.0)+((new_r10*x407))+(((-1.0)*new_r00*x413)));
evalcond[11]=(((cj1*x412))+((cj1*x410)));
evalcond[12]=(x417+x415+(((-1.0)*x408)));
evalcond[13]=((((-1.0)*x408*x412))+(((-1.0)*x408*x410)));
evalcond[14]=(((new_r02*x416))+((cj1*x415))+(((-1.0)*new_r22*x408)));
evalcond[15]=((1.0)+(((-1.0)*new_r21*x408))+((cj1*x414))+((cj1*x411)));
evalcond[16]=((((-1.0)*new_r21*x409))+(((-1.0)*x408*x411))+(((-1.0)*x408*x414)));
evalcond[17]=((1.0)+(((-1.0)*x408*x415))+(((-1.0)*x408*x417))+(((-1.0)*new_r22*x409)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[12]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[13]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[14]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[15]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[16]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[17]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j2)))), 6.28318530717959)));
evalcond[1]=new_r20;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j0array[1], cj0array[1], sj0array[1];
bool j0valid[1]={false};
_nj0 = 1;
if( IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r00)+IKsqr(((-1.0)*new_r10))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j0array[0]=IKatan2(new_r00, ((-1.0)*new_r10));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
for(int ij0 = 0; ij0 < 1; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 1; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[18];
IkReal x418=IKcos(j0);
IkReal x419=IKsin(j0);
IkReal x420=((1.0)*sj1);
IkReal x421=((1.0)*cj1);
IkReal x422=(new_r10*x419);
IkReal x423=(new_r01*x418);
IkReal x424=(new_r00*x418);
IkReal x425=((1.0)*x419);
IkReal x426=(new_r11*x419);
IkReal x427=(new_r12*x419);
IkReal x428=(new_r02*x418);
evalcond[0]=(x418+new_r10);
evalcond[1]=((((-1.0)*x425))+new_r00);
evalcond[2]=((((-1.0)*x418*x420))+new_r02);
evalcond[3]=((((-1.0)*x419*x420))+new_r12);
evalcond[4]=((((-1.0)*x418*x421))+new_r01);
evalcond[5]=((((-1.0)*x419*x421))+new_r11);
evalcond[6]=(x424+x422);
evalcond[7]=(((new_r12*x418))+(((-1.0)*new_r02*x425)));
evalcond[8]=(((new_r11*x418))+(((-1.0)*new_r01*x425)));
evalcond[9]=((1.0)+(((-1.0)*new_r00*x425))+((new_r10*x418)));
evalcond[10]=(((cj1*x422))+((cj1*x424)));
evalcond[11]=((((-1.0)*x420))+x428+x427);
evalcond[12]=((((-1.0)*x421))+x426+x423);
evalcond[13]=((((-1.0)*x420*x424))+(((-1.0)*x420*x422)));
evalcond[14]=((((-1.0)*new_r22*x420))+((cj1*x428))+((cj1*x427)));
evalcond[15]=((-1.0)+(sj1*sj1)+((cj1*x423))+((cj1*x426)));
evalcond[16]=((((-1.0)*x420*x426))+(((-1.0)*x420*x423))+(((-1.0)*new_r21*x421)));
evalcond[17]=((1.0)+(((-1.0)*x420*x427))+(((-1.0)*x420*x428))+(((-1.0)*new_r22*x421)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[12]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[13]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[14]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[15]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[16]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[17]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j1))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j0array[1], cj0array[1], sj0array[1];
bool j0valid[1]={false};
_nj0 = 1;
IkReal x429=((1.0)*sj2);
if( IKabs(((((-1.0)*new_r00*x429))+(((-1.0)*cj2*new_r01)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((((cj2*new_r00))+(((-1.0)*new_r01*x429)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*new_r00*x429))+(((-1.0)*cj2*new_r01))))+IKsqr((((cj2*new_r00))+(((-1.0)*new_r01*x429))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j0array[0]=IKatan2(((((-1.0)*new_r00*x429))+(((-1.0)*cj2*new_r01))), (((cj2*new_r00))+(((-1.0)*new_r01*x429))));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
for(int ij0 = 0; ij0 < 1; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 1; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[8];
IkReal x430=IKcos(j0);
IkReal x431=IKsin(j0);
IkReal x432=((1.0)*cj2);
IkReal x433=((1.0)*sj2);
IkReal x434=(sj2*x431);
IkReal x435=((1.0)*x431);
IkReal x436=(x430*x432);
evalcond[0]=(((new_r01*x430))+sj2+((new_r11*x431)));
evalcond[1]=(((sj2*x430))+((cj2*x431))+new_r01);
evalcond[2]=((((-1.0)*x436))+x434+new_r00);
evalcond[3]=((((-1.0)*x436))+x434+new_r11);
evalcond[4]=((((-1.0)*x432))+((new_r00*x430))+((new_r10*x431)));
evalcond[5]=((((-1.0)*x430*x433))+(((-1.0)*x431*x432))+new_r10);
evalcond[6]=((((-1.0)*new_r00*x435))+(((-1.0)*x433))+((new_r10*x430)));
evalcond[7]=((((-1.0)*x432))+(((-1.0)*new_r01*x435))+((new_r11*x430)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j1)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j0array[1], cj0array[1], sj0array[1];
bool j0valid[1]={false};
_nj0 = 1;
IkReal x437=((1.0)*new_r00);
if( IKabs(((((-1.0)*cj2*new_r01))+(((-1.0)*sj2*x437)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-1.0)*cj2*x437))+((new_r01*sj2)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*cj2*new_r01))+(((-1.0)*sj2*x437))))+IKsqr(((((-1.0)*cj2*x437))+((new_r01*sj2))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j0array[0]=IKatan2(((((-1.0)*cj2*new_r01))+(((-1.0)*sj2*x437))), ((((-1.0)*cj2*x437))+((new_r01*sj2))));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
for(int ij0 = 0; ij0 < 1; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 1; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[8];
IkReal x438=IKcos(j0);
IkReal x439=IKsin(j0);
IkReal x440=((1.0)*sj2);
IkReal x441=(cj2*x439);
IkReal x442=(sj2*x439);
IkReal x443=((1.0)*x438);
IkReal x444=((1.0)*x439);
IkReal x445=(x438*x440);
evalcond[0]=(((new_r00*x438))+cj2+((new_r10*x439)));
evalcond[1]=(x442+((cj2*x438))+new_r00);
evalcond[2]=((((-1.0)*x445))+x441+new_r01);
evalcond[3]=((((-1.0)*x445))+x441+new_r10);
evalcond[4]=((((-1.0)*x440))+((new_r01*x438))+((new_r11*x439)));
evalcond[5]=((((-1.0)*cj2*x443))+(((-1.0)*x439*x440))+new_r11);
evalcond[6]=((((-1.0)*x440))+(((-1.0)*new_r00*x444))+((new_r10*x438)));
evalcond[7]=((((-1.0)*new_r01*x444))+((new_r11*x438))+(((-1.0)*cj2)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r12))+(IKabs(new_r02)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j0eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
j0eval[0]=((IKabs(new_r11))+(IKabs(new_r01)));
if( IKabs(j0eval[0]) < 0.0000010000000000  )
{
{
IkReal j0eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
j0eval[0]=((IKabs(new_r10))+(IKabs(new_r00)));
if( IKabs(j0eval[0]) < 0.0000010000000000  )
{
{
IkReal j0eval[1];
new_r02=0;
new_r12=0;
new_r20=0;
new_r21=0;
j0eval[0]=((IKabs((new_r11*new_r22)))+(IKabs((new_r01*new_r22))));
if( IKabs(j0eval[0]) < 0.0000010000000000  )
{
continue; // no branches [j0]

} else
{
{
IkReal j0array[2], cj0array[2], sj0array[2];
bool j0valid[2]={false};
_nj0 = 2;
CheckValue<IkReal> x447 = IKatan2WithCheck(IkReal((new_r01*new_r22)),IkReal((new_r11*new_r22)),IKFAST_ATAN2_MAGTHRESH);
if(!x447.valid){
continue;
}
IkReal x446=x447.value;
j0array[0]=((-1.0)*x446);
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
j0array[1]=((3.14159265358979)+(((-1.0)*x446)));
sj0array[1]=IKsin(j0array[1]);
cj0array[1]=IKcos(j0array[1]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
if( j0array[1] > IKPI )
{
    j0array[1]-=IK2PI;
}
else if( j0array[1] < -IKPI )
{    j0array[1]+=IK2PI;
}
j0valid[1] = true;
for(int ij0 = 0; ij0 < 2; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 2; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[5];
IkReal x448=IKcos(j0);
IkReal x449=IKsin(j0);
IkReal x450=(new_r10*x449);
IkReal x451=((1.0)*x449);
IkReal x452=(new_r00*x448);
evalcond[0]=(((new_r01*x448))+((new_r11*x449)));
evalcond[1]=(x452+x450);
evalcond[2]=((((-1.0)*new_r00*x451))+((new_r10*x448)));
evalcond[3]=((((-1.0)*new_r01*x451))+((new_r11*x448)));
evalcond[4]=(((new_r22*x452))+((new_r22*x450)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j0array[2], cj0array[2], sj0array[2];
bool j0valid[2]={false};
_nj0 = 2;
CheckValue<IkReal> x454 = IKatan2WithCheck(IkReal(new_r00),IkReal(new_r10),IKFAST_ATAN2_MAGTHRESH);
if(!x454.valid){
continue;
}
IkReal x453=x454.value;
j0array[0]=((-1.0)*x453);
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
j0array[1]=((3.14159265358979)+(((-1.0)*x453)));
sj0array[1]=IKsin(j0array[1]);
cj0array[1]=IKcos(j0array[1]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
if( j0array[1] > IKPI )
{
    j0array[1]-=IK2PI;
}
else if( j0array[1] < -IKPI )
{    j0array[1]+=IK2PI;
}
j0valid[1] = true;
for(int ij0 = 0; ij0 < 2; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 2; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[5];
IkReal x455=IKcos(j0);
IkReal x456=IKsin(j0);
IkReal x457=((1.0)*x456);
IkReal x458=(new_r11*x456);
IkReal x459=(new_r22*x455);
evalcond[0]=(((new_r01*x455))+x458);
evalcond[1]=(((new_r10*x455))+(((-1.0)*new_r00*x457)));
evalcond[2]=(((new_r11*x455))+(((-1.0)*new_r01*x457)));
evalcond[3]=(((new_r01*x459))+((new_r22*x458)));
evalcond[4]=(((new_r10*new_r22*x456))+((new_r00*x459)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j0array[2], cj0array[2], sj0array[2];
bool j0valid[2]={false};
_nj0 = 2;
CheckValue<IkReal> x461 = IKatan2WithCheck(IkReal(new_r01),IkReal(new_r11),IKFAST_ATAN2_MAGTHRESH);
if(!x461.valid){
continue;
}
IkReal x460=x461.value;
j0array[0]=((-1.0)*x460);
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
j0array[1]=((3.14159265358979)+(((-1.0)*x460)));
sj0array[1]=IKsin(j0array[1]);
cj0array[1]=IKcos(j0array[1]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
if( j0array[1] > IKPI )
{
    j0array[1]-=IK2PI;
}
else if( j0array[1] < -IKPI )
{    j0array[1]+=IK2PI;
}
j0valid[1] = true;
for(int ij0 = 0; ij0 < 2; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 2; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[5];
IkReal x462=IKcos(j0);
IkReal x463=IKsin(j0);
IkReal x464=(new_r10*x463);
IkReal x465=((1.0)*x463);
IkReal x466=(new_r00*x462);
evalcond[0]=(x466+x464);
evalcond[1]=(((new_r10*x462))+(((-1.0)*new_r00*x465)));
evalcond[2]=(((new_r11*x462))+(((-1.0)*new_r01*x465)));
evalcond[3]=(((new_r11*new_r22*x463))+((new_r01*new_r22*x462)));
evalcond[4]=(((new_r22*x464))+((new_r22*x466)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j0]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j0array[1], cj0array[1], sj0array[1];
bool j0valid[1]={false};
_nj0 = 1;
CheckValue<IkReal> x468=IKPowWithIntegerCheck(sj1,-1);
if(!x468.valid){
continue;
}
IkReal x467=x468.value;
CheckValue<IkReal> x469=IKPowWithIntegerCheck(cj2,-1);
if(!x469.valid){
continue;
}
if( IKabs((x467*(x469.value)*(((((-1.0)*new_r01*sj1))+(((-1.0)*cj1*new_r02*sj2)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs((new_r02*x467)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x467*(x469.value)*(((((-1.0)*new_r01*sj1))+(((-1.0)*cj1*new_r02*sj2))))))+IKsqr((new_r02*x467))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j0array[0]=IKatan2((x467*(x469.value)*(((((-1.0)*new_r01*sj1))+(((-1.0)*cj1*new_r02*sj2))))), (new_r02*x467));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
for(int ij0 = 0; ij0 < 1; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 1; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[18];
IkReal x470=IKcos(j0);
IkReal x471=IKsin(j0);
IkReal x472=((1.0)*cj2);
IkReal x473=((1.0)*sj1);
IkReal x474=((1.0)*cj1);
IkReal x475=((1.0)*sj2);
IkReal x476=(new_r10*x471);
IkReal x477=(new_r01*x470);
IkReal x478=(new_r00*x470);
IkReal x479=((1.0)*x471);
IkReal x480=(new_r11*x471);
IkReal x481=(new_r12*x471);
IkReal x482=(sj2*x471);
IkReal x483=(cj1*x470);
IkReal x484=(cj2*x471);
IkReal x485=(new_r02*x470);
evalcond[0]=(new_r02+(((-1.0)*x470*x473)));
evalcond[1]=((((-1.0)*x471*x473))+new_r12);
evalcond[2]=(((new_r12*x470))+(((-1.0)*new_r02*x479)));
evalcond[3]=(x484+((sj2*x483))+new_r01);
evalcond[4]=((((-1.0)*x473))+x481+x485);
evalcond[5]=(((cj1*sj2))+x480+x477);
evalcond[6]=(x482+new_r00+(((-1.0)*x472*x483)));
evalcond[7]=(((cj1*x482))+new_r11+(((-1.0)*x470*x472)));
evalcond[8]=(((new_r10*x470))+(((-1.0)*new_r00*x479))+(((-1.0)*x475)));
evalcond[9]=(((new_r11*x470))+(((-1.0)*x472))+(((-1.0)*new_r01*x479)));
evalcond[10]=((((-1.0)*cj1*x472))+x476+x478);
evalcond[11]=(new_r10+(((-1.0)*x470*x475))+(((-1.0)*cj1*x471*x472)));
evalcond[12]=(((new_r02*x483))+(((-1.0)*new_r22*x473))+((cj1*x481)));
evalcond[13]=(sj2+(((-1.0)*new_r21*x473))+((cj1*x477))+((cj1*x480)));
evalcond[14]=((((-1.0)*x473*x478))+(((-1.0)*x473*x476))+(((-1.0)*new_r20*x474)));
evalcond[15]=((((-1.0)*new_r21*x474))+(((-1.0)*x473*x477))+(((-1.0)*x473*x480)));
evalcond[16]=((1.0)+(((-1.0)*new_r22*x474))+(((-1.0)*x473*x481))+(((-1.0)*x473*x485)));
evalcond[17]=((((-1.0)*x472))+((cj1*x478))+((cj1*x476))+(((-1.0)*new_r20*x473)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[12]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[13]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[14]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[15]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[16]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[17]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j0array[1], cj0array[1], sj0array[1];
bool j0valid[1]={false};
_nj0 = 1;
CheckValue<IkReal> x486=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x486.valid){
continue;
}
CheckValue<IkReal> x487 = IKatan2WithCheck(IkReal(new_r12),IkReal(new_r02),IKFAST_ATAN2_MAGTHRESH);
if(!x487.valid){
continue;
}
j0array[0]=((-1.5707963267949)+(((1.5707963267949)*(x486.value)))+(x487.value));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
for(int ij0 = 0; ij0 < 1; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 1; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[18];
IkReal x488=IKcos(j0);
IkReal x489=IKsin(j0);
IkReal x490=((1.0)*cj2);
IkReal x491=((1.0)*sj1);
IkReal x492=((1.0)*cj1);
IkReal x493=((1.0)*sj2);
IkReal x494=(new_r10*x489);
IkReal x495=(new_r01*x488);
IkReal x496=(new_r00*x488);
IkReal x497=((1.0)*x489);
IkReal x498=(new_r11*x489);
IkReal x499=(new_r12*x489);
IkReal x500=(sj2*x489);
IkReal x501=(cj1*x488);
IkReal x502=(cj2*x489);
IkReal x503=(new_r02*x488);
evalcond[0]=((((-1.0)*x488*x491))+new_r02);
evalcond[1]=((((-1.0)*x489*x491))+new_r12);
evalcond[2]=((((-1.0)*new_r02*x497))+((new_r12*x488)));
evalcond[3]=(((sj2*x501))+x502+new_r01);
evalcond[4]=((((-1.0)*x491))+x499+x503);
evalcond[5]=(((cj1*sj2))+x498+x495);
evalcond[6]=((((-1.0)*x490*x501))+x500+new_r00);
evalcond[7]=((((-1.0)*x488*x490))+((cj1*x500))+new_r11);
evalcond[8]=((((-1.0)*new_r00*x497))+(((-1.0)*x493))+((new_r10*x488)));
evalcond[9]=((((-1.0)*x490))+(((-1.0)*new_r01*x497))+((new_r11*x488)));
evalcond[10]=((((-1.0)*cj1*x490))+x496+x494);
evalcond[11]=((((-1.0)*x488*x493))+(((-1.0)*cj1*x489*x490))+new_r10);
evalcond[12]=(((new_r02*x501))+(((-1.0)*new_r22*x491))+((cj1*x499)));
evalcond[13]=(sj2+(((-1.0)*new_r21*x491))+((cj1*x498))+((cj1*x495)));
evalcond[14]=((((-1.0)*new_r20*x492))+(((-1.0)*x491*x494))+(((-1.0)*x491*x496)));
evalcond[15]=((((-1.0)*x491*x498))+(((-1.0)*x491*x495))+(((-1.0)*new_r21*x492)));
evalcond[16]=((1.0)+(((-1.0)*x491*x499))+(((-1.0)*x491*x503))+(((-1.0)*new_r22*x492)));
evalcond[17]=((((-1.0)*new_r20*x491))+(((-1.0)*x490))+((cj1*x496))+((cj1*x494)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[12]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[13]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[14]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[15]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[16]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[17]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
}
}

}

}

} else
{
{
IkReal j0array[1], cj0array[1], sj0array[1];
bool j0valid[1]={false};
_nj0 = 1;
CheckValue<IkReal> x504=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x504.valid){
continue;
}
CheckValue<IkReal> x505 = IKatan2WithCheck(IkReal(new_r12),IkReal(new_r02),IKFAST_ATAN2_MAGTHRESH);
if(!x505.valid){
continue;
}
j0array[0]=((-1.5707963267949)+(((1.5707963267949)*(x504.value)))+(x505.value));
sj0array[0]=IKsin(j0array[0]);
cj0array[0]=IKcos(j0array[0]);
if( j0array[0] > IKPI )
{
    j0array[0]-=IK2PI;
}
else if( j0array[0] < -IKPI )
{    j0array[0]+=IK2PI;
}
j0valid[0] = true;
for(int ij0 = 0; ij0 < 1; ++ij0)
{
if( !j0valid[ij0] )
{
    continue;
}
_ij0[0] = ij0; _ij0[1] = -1;
for(int iij0 = ij0+1; iij0 < 1; ++iij0)
{
if( j0valid[iij0] && IKabs(cj0array[ij0]-cj0array[iij0]) < IKFAST_SOLUTION_THRESH && IKabs(sj0array[ij0]-sj0array[iij0]) < IKFAST_SOLUTION_THRESH )
{
    j0valid[iij0]=false; _ij0[1] = iij0; break; 
}
}
j0 = j0array[ij0]; cj0 = cj0array[ij0]; sj0 = sj0array[ij0];
{
IkReal evalcond[8];
IkReal x506=IKcos(j0);
IkReal x507=IKsin(j0);
IkReal x508=((1.0)*cj1);
IkReal x509=((1.0)*sj1);
IkReal x510=(new_r12*x507);
IkReal x511=(new_r02*x506);
evalcond[0]=((((-1.0)*x506*x509))+new_r02);
evalcond[1]=((((-1.0)*x507*x509))+new_r12);
evalcond[2]=((((-1.0)*new_r02*x507))+((new_r12*x506)));
evalcond[3]=((((-1.0)*x509))+x511+x510);
evalcond[4]=(((cj1*x511))+((cj1*x510))+(((-1.0)*new_r22*x509)));
evalcond[5]=((((-1.0)*new_r00*x506*x509))+(((-1.0)*new_r10*x507*x509))+(((-1.0)*new_r20*x508)));
evalcond[6]=((((-1.0)*new_r21*x508))+(((-1.0)*new_r11*x507*x509))+(((-1.0)*new_r01*x506*x509)));
evalcond[7]=((1.0)+(((-1.0)*x509*x511))+(((-1.0)*x509*x510))+(((-1.0)*new_r22*x508)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
IkReal j2eval[3];
j2eval[0]=sj1;
j2eval[1]=IKsign(sj1);
j2eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[2];
j2eval[0]=sj0;
j2eval[1]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  )
{
{
IkReal j2eval[3];
j2eval[0]=cj0;
j2eval[1]=cj1;
j2eval[2]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal evalcond[5];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j0)))), 6.28318530717959)));
evalcond[1]=new_r02;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[3];
sj0=1.0;
cj0=0;
j0=1.5707963267949;
j2eval[0]=sj1;
j2eval[1]=IKsign(sj1);
j2eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[3];
sj0=1.0;
cj0=0;
j0=1.5707963267949;
j2eval[0]=cj1;
j2eval[1]=IKsign(cj1);
j2eval[2]=((IKabs(new_r11))+(IKabs(new_r10)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[1];
sj0=1.0;
cj0=0;
j0=1.5707963267949;
j2eval[0]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal evalcond[4];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j1))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r12;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r11))+IKsqr(new_r10)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r11), new_r10);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x512=IKsin(j2);
IkReal x513=((1.0)*(IKcos(j2)));
evalcond[0]=(x512+new_r11);
evalcond[1]=(new_r10+(((-1.0)*x513)));
evalcond[2]=((((-1.0)*x512))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x513)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j1)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r12;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r11)+IKsqr(((-1.0)*new_r10))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r11, ((-1.0)*new_r10));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x514=IKcos(j2);
IkReal x515=((1.0)*(IKsin(j2)));
evalcond[0]=(x514+new_r10);
evalcond[1]=(new_r11+(((-1.0)*x515)));
evalcond[2]=((((-1.0)*new_r00))+(((-1.0)*x515)));
evalcond[3]=((((-1.0)*x514))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r11;
evalcond[3]=new_r10;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x516=IKcos(j2);
IkReal x517=((1.0)*(IKsin(j2)));
evalcond[0]=(x516+new_r20);
evalcond[1]=(new_r21+(((-1.0)*x517)));
evalcond[2]=((((-1.0)*new_r00))+(((-1.0)*x517)));
evalcond[3]=((((-1.0)*x516))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r11;
evalcond[3]=new_r10;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x518=IKsin(j2);
IkReal x519=((1.0)*(IKcos(j2)));
evalcond[0]=(x518+new_r21);
evalcond[1]=(new_r20+(((-1.0)*x519)));
evalcond[2]=((((-1.0)*x518))+(((-1.0)*new_r00)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x519)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r01)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r01))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r01));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[6];
IkReal x520=IKsin(j2);
IkReal x521=IKcos(j2);
IkReal x522=((-1.0)*x521);
evalcond[0]=x520;
evalcond[1]=(new_r22*x520);
evalcond[2]=x522;
evalcond[3]=(new_r22*x522);
evalcond[4]=((((-1.0)*x520))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x521))+(((-1.0)*new_r01)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j2]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x523=IKPowWithIntegerCheck(sj1,-1);
if(!x523.valid){
continue;
}
if( IKabs(((-1.0)*new_r00)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*(x523.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r00))+IKsqr(((-1.0)*new_r20*(x523.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r00), ((-1.0)*new_r20*(x523.value)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x524=IKsin(j2);
IkReal x525=IKcos(j2);
IkReal x526=((1.0)*sj1);
IkReal x527=((1.0)*x525);
evalcond[0]=(((sj1*x525))+new_r20);
evalcond[1]=(((cj1*x524))+new_r11);
evalcond[2]=((((-1.0)*x524*x526))+new_r21);
evalcond[3]=((((-1.0)*cj1*x527))+new_r10);
evalcond[4]=((((-1.0)*x524))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x527))+(((-1.0)*new_r01)));
evalcond[6]=(((cj1*new_r11))+(((-1.0)*new_r21*x526))+x524);
evalcond[7]=(((cj1*new_r10))+(((-1.0)*new_r20*x526))+(((-1.0)*x527)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x528=IKPowWithIntegerCheck(IKsign(cj1),-1);
if(!x528.valid){
continue;
}
CheckValue<IkReal> x529 = IKatan2WithCheck(IkReal(((-1.0)*new_r11)),IkReal(new_r10),IKFAST_ATAN2_MAGTHRESH);
if(!x529.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x528.value)))+(x529.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x530=IKsin(j2);
IkReal x531=IKcos(j2);
IkReal x532=((1.0)*sj1);
IkReal x533=((1.0)*x531);
evalcond[0]=(((sj1*x531))+new_r20);
evalcond[1]=(((cj1*x530))+new_r11);
evalcond[2]=((((-1.0)*x530*x532))+new_r21);
evalcond[3]=((((-1.0)*cj1*x533))+new_r10);
evalcond[4]=((((-1.0)*x530))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x533))+(((-1.0)*new_r01)));
evalcond[6]=(((cj1*new_r11))+(((-1.0)*new_r21*x532))+x530);
evalcond[7]=(((cj1*new_r10))+(((-1.0)*x533))+(((-1.0)*new_r20*x532)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x534=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x534.valid){
continue;
}
CheckValue<IkReal> x535 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x535.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x534.value)))+(x535.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x536=IKsin(j2);
IkReal x537=IKcos(j2);
IkReal x538=((1.0)*sj1);
IkReal x539=((1.0)*x537);
evalcond[0]=(((sj1*x537))+new_r20);
evalcond[1]=(((cj1*x536))+new_r11);
evalcond[2]=(new_r21+(((-1.0)*x536*x538)));
evalcond[3]=((((-1.0)*cj1*x539))+new_r10);
evalcond[4]=((((-1.0)*x536))+(((-1.0)*new_r00)));
evalcond[5]=((((-1.0)*x539))+(((-1.0)*new_r01)));
evalcond[6]=(((cj1*new_r11))+(((-1.0)*new_r21*x538))+x536);
evalcond[7]=(((cj1*new_r10))+(((-1.0)*x539))+(((-1.0)*new_r20*x538)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j0)))), 6.28318530717959)));
evalcond[1]=new_r02;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r00)+IKsqr(new_r01)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r00, new_r01);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x540=IKcos(j2);
IkReal x541=IKsin(j2);
IkReal x542=((1.0)*sj1);
IkReal x543=((1.0)*new_r11);
IkReal x544=((1.0)*new_r10);
IkReal x545=((1.0)*x540);
evalcond[0]=(((sj1*x540))+new_r20);
evalcond[1]=((((-1.0)*x541))+new_r00);
evalcond[2]=((((-1.0)*x545))+new_r01);
evalcond[3]=((((-1.0)*x541*x542))+new_r21);
evalcond[4]=((((-1.0)*x543))+((cj1*x541)));
evalcond[5]=((((-1.0)*cj1*x545))+(((-1.0)*x544)));
evalcond[6]=((((-1.0)*cj1*x543))+(((-1.0)*new_r21*x542))+x541);
evalcond[7]=((((-1.0)*cj1*x544))+(((-1.0)*new_r20*x542))+(((-1.0)*x545)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x546=IKcos(j2);
IkReal x547=IKsin(j2);
IkReal x548=((1.0)*sj0);
IkReal x549=((1.0)*x547);
IkReal x550=((1.0)*x546);
evalcond[0]=(x546+new_r20);
evalcond[1]=((((-1.0)*x549))+new_r21);
evalcond[2]=(((sj0*x546))+new_r01);
evalcond[3]=(((sj0*x547))+new_r00);
evalcond[4]=((((-1.0)*cj0*x550))+new_r11);
evalcond[5]=(new_r10+(((-1.0)*new_r02*x549)));
evalcond[6]=((((-1.0)*new_r00*x548))+(((-1.0)*x549))+((cj0*new_r10)));
evalcond[7]=((((-1.0)*new_r01*x548))+(((-1.0)*x550))+((cj0*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x551=IKcos(j2);
IkReal x552=IKsin(j2);
IkReal x553=((1.0)*sj0);
IkReal x554=((1.0)*x551);
evalcond[0]=(x552+new_r21);
evalcond[1]=((((-1.0)*x554))+new_r20);
evalcond[2]=(new_r01+((sj0*x551)));
evalcond[3]=(new_r00+((sj0*x552)));
evalcond[4]=(((new_r02*x552))+new_r10);
evalcond[5]=((((-1.0)*cj0*x554))+new_r11);
evalcond[6]=((((-1.0)*x552))+(((-1.0)*new_r00*x553))+((cj0*new_r10)));
evalcond[7]=((((-1.0)*new_r01*x553))+(((-1.0)*x554))+((cj0*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j1))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
IkReal x555=((1.0)*new_r01);
if( IKabs(((((-1.0)*cj0*x555))+(((-1.0)*new_r00*sj0)))) < IKFAST_ATAN2_MAGTHRESH && IKabs((((cj0*new_r00))+(((-1.0)*sj0*x555)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*cj0*x555))+(((-1.0)*new_r00*sj0))))+IKsqr((((cj0*new_r00))+(((-1.0)*sj0*x555))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((((-1.0)*cj0*x555))+(((-1.0)*new_r00*sj0))), (((cj0*new_r00))+(((-1.0)*sj0*x555))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x556=IKsin(j2);
IkReal x557=IKcos(j2);
IkReal x558=((1.0)*sj0);
IkReal x559=((1.0)*x557);
IkReal x560=(sj0*x556);
IkReal x561=(cj0*x556);
IkReal x562=(cj0*x559);
evalcond[0]=(((new_r11*sj0))+x556+((cj0*new_r01)));
evalcond[1]=(x561+new_r01+((sj0*x557)));
evalcond[2]=(((new_r10*sj0))+(((-1.0)*x559))+((cj0*new_r00)));
evalcond[3]=((((-1.0)*x556))+(((-1.0)*new_r00*x558))+((cj0*new_r10)));
evalcond[4]=((((-1.0)*new_r01*x558))+(((-1.0)*x559))+((cj0*new_r11)));
evalcond[5]=(x560+new_r00+(((-1.0)*x562)));
evalcond[6]=(x560+new_r11+(((-1.0)*x562)));
evalcond[7]=((((-1.0)*x561))+new_r10+(((-1.0)*x557*x558)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j1)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r12;
evalcond[4]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  && IKabs(evalcond[4]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
IkReal x563=((1.0)*sj0);
if( IKabs(((((-1.0)*new_r00*x563))+((cj0*new_r01)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((((-1.0)*cj0*new_r00))+(((-1.0)*new_r01*x563)))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((((-1.0)*new_r00*x563))+((cj0*new_r01))))+IKsqr(((((-1.0)*cj0*new_r00))+(((-1.0)*new_r01*x563))))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((((-1.0)*new_r00*x563))+((cj0*new_r01))), ((((-1.0)*cj0*new_r00))+(((-1.0)*new_r01*x563))));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x564=IKsin(j2);
IkReal x565=IKcos(j2);
IkReal x566=((1.0)*sj0);
IkReal x567=((1.0)*x564);
IkReal x568=(sj0*x565);
IkReal x569=((1.0)*x565);
IkReal x570=(cj0*x567);
evalcond[0]=(((new_r10*sj0))+x565+((cj0*new_r00)));
evalcond[1]=(((new_r11*sj0))+((cj0*new_r01))+(((-1.0)*x567)));
evalcond[2]=(((cj0*x565))+((sj0*x564))+new_r00);
evalcond[3]=((((-1.0)*new_r00*x566))+((cj0*new_r10))+(((-1.0)*x567)));
evalcond[4]=(((cj0*new_r11))+(((-1.0)*x569))+(((-1.0)*new_r01*x566)));
evalcond[5]=((((-1.0)*x570))+x568+new_r01);
evalcond[6]=((((-1.0)*x570))+x568+new_r10);
evalcond[7]=((((-1.0)*x564*x566))+(((-1.0)*cj0*x569))+new_r11);
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j0))), 6.28318530717959)));
evalcond[1]=new_r12;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r10) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r11) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r10)+IKsqr(new_r11)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r10, new_r11);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x571=IKcos(j2);
IkReal x572=IKsin(j2);
IkReal x573=((1.0)*sj1);
IkReal x574=((1.0)*x571);
evalcond[0]=(((sj1*x571))+new_r20);
evalcond[1]=((((-1.0)*x572))+new_r10);
evalcond[2]=((((-1.0)*x574))+new_r11);
evalcond[3]=(new_r01+((cj1*x572)));
evalcond[4]=(new_r21+(((-1.0)*x572*x573)));
evalcond[5]=((((-1.0)*cj1*x574))+new_r00);
evalcond[6]=(((cj1*new_r01))+x572+(((-1.0)*new_r21*x573)));
evalcond[7]=(((cj1*new_r00))+(((-1.0)*x574))+(((-1.0)*new_r20*x573)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j0)))), 6.28318530717959)));
evalcond[1]=new_r12;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[3];
sj0=0;
cj0=-1.0;
j0=3.14159265358979;
j2eval[0]=sj1;
j2eval[1]=IKsign(sj1);
j2eval[2]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  || IKabs(j2eval[2]) < 0.0000010000000000  )
{
{
IkReal j2eval[1];
sj0=0;
cj0=-1.0;
j0=3.14159265358979;
j2eval[0]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  )
{
{
IkReal j2eval[2];
sj0=0;
cj0=-1.0;
j0=3.14159265358979;
j2eval[0]=cj1;
j2eval[1]=sj1;
if( IKabs(j2eval[0]) < 0.0000010000000000  || IKabs(j2eval[1]) < 0.0000010000000000  )
{
{
IkReal evalcond[4];
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r01;
evalcond[3]=new_r00;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r21) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r21)+IKsqr(((-1.0)*new_r20))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r21, ((-1.0)*new_r20));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x575=IKcos(j2);
IkReal x576=((1.0)*(IKsin(j2)));
evalcond[0]=(x575+new_r20);
evalcond[1]=((((-1.0)*x576))+new_r21);
evalcond[2]=((((-1.0)*x576))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x575))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((1.5707963267949)+j1)))), 6.28318530717959)));
evalcond[1]=new_r22;
evalcond[2]=new_r01;
evalcond[3]=new_r00;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r21)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r20) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r21))+IKsqr(new_r20)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r21), new_r20);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x577=IKsin(j2);
IkReal x578=((1.0)*(IKcos(j2)));
evalcond[0]=(x577+new_r21);
evalcond[1]=((((-1.0)*x578))+new_r20);
evalcond[2]=((((-1.0)*x577))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x578))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(j1))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(new_r01) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(new_r01)+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(new_r01, ((-1.0)*new_r11));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x579=IKsin(j2);
IkReal x580=((1.0)*(IKcos(j2)));
evalcond[0]=(x579+(((-1.0)*new_r01)));
evalcond[1]=((((-1.0)*x579))+(((-1.0)*new_r10)));
evalcond[2]=((((-1.0)*new_r11))+(((-1.0)*x580)));
evalcond[3]=((((-1.0)*new_r00))+(((-1.0)*x580)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((-3.14159265358979)+(IKfmod(((3.14159265358979)+(IKabs(((-3.14159265358979)+j1)))), 6.28318530717959)));
evalcond[1]=new_r20;
evalcond[2]=new_r02;
evalcond[3]=new_r21;
if( IKabs(evalcond[0]) < 0.0000050000000000  && IKabs(evalcond[1]) < 0.0000050000000000  && IKabs(evalcond[2]) < 0.0000050000000000  && IKabs(evalcond[3]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(new_r00) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(new_r00)-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r10), new_r00);
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[4];
IkReal x581=IKcos(j2);
IkReal x582=((1.0)*(IKsin(j2)));
evalcond[0]=(x581+(((-1.0)*new_r00)));
evalcond[1]=((((-1.0)*new_r10))+(((-1.0)*x582)));
evalcond[2]=((((-1.0)*x581))+(((-1.0)*new_r11)));
evalcond[3]=((((-1.0)*new_r01))+(((-1.0)*x582)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
if( IKabs(((-1.0)*new_r10)) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr(((-1.0)*new_r10))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2(((-1.0)*new_r10), ((-1.0)*new_r11));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[6];
IkReal x583=IKsin(j2);
IkReal x584=IKcos(j2);
IkReal x585=((-1.0)*x584);
evalcond[0]=x583;
evalcond[1]=(new_r22*x583);
evalcond[2]=x585;
evalcond[3]=(new_r22*x585);
evalcond[4]=((((-1.0)*x583))+(((-1.0)*new_r10)));
evalcond[5]=((((-1.0)*x584))+(((-1.0)*new_r11)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j2]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x586=IKPowWithIntegerCheck(cj1,-1);
if(!x586.valid){
continue;
}
CheckValue<IkReal> x587=IKPowWithIntegerCheck(sj1,-1);
if(!x587.valid){
continue;
}
if( IKabs((new_r01*(x586.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*(x587.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((new_r01*(x586.value)))+IKsqr(((-1.0)*new_r20*(x587.value)))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((new_r01*(x586.value)), ((-1.0)*new_r20*(x587.value)));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x588=IKsin(j2);
IkReal x589=IKcos(j2);
IkReal x590=((1.0)*new_r00);
IkReal x591=((1.0)*sj1);
IkReal x592=((1.0)*new_r01);
IkReal x593=((1.0)*x589);
evalcond[0]=(((sj1*x589))+new_r20);
evalcond[1]=((((-1.0)*x588*x591))+new_r21);
evalcond[2]=((((-1.0)*x588))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*x593))+(((-1.0)*new_r11)));
evalcond[4]=((((-1.0)*x592))+((cj1*x588)));
evalcond[5]=((((-1.0)*x590))+(((-1.0)*cj1*x593)));
evalcond[6]=((((-1.0)*new_r21*x591))+(((-1.0)*cj1*x592))+x588);
evalcond[7]=((((-1.0)*new_r20*x591))+(((-1.0)*x593))+(((-1.0)*cj1*x590)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x594=IKPowWithIntegerCheck(sj1,-1);
if(!x594.valid){
continue;
}
if( IKabs((new_r21*(x594.value))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r11)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((new_r21*(x594.value)))+IKsqr(((-1.0)*new_r11))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((new_r21*(x594.value)), ((-1.0)*new_r11));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x595=IKsin(j2);
IkReal x596=IKcos(j2);
IkReal x597=((1.0)*new_r00);
IkReal x598=((1.0)*sj1);
IkReal x599=((1.0)*new_r01);
IkReal x600=((1.0)*x596);
evalcond[0]=(((sj1*x596))+new_r20);
evalcond[1]=((((-1.0)*x595*x598))+new_r21);
evalcond[2]=((((-1.0)*x595))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x600)));
evalcond[4]=(((cj1*x595))+(((-1.0)*x599)));
evalcond[5]=((((-1.0)*cj1*x600))+(((-1.0)*x597)));
evalcond[6]=((((-1.0)*new_r21*x598))+(((-1.0)*cj1*x599))+x595);
evalcond[7]=((((-1.0)*new_r20*x598))+(((-1.0)*cj1*x597))+(((-1.0)*x600)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x601=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x601.valid){
continue;
}
CheckValue<IkReal> x602 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x602.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x601.value)))+(x602.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[8];
IkReal x603=IKsin(j2);
IkReal x604=IKcos(j2);
IkReal x605=((1.0)*new_r00);
IkReal x606=((1.0)*sj1);
IkReal x607=((1.0)*new_r01);
IkReal x608=((1.0)*x604);
evalcond[0]=(((sj1*x604))+new_r20);
evalcond[1]=((((-1.0)*x603*x606))+new_r21);
evalcond[2]=((((-1.0)*x603))+(((-1.0)*new_r10)));
evalcond[3]=((((-1.0)*new_r11))+(((-1.0)*x608)));
evalcond[4]=(((cj1*x603))+(((-1.0)*x607)));
evalcond[5]=((((-1.0)*cj1*x608))+(((-1.0)*x605)));
evalcond[6]=((((-1.0)*cj1*x607))+x603+(((-1.0)*new_r21*x606)));
evalcond[7]=((((-1.0)*new_r20*x606))+(((-1.0)*cj1*x605))+(((-1.0)*x608)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
evalcond[0]=((IKabs(new_r20))+(IKabs(new_r21)));
if( IKabs(evalcond[0]) < 0.0000050000000000  )
{
bgotonextstatement=false;
{
IkReal j2eval[1];
new_r21=0;
new_r20=0;
new_r02=0;
new_r12=0;
j2eval[0]=IKabs(new_r22);
if( IKabs(j2eval[0]) < 0.0000000100000000  )
{
continue; // no branches [j2]

} else
{
IkReal op[2+1], zeror[2];
int numroots;
op[0]=new_r22;
op[1]=0;
op[2]=((-1.0)*new_r22);
polyroots2(op,zeror,numroots);
IkReal j2array[2], cj2array[2], sj2array[2], tempj2array[1];
int numsolutions = 0;
for(int ij2 = 0; ij2 < numroots; ++ij2)
{
IkReal htj2 = zeror[ij2];
tempj2array[0]=((2.0)*(atan(htj2)));
for(int kj2 = 0; kj2 < 1; ++kj2)
{
j2array[numsolutions] = tempj2array[kj2];
if( j2array[numsolutions] > IKPI )
{
    j2array[numsolutions]-=IK2PI;
}
else if( j2array[numsolutions] < -IKPI )
{
    j2array[numsolutions]+=IK2PI;
}
sj2array[numsolutions] = IKsin(j2array[numsolutions]);
cj2array[numsolutions] = IKcos(j2array[numsolutions]);
numsolutions++;
}
}
bool j2valid[2]={true,true};
_nj2 = 2;
for(int ij2 = 0; ij2 < numsolutions; ++ij2)
    {
if( !j2valid[ij2] )
{
    continue;
}
    j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
htj2 = IKtan(j2/2);

_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < numsolutions; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
    }

}

}

}
} while(0);
if( bgotonextstatement )
{
bool bgotonextstatement = true;
do
{
if( 1 )
{
bgotonextstatement=false;
continue; // branch miss [j2]

}
} while(0);
if( bgotonextstatement )
{
}
}
}
}
}
}
}
}
}
}
}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x610=IKPowWithIntegerCheck(sj1,-1);
if(!x610.valid){
continue;
}
IkReal x609=x610.value;
CheckValue<IkReal> x611=IKPowWithIntegerCheck(cj0,-1);
if(!x611.valid){
continue;
}
CheckValue<IkReal> x612=IKPowWithIntegerCheck(cj1,-1);
if(!x612.valid){
continue;
}
if( IKabs((x609*(x611.value)*(x612.value)*((((new_r20*sj0))+(((-1.0)*new_r01*sj1)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*x609)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x609*(x611.value)*(x612.value)*((((new_r20*sj0))+(((-1.0)*new_r01*sj1))))))+IKsqr(((-1.0)*new_r20*x609))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((x609*(x611.value)*(x612.value)*((((new_r20*sj0))+(((-1.0)*new_r01*sj1))))), ((-1.0)*new_r20*x609));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[12];
IkReal x613=IKsin(j2);
IkReal x614=IKcos(j2);
IkReal x615=((1.0)*sj1);
IkReal x616=((1.0)*sj0);
IkReal x617=(cj0*new_r00);
IkReal x618=(cj0*cj1);
IkReal x619=(new_r11*sj0);
IkReal x620=(new_r10*sj0);
IkReal x621=((1.0)*x614);
IkReal x622=(cj1*x613);
IkReal x623=((1.0)*x613);
evalcond[0]=(((sj1*x614))+new_r20);
evalcond[1]=((((-1.0)*x613*x615))+new_r21);
evalcond[2]=(x619+x622+((cj0*new_r01)));
evalcond[3]=((((-1.0)*x623))+(((-1.0)*new_r00*x616))+((cj0*new_r10)));
evalcond[4]=((((-1.0)*x621))+((cj0*new_r11))+(((-1.0)*new_r01*x616)));
evalcond[5]=(((sj0*x614))+((x613*x618))+new_r01);
evalcond[6]=((((-1.0)*cj1*x621))+x617+x620);
evalcond[7]=(((sj0*x613))+(((-1.0)*x618*x621))+new_r00);
evalcond[8]=((((-1.0)*cj0*x621))+((sj0*x622))+new_r11);
evalcond[9]=((((-1.0)*cj0*x623))+(((-1.0)*cj1*x614*x616))+new_r10);
evalcond[10]=((((-1.0)*new_r21*x615))+((cj1*x619))+x613+((new_r01*x618)));
evalcond[11]=((((-1.0)*x621))+((cj1*x617))+(((-1.0)*new_r20*x615))+((cj1*x620)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x625=IKPowWithIntegerCheck(sj1,-1);
if(!x625.valid){
continue;
}
IkReal x624=x625.value;
CheckValue<IkReal> x626=IKPowWithIntegerCheck(sj0,-1);
if(!x626.valid){
continue;
}
if( IKabs((x624*(x626.value)*(((((-1.0)*cj0*cj1*new_r20))+(((-1.0)*new_r00*sj1)))))) < IKFAST_ATAN2_MAGTHRESH && IKabs(((-1.0)*new_r20*x624)) < IKFAST_ATAN2_MAGTHRESH && IKabs(IKsqr((x624*(x626.value)*(((((-1.0)*cj0*cj1*new_r20))+(((-1.0)*new_r00*sj1))))))+IKsqr(((-1.0)*new_r20*x624))-1) <= IKFAST_SINCOS_THRESH )
    continue;
j2array[0]=IKatan2((x624*(x626.value)*(((((-1.0)*cj0*cj1*new_r20))+(((-1.0)*new_r00*sj1))))), ((-1.0)*new_r20*x624));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[12];
IkReal x627=IKsin(j2);
IkReal x628=IKcos(j2);
IkReal x629=((1.0)*sj1);
IkReal x630=((1.0)*sj0);
IkReal x631=(cj0*new_r00);
IkReal x632=(cj0*cj1);
IkReal x633=(new_r11*sj0);
IkReal x634=(new_r10*sj0);
IkReal x635=((1.0)*x628);
IkReal x636=(cj1*x627);
IkReal x637=((1.0)*x627);
evalcond[0]=(((sj1*x628))+new_r20);
evalcond[1]=((((-1.0)*x627*x629))+new_r21);
evalcond[2]=(x636+x633+((cj0*new_r01)));
evalcond[3]=((((-1.0)*new_r00*x630))+(((-1.0)*x637))+((cj0*new_r10)));
evalcond[4]=((((-1.0)*new_r01*x630))+(((-1.0)*x635))+((cj0*new_r11)));
evalcond[5]=(((sj0*x628))+new_r01+((x627*x632)));
evalcond[6]=((((-1.0)*cj1*x635))+x634+x631);
evalcond[7]=(((sj0*x627))+new_r00+(((-1.0)*x632*x635)));
evalcond[8]=((((-1.0)*cj0*x635))+((sj0*x636))+new_r11);
evalcond[9]=((((-1.0)*cj1*x628*x630))+(((-1.0)*cj0*x637))+new_r10);
evalcond[10]=(((new_r01*x632))+(((-1.0)*new_r21*x629))+x627+((cj1*x633)));
evalcond[11]=((((-1.0)*x635))+(((-1.0)*new_r20*x629))+((cj1*x634))+((cj1*x631)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}

} else
{
{
IkReal j2array[1], cj2array[1], sj2array[1];
bool j2valid[1]={false};
_nj2 = 1;
CheckValue<IkReal> x638=IKPowWithIntegerCheck(IKsign(sj1),-1);
if(!x638.valid){
continue;
}
CheckValue<IkReal> x639 = IKatan2WithCheck(IkReal(new_r21),IkReal(((-1.0)*new_r20)),IKFAST_ATAN2_MAGTHRESH);
if(!x639.valid){
continue;
}
j2array[0]=((-1.5707963267949)+(((1.5707963267949)*(x638.value)))+(x639.value));
sj2array[0]=IKsin(j2array[0]);
cj2array[0]=IKcos(j2array[0]);
if( j2array[0] > IKPI )
{
    j2array[0]-=IK2PI;
}
else if( j2array[0] < -IKPI )
{    j2array[0]+=IK2PI;
}
j2valid[0] = true;
for(int ij2 = 0; ij2 < 1; ++ij2)
{
if( !j2valid[ij2] )
{
    continue;
}
_ij2[0] = ij2; _ij2[1] = -1;
for(int iij2 = ij2+1; iij2 < 1; ++iij2)
{
if( j2valid[iij2] && IKabs(cj2array[ij2]-cj2array[iij2]) < IKFAST_SOLUTION_THRESH && IKabs(sj2array[ij2]-sj2array[iij2]) < IKFAST_SOLUTION_THRESH )
{
    j2valid[iij2]=false; _ij2[1] = iij2; break; 
}
}
j2 = j2array[ij2]; cj2 = cj2array[ij2]; sj2 = sj2array[ij2];
{
IkReal evalcond[12];
IkReal x640=IKsin(j2);
IkReal x641=IKcos(j2);
IkReal x642=((1.0)*sj1);
IkReal x643=((1.0)*sj0);
IkReal x644=(cj0*new_r00);
IkReal x645=(cj0*cj1);
IkReal x646=(new_r11*sj0);
IkReal x647=(new_r10*sj0);
IkReal x648=((1.0)*x641);
IkReal x649=(cj1*x640);
IkReal x650=((1.0)*x640);
evalcond[0]=(((sj1*x641))+new_r20);
evalcond[1]=((((-1.0)*x640*x642))+new_r21);
evalcond[2]=(x646+x649+((cj0*new_r01)));
evalcond[3]=((((-1.0)*x650))+(((-1.0)*new_r00*x643))+((cj0*new_r10)));
evalcond[4]=((((-1.0)*x648))+(((-1.0)*new_r01*x643))+((cj0*new_r11)));
evalcond[5]=(((sj0*x641))+((x640*x645))+new_r01);
evalcond[6]=((((-1.0)*cj1*x648))+x647+x644);
evalcond[7]=(((sj0*x640))+(((-1.0)*x645*x648))+new_r00);
evalcond[8]=(((sj0*x649))+(((-1.0)*cj0*x648))+new_r11);
evalcond[9]=((((-1.0)*cj1*x641*x643))+(((-1.0)*cj0*x650))+new_r10);
evalcond[10]=(((cj1*x646))+(((-1.0)*new_r21*x642))+((new_r01*x645))+x640);
evalcond[11]=(((cj1*x644))+((cj1*x647))+(((-1.0)*x648))+(((-1.0)*new_r20*x642)));
if( IKabs(evalcond[0]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[1]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[2]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[3]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[4]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[5]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[6]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[7]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[8]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[9]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[10]) > IKFAST_EVALCOND_THRESH  || IKabs(evalcond[11]) > IKFAST_EVALCOND_THRESH  )
{
continue;
}
}

{
std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(7);
vinfos[0].jointtype = 1;
vinfos[0].foffset = j0;
vinfos[0].indices[0] = _ij0[0];
vinfos[0].indices[1] = _ij0[1];
vinfos[0].maxsolutions = _nj0;
vinfos[1].jointtype = 1;
vinfos[1].foffset = j1;
vinfos[1].indices[0] = _ij1[0];
vinfos[1].indices[1] = _ij1[1];
vinfos[1].maxsolutions = _nj1;
vinfos[2].jointtype = 1;
vinfos[2].foffset = j2;
vinfos[2].indices[0] = _ij2[0];
vinfos[2].indices[1] = _ij2[1];
vinfos[2].maxsolutions = _nj2;
vinfos[3].jointtype = 1;
vinfos[3].foffset = j3;
vinfos[3].indices[0] = _ij3[0];
vinfos[3].indices[1] = _ij3[1];
vinfos[3].maxsolutions = _nj3;
vinfos[4].jointtype = 1;
vinfos[4].foffset = j4;
vinfos[4].indices[0] = _ij4[0];
vinfos[4].indices[1] = _ij4[1];
vinfos[4].maxsolutions = _nj4;
vinfos[5].jointtype = 1;
vinfos[5].foffset = j5;
vinfos[5].indices[0] = _ij5[0];
vinfos[5].indices[1] = _ij5[1];
vinfos[5].maxsolutions = _nj5;
vinfos[6].jointtype = 1;
vinfos[6].foffset = j6;
vinfos[6].indices[0] = _ij6[0];
vinfos[6].indices[1] = _ij6[1];
vinfos[6].maxsolutions = _nj6;
std::vector<int> vfree(0);
solutions.AddSolution(vinfos,vfree);
}
}
}

}

}
}
}

}

}
}
}
}
}static inline void polyroots3(IkReal rawcoeffs[3+1], IkReal rawroots[3], int& numroots)
{
    using std::complex;
    if( rawcoeffs[0] == 0 ) {
        // solve with one reduced degree
        polyroots2(&rawcoeffs[1], &rawroots[0], numroots);
        return;
    }
    IKFAST_ASSERT(rawcoeffs[0] != 0);
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
    complex<IkReal> coeffs[3];
    const int maxsteps = 110;
    for(int i = 0; i < 3; ++i) {
        coeffs[i] = complex<IkReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IkReal> roots[3];
    IkReal err[3];
    roots[0] = complex<IkReal>(1,0);
    roots[1] = complex<IkReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < 3; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < 3; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IkReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < 3; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < 3; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    numroots = 0;
    bool visited[3] = {false};
    for(int i = 0; i < 3; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IkReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < 3; ++j) {
                // care about error in real much more than imaginary
                if( abs(real(roots[i])-real(roots[j])) < tolsqrt && abs(imag(roots[i])-imag(roots[j])) < 0.002 ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
            if( IKabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}
static inline void polyroots2(IkReal rawcoeffs[2+1], IkReal rawroots[2], int& numroots) {
    IkReal det = rawcoeffs[1]*rawcoeffs[1]-4*rawcoeffs[0]*rawcoeffs[2];
    if( det < 0 ) {
        numroots=0;
    }
    else if( det == 0 ) {
        rawroots[0] = -0.5*rawcoeffs[1]/rawcoeffs[0];
        numroots = 1;
    }
    else {
        det = IKsqrt(det);
        rawroots[0] = (-rawcoeffs[1]+det)/(2*rawcoeffs[0]);
        rawroots[1] = (-rawcoeffs[1]-det)/(2*rawcoeffs[0]);//rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
        numroots = 2;
    }
}
static inline void polyroots4(IkReal rawcoeffs[4+1], IkReal rawroots[4], int& numroots)
{
    using std::complex;
    if( rawcoeffs[0] == 0 ) {
        // solve with one reduced degree
        polyroots3(&rawcoeffs[1], &rawroots[0], numroots);
        return;
    }
    IKFAST_ASSERT(rawcoeffs[0] != 0);
    const IkReal tol = 128.0*std::numeric_limits<IkReal>::epsilon();
    const IkReal tolsqrt = sqrt(std::numeric_limits<IkReal>::epsilon());
    complex<IkReal> coeffs[4];
    const int maxsteps = 110;
    for(int i = 0; i < 4; ++i) {
        coeffs[i] = complex<IkReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IkReal> roots[4];
    IkReal err[4];
    roots[0] = complex<IkReal>(1,0);
    roots[1] = complex<IkReal>(0.4,0.9); // any complex number not a root of unity works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < 4; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < 4; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IkReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < 4; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < 4; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }

    numroots = 0;
    bool visited[4] = {false};
    for(int i = 0; i < 4; ++i) {
        if( !visited[i] ) {
            // might be a multiple root, in which case it will have more error than the other roots
            // find any neighboring roots, and take the average
            complex<IkReal> newroot=roots[i];
            int n = 1;
            for(int j = i+1; j < 4; ++j) {
                // care about error in real much more than imaginary
                if( abs(real(roots[i])-real(roots[j])) < tolsqrt && abs(imag(roots[i])-imag(roots[j])) < 0.002 ) {
                    newroot += roots[j];
                    n += 1;
                    visited[j] = true;
                }
            }
            if( n > 1 ) {
                newroot /= n;
            }
            // there are still cases where even the mean is not accurate enough, until a better multi-root algorithm is used, need to use the sqrt
            if( IKabs(imag(newroot)) < tolsqrt ) {
                rawroots[numroots++] = real(newroot);
            }
        }
    }
}
};


/// solves the inverse kinematics equations.
/// \param pfree is an array specifying the free joints of the chain.
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions) {
IKSolver solver;
return solver.ComputeIk(eetrans,eerot,pfree,solutions);
}

// TODO Add XArmKinematics::inverse() here

int XArmKinematics::inverse(const double* T, double* q_sols, double q6_des) {
    IKSolver solver;

    // q_sols is declared as double q_sols[8*7];
    int num_sols = 0;
    double * pfree = new double(q6_des);

    // hold the results
    IkReal* eetrans = new IkReal[9];
    IkReal* eerot = new IkReal[3];

    for(int i=0; i< 3;++i){
       eerot[i*3+0] = T[i*4+0];
       eerot[i*3+1] = T[i*4+1];
       eerot[i*3+2] = T[i*4+2];
       eetrans[i] = T[i*4+3];
    }
    // vector<vector<double> > sols;

    IkSolutionList<IkReal> solutions;

    bool result = solver.ComputeIk(eetrans, eerot, pfree, solutions); // q_sols
    
    const int numJoints = 7;
    vector<IkReal> solvalues(GetNumJoints());
    
    for (int i=0; i <solutions.GetNumSolutions(); i++) {
        // get solution
        const IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
        for (int j = 0; j < solvalues.size(); j++)
        {
            q_sols[i*numJoints+j]=solvalues[j];
        }
    }

    return result;
}

// END TODO

// IKFAST_API bool ComputeIk2(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, IkSolutionListBase<IkReal>& solutions, void* pOpenRAVEManip) {
// IKSolver solver;
// return solver.ComputeIk(eetrans,eerot,pfree,solutions);
// }

IKFAST_API const char* GetKinematicsHash() { return "b38bae72e6f105bd9572e8ac73a65e61"; }

IKFAST_API const char* GetIkFastVersion() { return "0x1000004a"; }

#ifdef IKFAST_NAMESPACE
} // end namespace
#endif

// #ifndef IKFAST_NO_MAIN
// #include <stdio.h>
// #include <stdlib.h>
// #ifdef IKFAST_NAMESPACE
// using namespace IKFAST_NAMESPACE;
// #endif
// int main(int argc, char** argv)
// {
//     if( argc != 12+GetNumFreeParameters()+1 ) {
//         printf("\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
//                "Returns the ik solutions given the transformation of the end effector specified by\n"
//                "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
//                "There are %d free parameters that have to be specified.\n\n",GetNumFreeParameters());
//         return 1;
//     }

//     IkSolutionList<IkReal> solutions;
//     std::vector<IkReal> vfree(GetNumFreeParameters());
//     IkReal eerot[9],eetrans[3];
//     eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
//     eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
//     eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
//     for(std::size_t i = 0; i < vfree.size(); ++i)
//         vfree[i] = atof(argv[13+i]);
//     bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

//     if( !bSuccess ) {
//         fprintf(stderr,"Failed to get ik solution\n");
//         return -1;
//     }

//     printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
//     std::vector<IkReal> solvalues(GetNumJoints());
//     for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
//         const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
//         printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
//         std::vector<IkReal> vsolfree(sol.GetFree().size());
//         sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
//         for( std::size_t j = 0; j < solvalues.size(); ++j)
//             printf("%.15f, ", solvalues[j]);
//         printf("\n");
//     }
//     return 0;
// }
// #endif

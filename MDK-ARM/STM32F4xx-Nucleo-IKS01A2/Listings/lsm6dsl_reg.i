#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.c"


































 

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"



































 

 







 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"




 





 












 






   









 






#line 61 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 75 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   




 















 
#line 112 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











 





extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double  , double  );
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassify(double  );
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static inline __declspec(__nothrow) __attribute__((__pcs__("aapcs"))) int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 230 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"







   
  typedef float float_t;
  typedef double double_t;
#line 251 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"



extern const int math_errhandling;
#line 261 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __attribute__((const)) double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    inline double _sqrt(double __x) { return sqrt(__x); }


    inline float _sqrtf(float __x) { return __sqrtf(__x); }



    



 

extern __declspec(__nothrow) __attribute__((const)) double ceil(double  );
    
    
extern __declspec(__nothrow) __attribute__((const)) double fabs(double  );
    
    

extern __declspec(__nothrow) __attribute__((const)) double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
inline __declspec(__nothrow) __attribute__((const)) double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
inline __declspec(__nothrow) __attribute__((const)) float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 479 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) int ilogb(double  );
    

 
extern __declspec(__nothrow) int ilogbf(float  );
    

 
extern __declspec(__nothrow) int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) double logb(double  );
    

 
extern __declspec(__nothrow) float logbf(float  );
    

 
extern __declspec(__nothrow) long double logbl(long double  );
    

 
extern __declspec(__nothrow) double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __attribute__((const)) double rint(double  );
    

 
extern __declspec(__nothrow) double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __attribute__((const)) float _fabsf(float);  
inline __declspec(__nothrow) __attribute__((const)) float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __attribute__((const)) float ceilf(float  );
extern __declspec(__nothrow) __attribute__((const)) float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );






 
extern __declspec(__nothrow) double exp2(double  );  
extern __declspec(__nothrow) float exp2f(float  );
__declspec(__nothrow) long double exp2l(long double );
extern __declspec(__nothrow) double fdim(double  , double  );
extern __declspec(__nothrow) float fdimf(float  , float  );
__declspec(__nothrow) long double fdiml(long double , long double );
#line 803 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) double fma(double  , double  , double  );
extern __declspec(__nothrow) float fmaf(float  , float  , float  );

inline __declspec(__nothrow) long double fmal(long double __x, long double __y, long double __z)     { return (long double)fma((double)__x, (double)__y, (double)__z); }


extern __declspec(__nothrow) __attribute__((const)) double fmax(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fmaxf(float  , float  );
__declspec(__nothrow) long double fmaxl(long double , long double );
extern __declspec(__nothrow) __attribute__((const)) double fmin(double  , double  );
extern __declspec(__nothrow) __attribute__((const)) float fminf(float  , float  );
__declspec(__nothrow) long double fminl(long double , long double );
extern __declspec(__nothrow) double log2(double  );  
extern __declspec(__nothrow) float log2f(float  );
__declspec(__nothrow) long double log2l(long double );
extern __declspec(__nothrow) long lrint(double  );
extern __declspec(__nothrow) long lrintf(float  );

inline __declspec(__nothrow) long lrintl(long double __x)     { return lrint((double)__x); }


extern __declspec(__nothrow) long long llrint(double  );
extern __declspec(__nothrow) long long llrintf(float  );

inline __declspec(__nothrow) long long llrintl(long double __x)     { return llrint((double)__x); }


extern __declspec(__nothrow) long lround(double  );
extern __declspec(__nothrow) long lroundf(float  );

inline __declspec(__nothrow) long lroundl(long double __x)     { return lround((double)__x); }


extern __declspec(__nothrow) long long llround(double  );
extern __declspec(__nothrow) long long llroundf(float  );

inline __declspec(__nothrow) long long llroundl(long double __x)     { return llround((double)__x); }


extern __declspec(__nothrow) __attribute__((const)) double nan(const char * );
extern __declspec(__nothrow) __attribute__((const)) float nanf(const char * );

inline __declspec(__nothrow) __attribute__((const)) long double nanl(const char *__t)     { return (long double)nan(__t); }
#line 856 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"
extern __declspec(__nothrow) __attribute__((const)) double nearbyint(double  );
extern __declspec(__nothrow) __attribute__((const)) float nearbyintf(float  );
__declspec(__nothrow) long double nearbyintl(long double );
extern  double remquo(double  , double  , int * );
extern  float remquof(float  , float  , int * );

inline long double remquol(long double __x, long double __y, int *__q)     { return (long double)remquo((double)__x, (double)__y, __q); }


extern __declspec(__nothrow) __attribute__((const)) double round(double  );
extern __declspec(__nothrow) __attribute__((const)) float roundf(float  );
__declspec(__nothrow) long double roundl(long double );
extern __declspec(__nothrow) double tgamma(double  );  
extern __declspec(__nothrow) float tgammaf(float  );
__declspec(__nothrow) long double tgammal(long double );
extern __declspec(__nothrow) __attribute__((const)) double trunc(double  );
extern __declspec(__nothrow) __attribute__((const)) float truncf(float  );
__declspec(__nothrow) long double truncl(long double );






#line 896 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"

#line 1087 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"











#line 1317 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\math.h"





 
#line 49 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"




 




 














 

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef union{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} axis3bit32_t;

typedef union{
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;




 

typedef struct{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;









 







 

typedef int32_t (*lsm6dsl_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lsm6dsl_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
   
  lsm6dsl_write_ptr  write_reg;
  lsm6dsl_read_ptr   read_reg;
   
  void *handle;
} lsm6dsl_ctx_t;




 




 

 



 





 


typedef struct {
  uint8_t not_used_01              : 5;
  uint8_t func_cfg_en              : 3;   
} lsm6dsl_func_cfg_access_t;


typedef struct {
  uint8_t tph                      : 4;
  uint8_t not_used_01              : 4;
} lsm6dsl_sensor_sync_time_frame_t;


typedef struct {
  uint8_t rr                       : 2;
  uint8_t not_used_01              : 6;
} lsm6dsl_sensor_sync_res_ratio_t;


typedef struct {
  uint8_t fth                      : 8;   
} lsm6dsl_fifo_ctrl1_t;


typedef struct {
  uint8_t fth                      : 3;   
  uint8_t fifo_temp_en             : 1;
  uint8_t not_used_01              : 2;
  uint8_t  timer_pedo_fifo_drdy    : 1;
  uint8_t timer_pedo_fifo_en       : 1;
} lsm6dsl_fifo_ctrl2_t;


typedef struct {
  uint8_t dec_fifo_xl              : 3;
  uint8_t dec_fifo_gyro            : 3;
  uint8_t not_used_01              : 2;
} lsm6dsl_fifo_ctrl3_t;


typedef struct {
  uint8_t dec_ds3_fifo             : 3;
  uint8_t dec_ds4_fifo             : 3;
  uint8_t only_high_data           : 1;
  uint8_t stop_on_fth              : 1;
} lsm6dsl_fifo_ctrl4_t;


typedef struct {
  uint8_t fifo_mode                : 3;
  uint8_t odr_fifo                 : 4;
  uint8_t not_used_01              : 1;
} lsm6dsl_fifo_ctrl5_t;


typedef struct {
  uint8_t int2_wrist_tilt          : 1;
  uint8_t not_used_01              : 6;
  uint8_t drdy_pulsed              : 1;
} lsm6dsl_drdy_pulse_cfg_g_t;


typedef struct {
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_full_flag           : 1;
  uint8_t int1_sign_mot            : 1;
  uint8_t int1_step_detector       : 1;
} lsm6dsl_int1_ctrl_t;


typedef struct {
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_full_flag           : 1;
  uint8_t int2_step_count_ov       : 1;
  uint8_t int2_step_delta          : 1;
} lsm6dsl_int2_ctrl_t;



typedef struct {
  uint8_t bw0_xl                   : 1;
  uint8_t lpf1_bw_sel              : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
} lsm6dsl_ctrl1_xl_t;


typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 3;   
  uint8_t odr_g                    : 4;
} lsm6dsl_ctrl2_g_t;


typedef struct {
  uint8_t sw_reset                 : 1;
  uint8_t ble                      : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
} lsm6dsl_ctrl3_c_t;


typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t den_drdy_int1            : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t sleep                    : 1;
  uint8_t den_xl_en                : 1;
} lsm6dsl_ctrl4_c_t;


typedef struct {
  uint8_t st_xl                    : 2;
  uint8_t st_g                     : 2;
  uint8_t den_lh                   : 1;
  uint8_t rounding                 : 3;
} lsm6dsl_ctrl5_c_t;


typedef struct {
  uint8_t ftype                    : 2;
  uint8_t not_used_01              : 1;
  uint8_t usr_off_w                : 1;
  uint8_t xl_hm_mode               : 1;
  uint8_t den_mode                 : 3;   
} lsm6dsl_ctrl6_c_t;


typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t rounding_status          : 1;
  uint8_t not_used_02              : 1;
  uint8_t hpm_g                    : 2;
  uint8_t hp_en_g                  : 1;
  uint8_t g_hm_mode                : 1;
} lsm6dsl_ctrl7_g_t;


typedef struct {
  uint8_t low_pass_on_6d           : 1;
  uint8_t not_used_01              : 1;
  uint8_t hp_slope_xl_en           : 1;
  uint8_t input_composite          : 1;
  uint8_t hp_ref_mode              : 1;
  uint8_t hpcf_xl                  : 2;
  uint8_t lpf2_xl_en               : 1;
} lsm6dsl_ctrl8_xl_t;


typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t soft_en                  : 1;
  uint8_t not_used_02              : 1;
  uint8_t den_xl_g                 : 1;
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
} lsm6dsl_ctrl9_xl_t;


typedef struct {
  uint8_t sign_motion_en           : 1;
  uint8_t pedo_rst_step            : 1;
  uint8_t func_en                  : 1;
  uint8_t tilt_en                  : 1;
  uint8_t pedo_en                  : 1;
  uint8_t timer_en                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t wrist_tilt_en            : 1;
} lsm6dsl_ctrl10_c_t;


typedef struct {
  uint8_t master_on                : 1;
  uint8_t iron_en                  : 1;
  uint8_t pass_through_mode        : 1;
  uint8_t pull_up_en               : 1;
  uint8_t start_config             : 1;
  uint8_t not_used_01              : 1;
  uint8_t  data_valid_sel_fifo     : 1;
  uint8_t drdy_on_int1             : 1;
} lsm6dsl_master_config_t;


typedef struct {
  uint8_t z_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_state_ia           : 1;
  uint8_t ff_ia                    : 1;
  uint8_t not_used_01              : 2;
} lsm6dsl_wake_up_src_t;


typedef struct {
  uint8_t z_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t tap_ia                   : 1;
  uint8_t not_used_01              : 1;
} lsm6dsl_tap_src_t;


typedef struct {
  uint8_t xl                       : 1;
  uint8_t xh                       : 1;
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t den_drdy                 : 1;
} lsm6dsl_d6d_src_t;


typedef struct {
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 5;
} lsm6dsl_status_reg_t;

#line 418 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"
typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub1_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub2_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub3_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub4_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub5_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub6_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub7_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub8_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub9_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub10_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub11_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub12_reg_t;


typedef struct {
  uint8_t diff_fifo                : 8;   
} lsm6dsl_fifo_status1_t;


typedef struct {
  uint8_t diff_fifo                : 3;   
  uint8_t not_used_01              : 1;
  uint8_t fifo_empty               : 1;
  uint8_t fifo_full_smart          : 1;
  uint8_t over_run                 : 1;
  uint8_t waterm                   : 1;
} lsm6dsl_fifo_status2_t;


typedef struct {
  uint8_t fifo_pattern             : 8;   
} lsm6dsl_fifo_status3_t;


typedef struct {
  uint8_t fifo_pattern             : 2;   
  uint8_t not_used_01              : 6;
} lsm6dsl_fifo_status4_t;

#line 596 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub13_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub14_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub15_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub16_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub17_reg_t;


typedef struct {
  uint8_t bit0                     : 1;
  uint8_t bit1                     : 1;
  uint8_t bit2                     : 1;
  uint8_t bit3                     : 1;
  uint8_t bit4                     : 1;
  uint8_t bit5                     : 1;
  uint8_t bit6                     : 1;
  uint8_t bit7                     : 1;
} lsm6dsl_sensorhub18_reg_t;


typedef struct {
  uint8_t sensorhub_end_op         : 1;
  uint8_t si_end_op                : 1;
  uint8_t hi_fail                  : 1;
  uint8_t step_overflow            : 1;
  uint8_t step_detected            : 1;
  uint8_t tilt_ia                  : 1;
  uint8_t sign_motion_ia           : 1;
  uint8_t  step_count_delta_ia     : 1;
} lsm6dsl_func_src1_t;


typedef struct {
  uint8_t wrist_tilt_ia            : 1;
  uint8_t not_used_01              : 2;
  uint8_t slave0_nack              : 1;
  uint8_t slave1_nack              : 1;
  uint8_t slave2_nack              : 1;
  uint8_t slave3_nack              : 1;
  uint8_t not_used_02              : 1;
} lsm6dsl_func_src2_t;


typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t wrist_tilt_ia_zneg       : 1;
  uint8_t wrist_tilt_ia_zpos       : 1;
  uint8_t wrist_tilt_ia_yneg       : 1;
  uint8_t wrist_tilt_ia_ypos       : 1;
  uint8_t wrist_tilt_ia_xneg       : 1;
  uint8_t wrist_tilt_ia_xpos       : 1;
} lsm6dsl_wrist_tilt_ia_t;


typedef struct {
  uint8_t lir                      : 1;
  uint8_t tap_z_en                 : 1;
  uint8_t tap_y_en                 : 1;
  uint8_t tap_x_en                 : 1;
  uint8_t slope_fds                : 1;
  uint8_t inact_en                 : 2;
  uint8_t interrupts_enable        : 1;
} lsm6dsl_tap_cfg_t;


typedef struct {
  uint8_t tap_ths                  : 5;
  uint8_t sixd_ths                 : 2;
  uint8_t d4d_en                   : 1;
} lsm6dsl_tap_ths_6d_t;


typedef struct {
  uint8_t shock                    : 2;
  uint8_t quiet                    : 2;
  uint8_t dur                      : 4;
} lsm6dsl_int_dur2_t;


typedef struct {
  uint8_t wk_ths                   : 6;
  uint8_t not_used_01              : 1;
  uint8_t single_double_tap        : 1;
} lsm6dsl_wake_up_ths_t;


typedef struct {
  uint8_t sleep_dur                : 4;
  uint8_t timer_hr                 : 1;
  uint8_t wake_dur                 : 2;
  uint8_t ff_dur                   : 1;
} lsm6dsl_wake_up_dur_t;


typedef struct {
  uint8_t ff_ths                   : 3;
  uint8_t ff_dur                   : 5;
} lsm6dsl_free_fall_t;


typedef struct {
  uint8_t int1_timer               : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_inact_state         : 1;
} lsm6dsl_md1_cfg_t;


typedef struct {
  uint8_t int2_iron                : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_inact_state         : 1;
} lsm6dsl_md2_cfg_t;


typedef struct {
  uint8_t master_cmd_code          : 8;
} lsm6dsl_master_cmd_code_t;


typedef struct {
  uint8_t error_code               : 8;
} lsm6dsl_sens_sync_spi_error_code_t;

#line 793 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"
typedef struct {
  uint8_t rw_0                     : 1;
  uint8_t slave0_add               : 7;
} lsm6dsl_slv0_add_t;


typedef struct {
  uint8_t slave0_reg               : 8;
} lsm6dsl_slv0_subadd_t;


typedef struct {
  uint8_t slave0_numop             : 3;
  uint8_t src_mode                 : 1;
  uint8_t aux_sens_on              : 2;
  uint8_t slave0_rate              : 2;
} lsm6dsl_slave0_config_t;


typedef struct {
  uint8_t r_1                      : 1;
  uint8_t slave1_add               : 7;
} lsm6dsl_slv1_add_t;


typedef struct {
  uint8_t slave1_reg               : 8;
} lsm6dsl_slv1_subadd_t;


typedef struct {
  uint8_t slave1_numop             : 3;
  uint8_t not_used_01              : 2;
  uint8_t write_once               : 1;
  uint8_t slave1_rate              : 2;
} lsm6dsl_slave1_config_t;


typedef struct {
  uint8_t r_2                      : 1;
  uint8_t slave2_add               : 7;
} lsm6dsl_slv2_add_t;


typedef struct {
  uint8_t slave2_reg               : 8;
} lsm6dsl_slv2_subadd_t;


typedef struct {
  uint8_t slave2_numop             : 3;
  uint8_t not_used_01              : 3;
  uint8_t slave2_rate              : 2;
} lsm6dsl_slave2_config_t;


typedef struct {
  uint8_t r_3                      : 1;
  uint8_t slave3_add               : 7;
} lsm6dsl_slv3_add_t;


typedef struct {
  uint8_t slave3_reg               : 8;
} lsm6dsl_slv3_subadd_t;


typedef struct {
  uint8_t slave3_numop             : 3;
  uint8_t not_used_01              : 3;
  uint8_t slave3_rate              : 2;
} lsm6dsl_slave3_config_t;


typedef struct {
  uint8_t slave_dataw              : 8;
} lsm6dsl_datawrite_src_mode_sub_slv0_t;


typedef struct {
  uint8_t ths_min                  : 5;
  uint8_t not_used_01              : 2;
  uint8_t pedo_fs                  : 1;
} lsm6dsl_config_pedo_ths_min_t;



typedef struct {
  uint8_t deb_step      : 3;
  uint8_t deb_time      : 5;
} lsm6dsl_pedo_deb_reg_t;

#line 904 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.h"
typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t  wrist_tilt_mask_zneg    : 1;
  uint8_t  wrist_tilt_mask_zpos    : 1;
  uint8_t  wrist_tilt_mask_yneg    : 1;
  uint8_t  wrist_tilt_mask_ypos    : 1;
  uint8_t  wrist_tilt_mask_xneg    : 1;
  uint8_t  wrist_tilt_mask_xpos    : 1;
} lsm6dsl_a_wrist_tilt_mask_t;












 
typedef union{
  lsm6dsl_func_cfg_access_t                  func_cfg_access;
  lsm6dsl_sensor_sync_time_frame_t           sensor_sync_time_frame;
  lsm6dsl_sensor_sync_res_ratio_t            sensor_sync_res_ratio;
  lsm6dsl_fifo_ctrl1_t                       fifo_ctrl1;
  lsm6dsl_fifo_ctrl2_t                       fifo_ctrl2;
  lsm6dsl_fifo_ctrl3_t                       fifo_ctrl3;
  lsm6dsl_fifo_ctrl4_t                       fifo_ctrl4;
  lsm6dsl_fifo_ctrl5_t                       fifo_ctrl5;
  lsm6dsl_drdy_pulse_cfg_g_t                 drdy_pulse_cfg_g;
  lsm6dsl_int1_ctrl_t                        int1_ctrl;
  lsm6dsl_int2_ctrl_t                        int2_ctrl;
  lsm6dsl_ctrl1_xl_t                         ctrl1_xl;
  lsm6dsl_ctrl2_g_t                          ctrl2_g;
  lsm6dsl_ctrl3_c_t                          ctrl3_c;
  lsm6dsl_ctrl4_c_t                          ctrl4_c;
  lsm6dsl_ctrl5_c_t                          ctrl5_c;
  lsm6dsl_ctrl6_c_t                          ctrl6_c;
  lsm6dsl_ctrl7_g_t                          ctrl7_g;
  lsm6dsl_ctrl8_xl_t                         ctrl8_xl;
  lsm6dsl_ctrl9_xl_t                         ctrl9_xl;
  lsm6dsl_ctrl10_c_t                         ctrl10_c;
  lsm6dsl_master_config_t                    master_config;
  lsm6dsl_wake_up_src_t                      wake_up_src;
  lsm6dsl_tap_src_t                          tap_src;
  lsm6dsl_d6d_src_t                          d6d_src;
  lsm6dsl_status_reg_t                       status_reg;
  lsm6dsl_sensorhub1_reg_t                   sensorhub1_reg;
  lsm6dsl_sensorhub2_reg_t                   sensorhub2_reg;
  lsm6dsl_sensorhub3_reg_t                   sensorhub3_reg;
  lsm6dsl_sensorhub4_reg_t                   sensorhub4_reg;
  lsm6dsl_sensorhub5_reg_t                   sensorhub5_reg;
  lsm6dsl_sensorhub6_reg_t                   sensorhub6_reg;
  lsm6dsl_sensorhub7_reg_t                   sensorhub7_reg;
  lsm6dsl_sensorhub8_reg_t                   sensorhub8_reg;
  lsm6dsl_sensorhub9_reg_t                   sensorhub9_reg;
  lsm6dsl_sensorhub10_reg_t                  sensorhub10_reg;
  lsm6dsl_sensorhub11_reg_t                  sensorhub11_reg;
  lsm6dsl_sensorhub12_reg_t                  sensorhub12_reg;
  lsm6dsl_fifo_status1_t                     fifo_status1;
  lsm6dsl_fifo_status2_t                     fifo_status2;
  lsm6dsl_fifo_status3_t                     fifo_status3;
  lsm6dsl_fifo_status4_t                     fifo_status4;
  lsm6dsl_sensorhub13_reg_t                  sensorhub13_reg;
  lsm6dsl_sensorhub14_reg_t                  sensorhub14_reg;
  lsm6dsl_sensorhub15_reg_t                  sensorhub15_reg;
  lsm6dsl_sensorhub16_reg_t                  sensorhub16_reg;
  lsm6dsl_sensorhub17_reg_t                  sensorhub17_reg;
  lsm6dsl_sensorhub18_reg_t                  sensorhub18_reg;
  lsm6dsl_func_src1_t                        func_src1;
  lsm6dsl_func_src2_t                        func_src2;
  lsm6dsl_wrist_tilt_ia_t                    wrist_tilt_ia;
  lsm6dsl_tap_cfg_t                          tap_cfg;
  lsm6dsl_tap_ths_6d_t                       tap_ths_6d;
  lsm6dsl_int_dur2_t                         int_dur2;
  lsm6dsl_wake_up_ths_t                      wake_up_ths;
  lsm6dsl_wake_up_dur_t                      wake_up_dur;
  lsm6dsl_free_fall_t                        free_fall;
  lsm6dsl_md1_cfg_t                          md1_cfg;
  lsm6dsl_md2_cfg_t                          md2_cfg;
  lsm6dsl_master_cmd_code_t                  master_cmd_code;
  lsm6dsl_sens_sync_spi_error_code_t         sens_sync_spi_error_code;
  lsm6dsl_slv0_add_t                         slv0_add;
  lsm6dsl_slv0_subadd_t                      slv0_subadd;
  lsm6dsl_slave0_config_t                    slave0_config;
  lsm6dsl_slv1_add_t                         slv1_add;
  lsm6dsl_slv1_subadd_t                      slv1_subadd;
  lsm6dsl_slave1_config_t                    slave1_config;
  lsm6dsl_slv2_add_t                         slv2_add;
  lsm6dsl_slv2_subadd_t                      slv2_subadd;
  lsm6dsl_slave2_config_t                    slave2_config;
  lsm6dsl_slv3_add_t                         slv3_add;
  lsm6dsl_slv3_subadd_t                      slv3_subadd;
  lsm6dsl_slave3_config_t                    slave3_config;
  lsm6dsl_datawrite_src_mode_sub_slv0_t      datawrite_src_mode_sub_slv0;
  lsm6dsl_config_pedo_ths_min_t              config_pedo_ths_min;
  lsm6dsl_pedo_deb_reg_t                     pedo_deb_reg;
  lsm6dsl_a_wrist_tilt_mask_t                a_wrist_tilt_mask;
  bitwise_t                                  bitwise;
  uint8_t                                    byte;
} lsm6dsl_reg_t;




 

int32_t lsm6dsl_read_reg(lsm6dsl_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lsm6dsl_write_reg(lsm6dsl_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t lsm6dsl_from_fs2g_to_mg(int16_t lsb);
extern float_t lsm6dsl_from_fs4g_to_mg(int16_t lsb);
extern float_t lsm6dsl_from_fs8g_to_mg(int16_t lsb);
extern float_t lsm6dsl_from_fs16g_to_mg(int16_t lsb);

extern float_t lsm6dsl_from_fs125dps_to_mdps(int16_t lsb);
extern float_t lsm6dsl_from_fs250dps_to_mdps(int16_t lsb);
extern float_t lsm6dsl_from_fs500dps_to_mdps(int16_t lsb);
extern float_t lsm6dsl_from_fs1000dps_to_mdps(int16_t lsb);
extern float_t lsm6dsl_from_fs2000dps_to_mdps(int16_t lsb);

extern float_t lsm6dsl_from_lsb_to_celsius(int16_t lsb);

typedef enum {
  LSM6DSL_2g       = 0,
  LSM6DSL_16g      = 1,
  LSM6DSL_4g       = 2,
  LSM6DSL_8g       = 3,
  LSM6DSL_XL_FS_ND = 4,   
} lsm6dsl_fs_xl_t;
int32_t lsm6dsl_xl_full_scale_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_xl_t val);
int32_t lsm6dsl_xl_full_scale_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_xl_t *val);

typedef enum {
  LSM6DSL_XL_ODR_OFF      =  0,
  LSM6DSL_XL_ODR_12Hz5    =  1,
  LSM6DSL_XL_ODR_26Hz     =  2,
  LSM6DSL_XL_ODR_52Hz     =  3,
  LSM6DSL_XL_ODR_104Hz    =  4,
  LSM6DSL_XL_ODR_208Hz    =  5,
  LSM6DSL_XL_ODR_416Hz    =  6,
  LSM6DSL_XL_ODR_833Hz    =  7,
  LSM6DSL_XL_ODR_1k66Hz   =  8,
  LSM6DSL_XL_ODR_3k33Hz   =  9,
  LSM6DSL_XL_ODR_6k66Hz   = 10,
  LSM6DSL_XL_ODR_1Hz6     = 11,
  LSM6DSL_XL_ODR_ND       = 12,   
} lsm6dsl_odr_xl_t;
int32_t lsm6dsl_xl_data_rate_set(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_xl_t val);
int32_t lsm6dsl_xl_data_rate_get(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_xl_t *val);

typedef enum {
  LSM6DSL_250dps     = 0,
  LSM6DSL_125dps     = 1,
  LSM6DSL_500dps     = 2,
  LSM6DSL_1000dps    = 4,
  LSM6DSL_2000dps    = 6,
  LSM6DSL_GY_FS_ND   = 7,     
} lsm6dsl_fs_g_t;
int32_t lsm6dsl_gy_full_scale_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_g_t val);
int32_t lsm6dsl_gy_full_scale_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_g_t *val);

typedef enum {
  LSM6DSL_GY_ODR_OFF    =  0,
  LSM6DSL_GY_ODR_12Hz5  =  1,
  LSM6DSL_GY_ODR_26Hz   =  2,
  LSM6DSL_GY_ODR_52Hz   =  3,
  LSM6DSL_GY_ODR_104Hz  =  4,
  LSM6DSL_GY_ODR_208Hz  =  5,
  LSM6DSL_GY_ODR_416Hz  =  6,
  LSM6DSL_GY_ODR_833Hz  =  7,
  LSM6DSL_GY_ODR_1k66Hz =  8,
  LSM6DSL_GY_ODR_3k33Hz =  9,
  LSM6DSL_GY_ODR_6k66Hz = 10,
  LSM6DSL_GY_ODR_ND     = 11,     
} lsm6dsl_odr_g_t;
int32_t lsm6dsl_gy_data_rate_set(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_g_t val);
int32_t lsm6dsl_gy_data_rate_get(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_g_t *val);

int32_t lsm6dsl_block_data_update_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_block_data_update_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_LSb_1mg   = 0,
  LSM6DSL_LSb_16mg  = 1,
  LSM6DSL_WEIGHT_ND = 2,
} lsm6dsl_usr_off_w_t;
int32_t lsm6dsl_xl_offset_weight_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_usr_off_w_t val);
int32_t lsm6dsl_xl_offset_weight_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_usr_off_w_t *val);

typedef enum {
  LSM6DSL_XL_HIGH_PERFORMANCE  = 0,
  LSM6DSL_XL_NORMAL            = 1,
  LSM6DSL_XL_PW_MODE_ND        = 2,     
} lsm6dsl_xl_hm_mode_t;
int32_t lsm6dsl_xl_power_mode_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_xl_hm_mode_t val);
int32_t lsm6dsl_xl_power_mode_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_xl_hm_mode_t *val);

typedef enum {
  LSM6DSL_STAT_RND_DISABLE  = 0,
  LSM6DSL_STAT_RND_ENABLE   = 1,
  LSM6DSL_STAT_RND_ND       = 2,     
} lsm6dsl_rounding_status_t;
int32_t lsm6dsl_rounding_on_status_set(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_rounding_status_t val);
int32_t lsm6dsl_rounding_on_status_get(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_rounding_status_t *val);

typedef enum {
  LSM6DSL_GY_HIGH_PERFORMANCE  = 0,
  LSM6DSL_GY_NORMAL            = 1,
  LSM6DSL_GY_PW_MODE_ND        = 2,     
} lsm6dsl_g_hm_mode_t;
int32_t lsm6dsl_gy_power_mode_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_g_hm_mode_t val);
int32_t lsm6dsl_gy_power_mode_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_g_hm_mode_t *val);

typedef struct {
  lsm6dsl_wake_up_src_t        wake_up_src;
  lsm6dsl_tap_src_t            tap_src;
  lsm6dsl_d6d_src_t            d6d_src;
  lsm6dsl_status_reg_t         status_reg;
  lsm6dsl_func_src1_t          func_src1;
  lsm6dsl_func_src2_t          func_src2;
  lsm6dsl_wrist_tilt_ia_t      wrist_tilt_ia;
  lsm6dsl_a_wrist_tilt_mask_t  a_wrist_tilt_mask;
} lsm6dsl_all_sources_t;
int32_t lsm6dsl_all_sources_get(lsm6dsl_ctx_t *ctx,
                                lsm6dsl_all_sources_t *val);

int32_t lsm6dsl_status_reg_get(lsm6dsl_ctx_t *ctx, lsm6dsl_status_reg_t *val);

int32_t lsm6dsl_xl_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_gy_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_temp_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_xl_usr_offset_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_xl_usr_offset_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_timestamp_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_timestamp_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_LSB_6ms4    = 0,
  LSM6DSL_LSB_25us    = 1,
  LSM6DSL_TS_RES_ND   = 2,     
} lsm6dsl_timer_hr_t;
int32_t lsm6dsl_timestamp_res_set(lsm6dsl_ctx_t *ctx, lsm6dsl_timer_hr_t val);
int32_t lsm6dsl_timestamp_res_get(lsm6dsl_ctx_t *ctx, lsm6dsl_timer_hr_t *val);

typedef enum {
  LSM6DSL_ROUND_DISABLE            = 0,
  LSM6DSL_ROUND_XL                 = 1,
  LSM6DSL_ROUND_GY                 = 2,
  LSM6DSL_ROUND_GY_XL              = 3,
  LSM6DSL_ROUND_SH1_TO_SH6         = 4,
  LSM6DSL_ROUND_XL_SH1_TO_SH6      = 5,
  LSM6DSL_ROUND_GY_XL_SH1_TO_SH12  = 6,
  LSM6DSL_ROUND_GY_XL_SH1_TO_SH6   = 7,
  LSM6DSL_ROUND_OUT_ND             = 8,     
} lsm6dsl_rounding_t;
int32_t lsm6dsl_rounding_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_rounding_t val);
int32_t lsm6dsl_rounding_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_rounding_t *val);

int32_t lsm6dsl_temperature_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_angular_rate_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_acceleration_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_mag_calibrated_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_fifo_raw_data_get(lsm6dsl_ctx_t *ctx, uint8_t *buffer,
                                  uint8_t len);

typedef enum {
  LSM6DSL_USER_BANK   = 0,
  LSM6DSL_BANK_A      = 4,
  LSM6DSL_BANK_B      = 5,
  LSM6DSL_BANK_ND     = 6,     
} lsm6dsl_func_cfg_en_t;
int32_t lsm6dsl_mem_bank_set(lsm6dsl_ctx_t *ctx, lsm6dsl_func_cfg_en_t val);
int32_t lsm6dsl_mem_bank_get(lsm6dsl_ctx_t *ctx, lsm6dsl_func_cfg_en_t *val);

typedef enum {
  LSM6DSL_DRDY_LATCHED    = 0,
  LSM6DSL_DRDY_PULSED     = 1,
  LSM6DSL_DRDY_ND         = 2,   
} lsm6dsl_drdy_pulsed_g_t;
int32_t lsm6dsl_data_ready_mode_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_drdy_pulsed_g_t val);
int32_t lsm6dsl_data_ready_mode_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_drdy_pulsed_g_t *val);

int32_t lsm6dsl_device_id_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_reset_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_reset_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_LSB_AT_LOW_ADD  = 0,
  LSM6DSL_MSB_AT_LOW_ADD  = 1,
  LSM6DSL_DATA_FMT_ND     = 2,     
} lsm6dsl_ble_t;
int32_t lsm6dsl_data_format_set(lsm6dsl_ctx_t *ctx, lsm6dsl_ble_t val);
int32_t lsm6dsl_data_format_get(lsm6dsl_ctx_t *ctx, lsm6dsl_ble_t *val);

int32_t lsm6dsl_auto_increment_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_auto_increment_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_boot_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_boot_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_XL_ST_DISABLE    = 0,
  LSM6DSL_XL_ST_POSITIVE   = 1,
  LSM6DSL_XL_ST_NEGATIVE   = 2,
  LSM6DSL_XL_ST_ND         = 3,     
} lsm6dsl_st_xl_t;
int32_t lsm6dsl_xl_self_test_set(lsm6dsl_ctx_t *ctx, lsm6dsl_st_xl_t val);
int32_t lsm6dsl_xl_self_test_get(lsm6dsl_ctx_t *ctx, lsm6dsl_st_xl_t *val);

typedef enum {
  LSM6DSL_GY_ST_DISABLE    = 0,
  LSM6DSL_GY_ST_POSITIVE   = 1,
  LSM6DSL_GY_ST_NEGATIVE   = 3,
  LSM6DSL_GY_ST_ND         = 4,     
} lsm6dsl_st_g_t;
int32_t lsm6dsl_gy_self_test_set(lsm6dsl_ctx_t *ctx, lsm6dsl_st_g_t val);
int32_t lsm6dsl_gy_self_test_get(lsm6dsl_ctx_t *ctx, lsm6dsl_st_g_t *val);

int32_t lsm6dsl_filter_settling_mask_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_filter_settling_mask_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_USE_SLOPE    = 0,
  LSM6DSL_USE_HPF      = 1,
  LSM6DSL_HP_PATH_ND   = 2,     
} lsm6dsl_slope_fds_t;
int32_t lsm6dsl_xl_hp_path_internal_set(lsm6dsl_ctx_t *ctx,
                                        lsm6dsl_slope_fds_t val);
int32_t lsm6dsl_xl_hp_path_internal_get(lsm6dsl_ctx_t *ctx,
                                        lsm6dsl_slope_fds_t *val);

typedef enum {
  LSM6DSL_XL_ANA_BW_1k5Hz = 0,
  LSM6DSL_XL_ANA_BW_400Hz = 1,
  LSM6DSL_XL_ANA_BW_ND    = 2,     
} lsm6dsl_bw0_xl_t;
int32_t lsm6dsl_xl_filter_analog_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_bw0_xl_t val);
int32_t lsm6dsl_xl_filter_analog_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_bw0_xl_t *val);

typedef enum {
  LSM6DSL_XL_LP1_ODR_DIV_2 = 0,
  LSM6DSL_XL_LP1_ODR_DIV_4 = 1,
  LSM6DSL_XL_LP1_NA        = 2,   
} lsm6dsl_lpf1_bw_sel_t;
int32_t lsm6dsl_xl_lp1_bandwidth_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_lpf1_bw_sel_t val);
int32_t lsm6dsl_xl_lp1_bandwidth_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_lpf1_bw_sel_t *val);

typedef enum {
  LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_50     = 0x00,
  LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_100    = 0x01,
  LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9      = 0x02,
  LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_400    = 0x03,
  LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_50   = 0x10,
  LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100  = 0x11,
  LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_9    = 0x12,
  LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_400  = 0x13,
  LSM6DSL_XL_LP_NA                     = 0x20,  
} lsm6dsl_input_composite_t;
int32_t lsm6dsl_xl_lp2_bandwidth_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_input_composite_t val);
int32_t lsm6dsl_xl_lp2_bandwidth_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_input_composite_t *val);

int32_t lsm6dsl_xl_reference_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_xl_reference_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_XL_HP_ODR_DIV_4      = 0x00,  
  LSM6DSL_XL_HP_ODR_DIV_100    = 0x01,
  LSM6DSL_XL_HP_ODR_DIV_9      = 0x02,
  LSM6DSL_XL_HP_ODR_DIV_400    = 0x03,
  LSM6DSL_XL_HP_NA             = 0x10,  
} lsm6dsl_hpcf_xl_t;
int32_t lsm6dsl_xl_hp_bandwidth_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_hpcf_xl_t val);
int32_t lsm6dsl_xl_hp_bandwidth_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_hpcf_xl_t *val);

typedef enum {
  LSM6DSL_LP2_ONLY                    = 0x00,

  LSM6DSL_HP_16mHz_LP2                = 0x80,
  LSM6DSL_HP_65mHz_LP2                = 0x90,
  LSM6DSL_HP_260mHz_LP2               = 0xA0,
  LSM6DSL_HP_1Hz04_LP2                = 0xB0,

  LSM6DSL_HP_DISABLE_LP1_LIGHT        = 0x0A,
  LSM6DSL_HP_DISABLE_LP1_NORMAL       = 0x09,
  LSM6DSL_HP_DISABLE_LP_STRONG        = 0x08,
  LSM6DSL_HP_DISABLE_LP1_AGGRESSIVE   = 0x0B,

  LSM6DSL_HP_16mHz_LP1_LIGHT          = 0x8A,
  LSM6DSL_HP_65mHz_LP1_NORMAL         = 0x99,
  LSM6DSL_HP_260mHz_LP1_STRONG        = 0xA8,
  LSM6DSL_HP_1Hz04_LP1_AGGRESSIVE     = 0xBB,

  LSM6DSL_HP_GY_BAND_NA               = 0xFF,     
} lsm6dsl_lpf1_sel_g_t;
int32_t lsm6dsl_gy_band_pass_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_lpf1_sel_g_t val);
int32_t lsm6dsl_gy_band_pass_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_lpf1_sel_g_t *val);

typedef enum {
  LSM6DSL_SPI_4_WIRE  = 0,
  LSM6DSL_SPI_3_WIRE  = 1,
  LSM6DSL_SPI_MODE_ND = 2,     
} lsm6dsl_sim_t;
int32_t lsm6dsl_spi_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_sim_t val);
int32_t lsm6dsl_spi_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_sim_t *val);

typedef enum {
  LSM6DSL_I2C_ENABLE   = 0,
  LSM6DSL_I2C_DISABLE  = 1,
  LSM6DSL_I2C_MODE_ND  = 2,     
} lsm6dsl_i2c_disable_t;
int32_t lsm6dsl_i2c_interface_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_i2c_disable_t val);
int32_t lsm6dsl_i2c_interface_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_i2c_disable_t *val);

typedef struct {
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fth                 : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_full_flag           : 1;
  uint8_t int1_sign_mot            : 1;
  uint8_t int1_step_detector       : 1;
  uint8_t int1_timer               : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_inact_state         : 1;
  uint8_t den_drdy_int1            : 1;
  uint8_t drdy_on_int1             : 1;
} lsm6dsl_int1_route_t;
int32_t lsm6dsl_pin_int1_route_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int1_route_t val);
int32_t lsm6dsl_pin_int1_route_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int1_route_t *val);

typedef struct{
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fth                 : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_full_flag           : 1;
  uint8_t int2_step_count_ov       : 1;
  uint8_t int2_step_delta          : 1;
  uint8_t int2_iron                : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_inact_state         : 1;
  uint8_t int2_wrist_tilt          : 1;
} lsm6dsl_int2_route_t;
int32_t lsm6dsl_pin_int2_route_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int2_route_t val);
int32_t lsm6dsl_pin_int2_route_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int2_route_t *val);

typedef enum {
  LSM6DSL_PUSH_PULL   = 0,
  LSM6DSL_OPEN_DRAIN  = 1,
  LSM6DSL_PIN_MODE_ND = 2,     
} lsm6dsl_pp_od_t;
int32_t lsm6dsl_pin_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_pp_od_t val);
int32_t lsm6dsl_pin_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_pp_od_t *val);

typedef enum {
  LSM6DSL_ACTIVE_HIGH   = 0,
  LSM6DSL_ACTIVE_LOW    = 1,
  LSM6DSL_POLARITY_ND   = 2,     
} lsm6dsl_h_lactive_t;
int32_t lsm6dsl_pin_polarity_set(lsm6dsl_ctx_t *ctx, lsm6dsl_h_lactive_t val);
int32_t lsm6dsl_pin_polarity_get(lsm6dsl_ctx_t *ctx, lsm6dsl_h_lactive_t *val);

int32_t lsm6dsl_all_on_int1_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_all_on_int1_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_INT_PULSED   = 0,
  LSM6DSL_INT_LATCHED  = 1,
  LSM6DSL_INT_MODE     = 2,     
} lsm6dsl_lir_t;
int32_t lsm6dsl_int_notification_set(lsm6dsl_ctx_t *ctx, lsm6dsl_lir_t val);
int32_t lsm6dsl_int_notification_get(lsm6dsl_ctx_t *ctx, lsm6dsl_lir_t *val);

int32_t lsm6dsl_wkup_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_wkup_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_wkup_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_wkup_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_gy_sleep_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_gy_sleep_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_PROPERTY_DISABLE          = 0,
  LSM6DSL_XL_12Hz5_GY_NOT_AFFECTED  = 1,
  LSM6DSL_XL_12Hz5_GY_SLEEP         = 2,
  LSM6DSL_XL_12Hz5_GY_PD            = 3,
  LSM6DSL_ACT_MODE_ND               = 4,     
} lsm6dsl_inact_en_t;
int32_t lsm6dsl_act_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_inact_en_t val);
int32_t lsm6dsl_act_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_inact_en_t *val);

int32_t lsm6dsl_act_sleep_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_act_sleep_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_src_get(lsm6dsl_ctx_t *ctx, lsm6dsl_tap_src_t *val);

int32_t lsm6dsl_tap_detection_on_z_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_detection_on_z_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_detection_on_y_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_detection_on_y_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_detection_on_x_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_detection_on_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_threshold_x_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_threshold_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_shock_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_shock_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_quiet_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_quiet_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tap_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tap_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_ONLY_SINGLE          = 0,
  LSM6DSL_BOTH_SINGLE_DOUBLE   = 1,
  LSM6DSL_TAP_MODE_ND          = 2,     
} lsm6dsl_single_double_tap_t;
int32_t lsm6dsl_tap_mode_set(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_single_double_tap_t val);
int32_t lsm6dsl_tap_mode_get(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_single_double_tap_t *val);

typedef enum {
  LSM6DSL_ODR_DIV_2_FEED      = 0,
  LSM6DSL_LPF2_FEED           = 1,
  LSM6DSL_6D_FEED_ND          = 2,     
} lsm6dsl_low_pass_on_6d_t;
int32_t lsm6dsl_6d_feed_data_set(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_low_pass_on_6d_t val);
int32_t lsm6dsl_6d_feed_data_get(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_low_pass_on_6d_t *val);

typedef enum {
  LSM6DSL_DEG_80      = 0,
  LSM6DSL_DEG_70      = 1,
  LSM6DSL_DEG_60      = 2,
  LSM6DSL_DEG_50      = 3,
  LSM6DSL_6D_TH_ND    = 4,     
} lsm6dsl_sixd_ths_t;
int32_t lsm6dsl_6d_threshold_set(lsm6dsl_ctx_t *ctx, lsm6dsl_sixd_ths_t val);
int32_t lsm6dsl_6d_threshold_get(lsm6dsl_ctx_t *ctx, lsm6dsl_sixd_ths_t *val);

int32_t lsm6dsl_4d_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_4d_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_ff_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_ff_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_FF_TSH_156mg = 0,
  LSM6DSL_FF_TSH_219mg = 1,
  LSM6DSL_FF_TSH_250mg = 2,
  LSM6DSL_FF_TSH_312mg = 3,
  LSM6DSL_FF_TSH_344mg = 4,
  LSM6DSL_FF_TSH_406mg = 5,
  LSM6DSL_FF_TSH_469mg = 6,
  LSM6DSL_FF_TSH_500mg = 7,
  LSM6DSL_FF_TSH_ND    = 8,     
} lsm6dsl_ff_ths_t;
int32_t lsm6dsl_ff_threshold_set(lsm6dsl_ctx_t *ctx, lsm6dsl_ff_ths_t val);
int32_t lsm6dsl_ff_threshold_get(lsm6dsl_ctx_t *ctx, lsm6dsl_ff_ths_t *val);

int32_t lsm6dsl_fifo_watermark_set(lsm6dsl_ctx_t *ctx, uint16_t val);
int32_t lsm6dsl_fifo_watermark_get(lsm6dsl_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsl_fifo_data_level_get(lsm6dsl_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsl_fifo_wtm_flag_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_fifo_pattern_get(lsm6dsl_ctx_t *ctx, uint16_t *val);

int32_t lsm6dsl_fifo_temp_batch_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_fifo_temp_batch_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_TRG_XL_GY_DRDY     = 0,
  LSM6DSL_TRG_STEP_DETECT    = 1,
  LSM6DSL_TRG_SH_DRDY        = 2,
  LSM6DSL_TRG_SH_ND          = 3,     
} lsm6dsl_trigger_fifo_t;
int32_t lsm6dsl_fifo_write_trigger_set(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_trigger_fifo_t val);
int32_t lsm6dsl_fifo_write_trigger_get(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_trigger_fifo_t *val);

int32_t lsm6dsl_fifo_pedo_and_timestamp_batch_set(lsm6dsl_ctx_t *ctx,
                                                  uint8_t val);
int32_t lsm6dsl_fifo_pedo_and_timestamp_batch_get(lsm6dsl_ctx_t *ctx,
                                                  uint8_t *val);

typedef enum {
  LSM6DSL_FIFO_XL_DISABLE  = 0,
  LSM6DSL_FIFO_XL_NO_DEC   = 1,
  LSM6DSL_FIFO_XL_DEC_2    = 2,
  LSM6DSL_FIFO_XL_DEC_3    = 3,
  LSM6DSL_FIFO_XL_DEC_4    = 4,
  LSM6DSL_FIFO_XL_DEC_8    = 5,
  LSM6DSL_FIFO_XL_DEC_16   = 6,
  LSM6DSL_FIFO_XL_DEC_32   = 7,
  LSM6DSL_FIFO_XL_DEC_ND   = 8,     
} lsm6dsl_dec_fifo_xl_t;
int32_t lsm6dsl_fifo_xl_batch_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_xl_t val);
int32_t lsm6dsl_fifo_xl_batch_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_xl_t *val);

typedef enum {
  LSM6DSL_FIFO_GY_DISABLE = 0,
  LSM6DSL_FIFO_GY_NO_DEC  = 1,
  LSM6DSL_FIFO_GY_DEC_2   = 2,
  LSM6DSL_FIFO_GY_DEC_3   = 3,
  LSM6DSL_FIFO_GY_DEC_4   = 4,
  LSM6DSL_FIFO_GY_DEC_8   = 5,
  LSM6DSL_FIFO_GY_DEC_16  = 6,
  LSM6DSL_FIFO_GY_DEC_32  = 7,
  LSM6DSL_FIFO_GY_DEC_ND  = 8,     
} lsm6dsl_dec_fifo_gyro_t;
int32_t lsm6dsl_fifo_gy_batch_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_gyro_t val);
int32_t lsm6dsl_fifo_gy_batch_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_gyro_t *val);

typedef enum {
  LSM6DSL_FIFO_DS3_DISABLE   = 0,
  LSM6DSL_FIFO_DS3_NO_DEC    = 1,
  LSM6DSL_FIFO_DS3_DEC_2     = 2,
  LSM6DSL_FIFO_DS3_DEC_3     = 3,
  LSM6DSL_FIFO_DS3_DEC_4     = 4,
  LSM6DSL_FIFO_DS3_DEC_8     = 5,
  LSM6DSL_FIFO_DS3_DEC_16    = 6,
  LSM6DSL_FIFO_DS3_DEC_32    = 7,
  LSM6DSL_FIFO_DS3_DEC_ND    = 8,     
} lsm6dsl_dec_ds3_fifo_t;
int32_t lsm6dsl_fifo_dataset_3_batch_set(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds3_fifo_t val);
int32_t lsm6dsl_fifo_dataset_3_batch_get(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds3_fifo_t *val);

typedef enum {
  LSM6DSL_FIFO_DS4_DISABLE  = 0,
  LSM6DSL_FIFO_DS4_NO_DEC   = 1,
  LSM6DSL_FIFO_DS4_DEC_2    = 2,
  LSM6DSL_FIFO_DS4_DEC_3    = 3,
  LSM6DSL_FIFO_DS4_DEC_4    = 4,
  LSM6DSL_FIFO_DS4_DEC_8    = 5,
  LSM6DSL_FIFO_DS4_DEC_16   = 6,
  LSM6DSL_FIFO_DS4_DEC_32   = 7,
  LSM6DSL_FIFO_DS4_DEC_ND   = 8,     
} lsm6dsl_dec_ds4_fifo_t;
int32_t lsm6dsl_fifo_dataset_4_batch_set(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds4_fifo_t val);
int32_t lsm6dsl_fifo_dataset_4_batch_get(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds4_fifo_t *val);

int32_t lsm6dsl_fifo_xl_gy_8bit_format_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_fifo_xl_gy_8bit_format_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_fifo_stop_on_wtm_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_fifo_stop_on_wtm_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_BYPASS_MODE           = 0,
  LSM6DSL_FIFO_MODE             = 1,
  LSM6DSL_STREAM_TO_FIFO_MODE   = 3,
  LSM6DSL_BYPASS_TO_STREAM_MODE = 4,
  LSM6DSL_STREAM_MODE           = 6,
  LSM6DSL_FIFO_MODE_ND          = 8,     
} lsm6dsl_fifo_mode_t;
int32_t lsm6dsl_fifo_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fifo_mode_t val);
int32_t lsm6dsl_fifo_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fifo_mode_t *val);

typedef enum {
  LSM6DSL_FIFO_DISABLE   =  0,
  LSM6DSL_FIFO_12Hz5     =  1,
  LSM6DSL_FIFO_26Hz      =  2,
  LSM6DSL_FIFO_52Hz      =  3,
  LSM6DSL_FIFO_104Hz     =  4,
  LSM6DSL_FIFO_208Hz     =  5,
  LSM6DSL_FIFO_416Hz     =  6,
  LSM6DSL_FIFO_833Hz     =  7,
  LSM6DSL_FIFO_1k66Hz    =  8,
  LSM6DSL_FIFO_3k33Hz    =  9,
  LSM6DSL_FIFO_6k66Hz    = 10,
  LSM6DSL_FIFO_RATE_ND   = 11,     
} lsm6dsl_odr_fifo_t;
int32_t lsm6dsl_fifo_data_rate_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_odr_fifo_t val);
int32_t lsm6dsl_fifo_data_rate_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_odr_fifo_t *val);

typedef enum {
  LSM6DSL_DEN_ACT_LOW    = 0,
  LSM6DSL_DEN_ACT_HIGH   = 1,
  LSM6DSL_DEN_POL_ND     = 2,     
} lsm6dsl_den_lh_t;
int32_t lsm6dsl_den_polarity_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_lh_t val);
int32_t lsm6dsl_den_polarity_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_lh_t *val);

typedef enum {
  LSM6DSL_DEN_DISABLE    = 0,
  LSM6DSL_LEVEL_FIFO     = 6,
  LSM6DSL_LEVEL_LETCHED  = 3,
  LSM6DSL_LEVEL_TRIGGER  = 2,
  LSM6DSL_EDGE_TRIGGER   = 4,
  LSM6DSL_DEN_MODE_ND    = 5,     
} lsm6dsl_den_mode_t;
int32_t lsm6dsl_den_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_mode_t val);
int32_t lsm6dsl_den_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_mode_t *val);

typedef enum {
  LSM6DSL_STAMP_IN_GY_DATA     = 0,
  LSM6DSL_STAMP_IN_XL_DATA     = 1,
  LSM6DSL_STAMP_IN_GY_XL_DATA  = 2,
  LSM6DSL_DEN_STAMP_ND         = 3,     
} lsm6dsl_den_xl_en_t;
int32_t lsm6dsl_den_enable_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_xl_en_t val);
int32_t lsm6dsl_den_enable_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_xl_en_t *val);

int32_t lsm6dsl_den_mark_axis_z_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_den_mark_axis_z_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_den_mark_axis_y_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_den_mark_axis_y_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_den_mark_axis_x_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_den_mark_axis_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_step_reset_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_step_reset_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_PEDO_AT_2g = 0,
  LSM6DSL_PEDO_AT_4g = 1,
  LSM6DSL_PEDO_FS_ND = 2,     
} lsm6dsl_pedo_fs_t;
int32_t lsm6dsl_pedo_full_scale_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_pedo_fs_t val);
int32_t lsm6dsl_pedo_full_scale_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_pedo_fs_t *val);

int32_t lsm6dsl_pedo_debounce_steps_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_debounce_steps_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_timeout_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_pedo_timeout_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_pedo_steps_period_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_pedo_steps_period_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_motion_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_motion_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_motion_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_motion_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_tilt_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_tilt_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_wrist_tilt_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_wrist_tilt_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_tilt_latency_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_tilt_latency_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_tilt_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_tilt_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_tilt_src_set(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_a_wrist_tilt_mask_t *val);
int32_t lsm6dsl_tilt_src_get(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_a_wrist_tilt_mask_t *val);

int32_t lsm6dsl_mag_soft_iron_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_mag_soft_iron_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_mag_hard_iron_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_mag_hard_iron_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_mag_soft_iron_mat_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_mag_soft_iron_mat_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_mag_offset_set(lsm6dsl_ctx_t *ctx, uint8_t *buff);
int32_t lsm6dsl_mag_offset_get(lsm6dsl_ctx_t *ctx, uint8_t *buff);

int32_t lsm6dsl_func_en_set(lsm6dsl_ctx_t *ctx, uint8_t val);

int32_t lsm6dsl_sh_sync_sens_frame_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_sync_sens_frame_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_RES_RATIO_2_11  = 0,
  LSM6DSL_RES_RATIO_2_12  = 1,
  LSM6DSL_RES_RATIO_2_13  = 2,
  LSM6DSL_RES_RATIO_2_14  = 3,
  LSM6DSL_RES_RATIO_ND    = 4,     
} lsm6dsl_rr_t;
int32_t lsm6dsl_sh_sync_sens_ratio_set(lsm6dsl_ctx_t *ctx, lsm6dsl_rr_t val);
int32_t lsm6dsl_sh_sync_sens_ratio_get(lsm6dsl_ctx_t *ctx, lsm6dsl_rr_t *val);

int32_t lsm6dsl_sh_master_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_master_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_sh_pass_through_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_pass_through_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_EXT_PULL_UP       = 0,
  LSM6DSL_INTERNAL_PULL_UP  = 1,
  LSM6DSL_SH_PIN_MODE       = 2,     
} lsm6dsl_pull_up_en_t;
int32_t lsm6dsl_sh_pin_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_pull_up_en_t val);
int32_t lsm6dsl_sh_pin_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_pull_up_en_t *val);

typedef enum {
  LSM6DSL_XL_GY_DRDY        = 0,
  LSM6DSL_EXT_ON_INT2_PIN   = 1,
  LSM6DSL_SH_SYNCRO_ND      = 2,     
} lsm6dsl_start_config_t;
int32_t lsm6dsl_sh_syncro_mode_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_start_config_t val);
int32_t lsm6dsl_sh_syncro_mode_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_start_config_t *val);

int32_t lsm6dsl_sh_drdy_on_int1_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_drdy_on_int1_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef struct {
    lsm6dsl_sensorhub1_reg_t   sh_byte_1;
    lsm6dsl_sensorhub2_reg_t   sh_byte_2;
    lsm6dsl_sensorhub3_reg_t   sh_byte_3;
    lsm6dsl_sensorhub4_reg_t   sh_byte_4;
    lsm6dsl_sensorhub5_reg_t   sh_byte_5;
    lsm6dsl_sensorhub6_reg_t   sh_byte_6;
    lsm6dsl_sensorhub7_reg_t   sh_byte_7;
    lsm6dsl_sensorhub8_reg_t   sh_byte_8;
    lsm6dsl_sensorhub9_reg_t   sh_byte_9;
    lsm6dsl_sensorhub10_reg_t  sh_byte_10;
    lsm6dsl_sensorhub11_reg_t  sh_byte_11;
    lsm6dsl_sensorhub12_reg_t  sh_byte_12;
    lsm6dsl_sensorhub13_reg_t  sh_byte_13;
    lsm6dsl_sensorhub14_reg_t  sh_byte_14;
    lsm6dsl_sensorhub15_reg_t  sh_byte_15;
    lsm6dsl_sensorhub16_reg_t  sh_byte_16;
    lsm6dsl_sensorhub17_reg_t  sh_byte_17;
    lsm6dsl_sensorhub18_reg_t  sh_byte_18;
} lsm6dsl_emb_sh_read_t;
int32_t lsm6dsl_sh_read_data_raw_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_emb_sh_read_t *val);

int32_t lsm6dsl_sh_cmd_sens_sync_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_cmd_sens_sync_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

int32_t lsm6dsl_sh_spi_sync_error_set(lsm6dsl_ctx_t *ctx, uint8_t val);
int32_t lsm6dsl_sh_spi_sync_error_get(lsm6dsl_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM6DSL_SLV_0        = 0,
  LSM6DSL_SLV_0_1      = 1,
  LSM6DSL_SLV_0_1_2    = 2,
  LSM6DSL_SLV_0_1_2_3  = 3,
  LSM6DSL_SLV_EN_ND    = 4,     
} lsm6dsl_aux_sens_on_t;
int32_t lsm6dsl_sh_num_of_dev_connected_set(lsm6dsl_ctx_t *ctx,
                                            lsm6dsl_aux_sens_on_t val);
int32_t lsm6dsl_sh_num_of_dev_connected_get(lsm6dsl_ctx_t *ctx,
                                            lsm6dsl_aux_sens_on_t *val);

typedef struct{
  uint8_t   slv0_add;
  uint8_t   slv0_subadd;
  uint8_t   slv0_data;
} lsm6dsl_sh_cfg_write_t;
int32_t lsm6dsl_sh_cfg_write(lsm6dsl_ctx_t *ctx, lsm6dsl_sh_cfg_write_t *val);

typedef struct{
  uint8_t   slv_add;
  uint8_t   slv_subadd;
  uint8_t   slv_len;
} lsm6dsl_sh_cfg_read_t;
int32_t lsm6dsl_sh_slv0_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val);
int32_t lsm6dsl_sh_slv1_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val);
int32_t lsm6dsl_sh_slv2_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val);
int32_t lsm6dsl_sh_slv3_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val);

typedef enum {
  LSM6DSL_SL0_NO_DEC   = 0,
  LSM6DSL_SL0_DEC_2    = 1,
  LSM6DSL_SL0_DEC_4    = 2,
  LSM6DSL_SL0_DEC_8    = 3,
  LSM6DSL_SL0_DEC_ND   = 4,     
} lsm6dsl_slave0_rate_t;
int32_t lsm6dsl_sh_slave_0_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave0_rate_t val);
int32_t lsm6dsl_sh_slave_0_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave0_rate_t *val);

typedef enum {
  LSM6DSL_EACH_SH_CYCLE     = 0,
  LSM6DSL_ONLY_FIRST_CYCLE  = 1,
  LSM6DSL_SH_WR_MODE_ND     = 2,     
} lsm6dsl_write_once_t;
int32_t lsm6dsl_sh_write_mode_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_write_once_t val);
int32_t lsm6dsl_sh_write_mode_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_write_once_t *val);

typedef enum {
  LSM6DSL_SL1_NO_DEC   = 0,
  LSM6DSL_SL1_DEC_2    = 1,
  LSM6DSL_SL1_DEC_4    = 2,
  LSM6DSL_SL1_DEC_8    = 3,
  LSM6DSL_SL1_DEC_ND   = 4,     
} lsm6dsl_slave1_rate_t;
int32_t lsm6dsl_sh_slave_1_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave1_rate_t val);
int32_t lsm6dsl_sh_slave_1_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave1_rate_t *val);

typedef enum {
  LSM6DSL_SL2_NO_DEC  = 0,
  LSM6DSL_SL2_DEC_2   = 1,
  LSM6DSL_SL2_DEC_4   = 2,
  LSM6DSL_SL2_DEC_8   = 3,
  LSM6DSL_SL2_DEC_ND  = 4,     
} lsm6dsl_slave2_rate_t;
int32_t lsm6dsl_sh_slave_2_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave2_rate_t val);
int32_t lsm6dsl_sh_slave_2_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave2_rate_t *val);

typedef enum {
  LSM6DSL_SL3_NO_DEC  = 0,
  LSM6DSL_SL3_DEC_2   = 1,
  LSM6DSL_SL3_DEC_4   = 2,
  LSM6DSL_SL3_DEC_8   = 3,
  LSM6DSL_SL3_DEC_ND  = 4,     
} lsm6dsl_slave3_rate_t;
int32_t lsm6dsl_sh_slave_3_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave3_rate_t val);
int32_t lsm6dsl_sh_slave_3_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave3_rate_t *val);




 







 
#line 38 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm6dsl\\lsm6dsl_reg.c"







 








 










 
int32_t lsm6dsl_read_reg(lsm6dsl_ctx_t* ctx, uint8_t reg, uint8_t* data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}










 
int32_t lsm6dsl_write_reg(lsm6dsl_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}




 






 

float_t lsm6dsl_from_fs2g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.061f);
}

float_t lsm6dsl_from_fs4g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.122f);
}

float_t lsm6dsl_from_fs8g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.244f);
}

float_t lsm6dsl_from_fs16g_to_mg(int16_t lsb)
{
  return ((float_t)lsb * 0.488f);
}

float_t lsm6dsl_from_fs125dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 4.375f);
}

float_t lsm6dsl_from_fs250dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 8.750f);
}

float_t lsm6dsl_from_fs500dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 17.50f);
}

float_t lsm6dsl_from_fs1000dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 35.0f);
}

float_t lsm6dsl_from_fs2000dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 70.0f);
}

float_t lsm6dsl_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}




 








 








 
int32_t lsm6dsl_xl_full_scale_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_xl_t val)
{
  lsm6dsl_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ctrl1_xl.fs_xl = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_xl_full_scale_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_xl_t *val)
{
  lsm6dsl_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  switch (ctrl1_xl.fs_xl) {
    case LSM6DSL_2g:
      *val = LSM6DSL_2g;
      break;
    case LSM6DSL_16g:
      *val = LSM6DSL_16g;
      break;
    case LSM6DSL_4g:
      *val = LSM6DSL_4g;
      break;
    case LSM6DSL_8g:
      *val = LSM6DSL_8g;
      break;
    default:
      *val = LSM6DSL_XL_FS_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_xl_data_rate_set(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_xl_t val)
{
  lsm6dsl_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ctrl1_xl.odr_xl = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_xl_data_rate_get(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_xl_t *val)
{
  lsm6dsl_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  switch (ctrl1_xl.odr_xl) {
    case LSM6DSL_XL_ODR_OFF:
      *val = LSM6DSL_XL_ODR_OFF;
      break;
    case LSM6DSL_XL_ODR_12Hz5:
      *val = LSM6DSL_XL_ODR_12Hz5;
      break;
    case LSM6DSL_XL_ODR_26Hz:
      *val = LSM6DSL_XL_ODR_26Hz;
      break;
    case LSM6DSL_XL_ODR_52Hz:
      *val = LSM6DSL_XL_ODR_52Hz;
      break;
    case LSM6DSL_XL_ODR_104Hz:
      *val = LSM6DSL_XL_ODR_104Hz;
      break;
    case LSM6DSL_XL_ODR_208Hz:
      *val = LSM6DSL_XL_ODR_208Hz;
      break;
    case LSM6DSL_XL_ODR_416Hz:
      *val = LSM6DSL_XL_ODR_416Hz;
      break;
    case LSM6DSL_XL_ODR_833Hz:
      *val = LSM6DSL_XL_ODR_833Hz;
      break;
    case LSM6DSL_XL_ODR_1k66Hz:
      *val = LSM6DSL_XL_ODR_1k66Hz;
      break;
    case LSM6DSL_XL_ODR_3k33Hz:
      *val = LSM6DSL_XL_ODR_3k33Hz;
      break;
    case LSM6DSL_XL_ODR_6k66Hz:
      *val = LSM6DSL_XL_ODR_6k66Hz;
      break;
    case LSM6DSL_XL_ODR_1Hz6:
      *val = LSM6DSL_XL_ODR_1Hz6;
      break;
    default:
      *val = LSM6DSL_XL_ODR_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_gy_full_scale_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_g_t val)
{
  lsm6dsl_ctrl2_g_t ctrl2_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x11U, (uint8_t*)&ctrl2_g, 1);
  if(ret == 0){
    ctrl2_g.fs_g = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x11U, (uint8_t*)&ctrl2_g, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_gy_full_scale_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fs_g_t *val)
{
  lsm6dsl_ctrl2_g_t ctrl2_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x11U, (uint8_t*)&ctrl2_g, 1);
  switch (ctrl2_g.fs_g) {
    case LSM6DSL_250dps:
      *val = LSM6DSL_250dps;
      break;
    case LSM6DSL_125dps:
      *val = LSM6DSL_125dps;
      break;
    case LSM6DSL_500dps:
      *val = LSM6DSL_500dps;
      break;
    case LSM6DSL_1000dps:
      *val = LSM6DSL_1000dps;
      break;
    case LSM6DSL_2000dps:
      *val = LSM6DSL_2000dps;
      break;
    default:
      *val = LSM6DSL_GY_FS_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_gy_data_rate_set(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_g_t val)
{
  lsm6dsl_ctrl2_g_t ctrl2_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x11U, (uint8_t*)&ctrl2_g, 1);
  if(ret == 0){
    ctrl2_g.odr_g = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x11U, (uint8_t*)&ctrl2_g, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_gy_data_rate_get(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_g_t *val)
{
  lsm6dsl_ctrl2_g_t ctrl2_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x11U, (uint8_t*)&ctrl2_g, 1);
  switch (ctrl2_g.odr_g) {
    case LSM6DSL_GY_ODR_OFF:
      *val = LSM6DSL_GY_ODR_OFF;
      break;
    case LSM6DSL_GY_ODR_12Hz5:
      *val = LSM6DSL_GY_ODR_12Hz5;
      break;
    case LSM6DSL_GY_ODR_26Hz:
      *val = LSM6DSL_GY_ODR_26Hz;
      break;
    case LSM6DSL_GY_ODR_52Hz:
      *val = LSM6DSL_GY_ODR_52Hz;
      break;
    case LSM6DSL_GY_ODR_104Hz:
      *val = LSM6DSL_GY_ODR_104Hz;
      break;
    case LSM6DSL_GY_ODR_208Hz:
      *val = LSM6DSL_GY_ODR_208Hz;
      break;
    case LSM6DSL_GY_ODR_416Hz:
      *val = LSM6DSL_GY_ODR_416Hz;
      break;
    case LSM6DSL_GY_ODR_833Hz:
      *val = LSM6DSL_GY_ODR_833Hz;
      break;
    case LSM6DSL_GY_ODR_1k66Hz:
      *val = LSM6DSL_GY_ODR_1k66Hz;
      break;
    case LSM6DSL_GY_ODR_3k33Hz:
      *val = LSM6DSL_GY_ODR_3k33Hz;
      break;
    case LSM6DSL_GY_ODR_6k66Hz:
      *val = LSM6DSL_GY_ODR_6k66Hz;
      break;
    default:
      *val = LSM6DSL_GY_ODR_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_block_data_update_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.bdu = val;
    ret = lsm6dsl_write_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_block_data_update_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.bdu;

  return ret;
}









 
int32_t lsm6dsl_xl_offset_weight_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_usr_off_w_t val)
{
  lsm6dsl_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  if(ret == 0){
    ctrl6_c.usr_off_w = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_xl_offset_weight_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_usr_off_w_t *val)
{
  lsm6dsl_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  switch (ctrl6_c.usr_off_w) {
    case LSM6DSL_LSb_1mg:
      *val = LSM6DSL_LSb_1mg;
      break;
    case LSM6DSL_LSb_16mg:
      *val = LSM6DSL_LSb_16mg;
      break;
    default:
      *val = LSM6DSL_WEIGHT_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_xl_power_mode_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_xl_hm_mode_t val)
{
  lsm6dsl_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  if(ret == 0){
    ctrl6_c.xl_hm_mode = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_xl_power_mode_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_xl_hm_mode_t *val)
{
  lsm6dsl_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  switch (ctrl6_c.xl_hm_mode) {
    case LSM6DSL_XL_HIGH_PERFORMANCE:
      *val = LSM6DSL_XL_HIGH_PERFORMANCE;
      break;
    case LSM6DSL_XL_NORMAL:
      *val = LSM6DSL_XL_NORMAL;
      break;
    default:
      *val = LSM6DSL_XL_PW_MODE_ND;
      break;
  }

  return ret;
}










 
int32_t lsm6dsl_rounding_on_status_set(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_rounding_status_t val)
{
  lsm6dsl_ctrl7_g_t ctrl7_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);
  if(ret == 0){
    ctrl7_g.rounding_status = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);
  }
  return ret;
}










 
int32_t lsm6dsl_rounding_on_status_get(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_rounding_status_t *val)
{
  lsm6dsl_ctrl7_g_t ctrl7_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);
  switch (ctrl7_g.rounding_status) {
    case LSM6DSL_STAT_RND_DISABLE:
      *val = LSM6DSL_STAT_RND_DISABLE;
      break;
    case LSM6DSL_STAT_RND_ENABLE:
      *val = LSM6DSL_STAT_RND_ENABLE;
      break;
    default:
      *val = LSM6DSL_STAT_RND_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_gy_power_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_g_hm_mode_t val)
{
  lsm6dsl_ctrl7_g_t ctrl7_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);
  if(ret == 0){
    ctrl7_g.g_hm_mode = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_gy_power_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_g_hm_mode_t *val)
{
  lsm6dsl_ctrl7_g_t ctrl7_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);
  switch (ctrl7_g.g_hm_mode) {
    case LSM6DSL_GY_HIGH_PERFORMANCE:
      *val = LSM6DSL_GY_HIGH_PERFORMANCE;
      break;
    case LSM6DSL_GY_NORMAL:
      *val = LSM6DSL_GY_NORMAL;
      break;
    default:
      *val = LSM6DSL_GY_PW_MODE_ND;
      break;
  }

  return ret;
}









 
int32_t lsm6dsl_all_sources_get(lsm6dsl_ctx_t *ctx,
                                lsm6dsl_all_sources_t *val)
{
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1BU,
                         (uint8_t*)&(val->wake_up_src), 1);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x1CU,
                           (uint8_t*)&(val->tap_src), 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x1DU,
                           (uint8_t*)&(val->d6d_src), 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x1EU,
                           (uint8_t*)&(val->status_reg), 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x53U,
                           (uint8_t*)&(val->func_src1), 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x54U,
                           (uint8_t*)&(val->func_src2), 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x55U,
                           (uint8_t*)&(val->wrist_tilt_ia), 1);
  }
  if(ret == 0){
    ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_B);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x59U,
                           (uint8_t*)&(val->a_wrist_tilt_mask), 1);
  }
  if(ret == 0){
    ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
  }

  return ret;
}







 
int32_t lsm6dsl_status_reg_get(lsm6dsl_ctx_t *ctx, lsm6dsl_status_reg_t *val)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x1EU, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm6dsl_xl_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_status_reg_t status_reg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1EU, (uint8_t*)&status_reg, 1);
  *val = status_reg.xlda;

  return ret;
}








 
int32_t lsm6dsl_gy_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_status_reg_t status_reg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1EU, (uint8_t*)&status_reg, 1);
  *val = status_reg.gda;

  return ret;
}








 
int32_t lsm6dsl_temp_flag_data_ready_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_status_reg_t status_reg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1EU, (uint8_t*)&status_reg, 1);
  *val = status_reg.tda;

  return ret;
}










 
int32_t lsm6dsl_xl_usr_offset_set(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsl_write_reg(ctx, 0x73U, buff, 3);
  return ret;
}










 
int32_t lsm6dsl_xl_usr_offset_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x73U, buff, 3);
  return ret;
}




 







 









 
int32_t lsm6dsl_timestamp_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  if(ret == 0){
    ctrl10_c.timer_en = val;
    if ( val != 0x00U) {
      ctrl10_c.func_en = val;
      ret = lsm6dsl_write_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
    }
  }
  return ret;
}









 
int32_t lsm6dsl_timestamp_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  *val = ctrl10_c.timer_en;

  return ret;
}













 
int32_t lsm6dsl_timestamp_res_set(lsm6dsl_ctx_t *ctx, lsm6dsl_timer_hr_t val)
{
  lsm6dsl_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5CU, (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    wake_up_dur.timer_hr = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x5CU,
                            (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}













 
int32_t lsm6dsl_timestamp_res_get(lsm6dsl_ctx_t *ctx, lsm6dsl_timer_hr_t *val)
{
  lsm6dsl_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5CU, (uint8_t*)&wake_up_dur, 1);
  switch (wake_up_dur.timer_hr) {
    case LSM6DSL_LSB_6ms4:
      *val = LSM6DSL_LSB_6ms4;
      break;
    case LSM6DSL_LSB_25us:
      *val = LSM6DSL_LSB_25us;
      break;
    default:
      *val = LSM6DSL_TS_RES_ND;
      break;
  }

  return ret;
}




 






 









 
int32_t lsm6dsl_rounding_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_rounding_t val)
{
  lsm6dsl_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  if(ret == 0){
    ctrl5_c.rounding = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_rounding_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_rounding_t *val)
{
  lsm6dsl_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  switch (ctrl5_c.rounding) {
    case LSM6DSL_ROUND_DISABLE:
      *val = LSM6DSL_ROUND_DISABLE;
      break;
    case LSM6DSL_ROUND_XL:
      *val = LSM6DSL_ROUND_XL;
      break;
    case LSM6DSL_ROUND_GY:
      *val = LSM6DSL_ROUND_GY;
      break;
    case LSM6DSL_ROUND_GY_XL:
      *val = LSM6DSL_ROUND_GY_XL;
      break;
    case LSM6DSL_ROUND_SH1_TO_SH6:
      *val = LSM6DSL_ROUND_SH1_TO_SH6;
      break;
    case LSM6DSL_ROUND_XL_SH1_TO_SH6:
      *val = LSM6DSL_ROUND_XL_SH1_TO_SH6;
      break;
    case LSM6DSL_ROUND_GY_XL_SH1_TO_SH12:
      *val = LSM6DSL_ROUND_GY_XL_SH1_TO_SH12;
      break;
    case LSM6DSL_ROUND_GY_XL_SH1_TO_SH6:
      *val = LSM6DSL_ROUND_GY_XL_SH1_TO_SH6;
      break;
    default:
      *val = LSM6DSL_ROUND_OUT_ND;
      break;
  }

  return ret;
}









 
int32_t lsm6dsl_temperature_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x20U, buff, 2);
  return ret;
}









 
int32_t lsm6dsl_angular_rate_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x22U, buff, 6);
  return ret;
}









 
int32_t lsm6dsl_acceleration_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x28U, buff, 6);
  return ret;
}








 
int32_t lsm6dsl_mag_calibrated_raw_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x66U, buff, 6);
  return ret;
}









 
int32_t lsm6dsl_fifo_raw_data_get(lsm6dsl_ctx_t *ctx, uint8_t *buffer,
                                  uint8_t len)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x3EU, buffer, len);
  return ret;
}




 






 









 
int32_t lsm6dsl_mem_bank_set(lsm6dsl_ctx_t *ctx, lsm6dsl_func_cfg_en_t val)
{
  lsm6dsl_func_cfg_access_t func_cfg_access;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x01U,
                         (uint8_t*)&func_cfg_access, 1);
  if(ret == 0){
    func_cfg_access.func_cfg_en = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x01U,
                            (uint8_t*)&func_cfg_access, 1);
  }

  return ret;
}









 
int32_t lsm6dsl_mem_bank_get(lsm6dsl_ctx_t *ctx, lsm6dsl_func_cfg_en_t *val)
{
  lsm6dsl_func_cfg_access_t func_cfg_access;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x01U,
                         (uint8_t*)&func_cfg_access, 1);
  switch (func_cfg_access.func_cfg_en) {
    case LSM6DSL_USER_BANK:
      *val = LSM6DSL_USER_BANK;
      break;
    case LSM6DSL_BANK_B:
      *val = LSM6DSL_BANK_B;
      break;
    default:
      *val = LSM6DSL_BANK_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_data_ready_mode_set(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_drdy_pulsed_g_t val)
{
  lsm6dsl_drdy_pulse_cfg_g_t drdy_pulse_cfg_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0BU,
                         (uint8_t*)&drdy_pulse_cfg_g, 1);
  if(ret == 0){
    drdy_pulse_cfg_g.drdy_pulsed = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x0BU,
                            (uint8_t*)&drdy_pulse_cfg_g, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_data_ready_mode_get(lsm6dsl_ctx_t *ctx,
                                    lsm6dsl_drdy_pulsed_g_t *val)
{
  lsm6dsl_drdy_pulse_cfg_g_t drdy_pulse_cfg_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0BU,
                         (uint8_t*)&drdy_pulse_cfg_g, 1);
  switch (drdy_pulse_cfg_g.drdy_pulsed) {
    case LSM6DSL_DRDY_LATCHED:
      *val = LSM6DSL_DRDY_LATCHED;
      break;
    case LSM6DSL_DRDY_PULSED:
      *val = LSM6DSL_DRDY_PULSED;
      break;
    default:
      *val = LSM6DSL_DRDY_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_device_id_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x0FU, buff, 1);
  return ret;
}








 
int32_t lsm6dsl_reset_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.sw_reset = val;
    ret = lsm6dsl_write_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_reset_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.sw_reset;

  return ret;
}








 
int32_t lsm6dsl_data_format_set(lsm6dsl_ctx_t *ctx, lsm6dsl_ble_t val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.ble = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_data_format_get(lsm6dsl_ctx_t *ctx, lsm6dsl_ble_t *val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  switch (ctrl3_c.ble) {
    case LSM6DSL_LSB_AT_LOW_ADD:
      *val = LSM6DSL_LSB_AT_LOW_ADD;
      break;
    case LSM6DSL_MSB_AT_LOW_ADD:
      *val = LSM6DSL_MSB_AT_LOW_ADD;
      break;
    default:
      *val = LSM6DSL_DATA_FMT_ND;
      break;
  }

  return ret;
}









 
int32_t lsm6dsl_auto_increment_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.if_inc = val;
    ret = lsm6dsl_write_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_auto_increment_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.if_inc;

  return ret;
}








 
int32_t lsm6dsl_boot_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.boot = val;
    ret = lsm6dsl_write_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_boot_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  *val = ctrl3_c.boot;

  return ret;
}








 
int32_t lsm6dsl_xl_self_test_set(lsm6dsl_ctx_t *ctx, lsm6dsl_st_xl_t val)
{
  lsm6dsl_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  if(ret == 0){
    ctrl5_c.st_xl = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_xl_self_test_get(lsm6dsl_ctx_t *ctx, lsm6dsl_st_xl_t *val)
{
  lsm6dsl_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  switch (ctrl5_c.st_xl) {
    case LSM6DSL_XL_ST_DISABLE:
      *val = LSM6DSL_XL_ST_DISABLE;
      break;
    case LSM6DSL_XL_ST_POSITIVE:
      *val = LSM6DSL_XL_ST_POSITIVE;
      break;
    case LSM6DSL_XL_ST_NEGATIVE:
      *val = LSM6DSL_XL_ST_NEGATIVE;
      break;
    default:
      *val = LSM6DSL_XL_ST_ND;
      break;
  }
  return ret;
}








 
int32_t lsm6dsl_gy_self_test_set(lsm6dsl_ctx_t *ctx, lsm6dsl_st_g_t val)
{
  lsm6dsl_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  if(ret == 0){
    ctrl5_c.st_g = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_gy_self_test_get(lsm6dsl_ctx_t *ctx, lsm6dsl_st_g_t *val)
{
  lsm6dsl_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  switch (ctrl5_c.st_g) {
    case LSM6DSL_GY_ST_DISABLE:
      *val = LSM6DSL_GY_ST_DISABLE;
      break;
    case LSM6DSL_GY_ST_POSITIVE:
      *val = LSM6DSL_GY_ST_POSITIVE;
      break;
    case LSM6DSL_GY_ST_NEGATIVE:
      *val = LSM6DSL_GY_ST_NEGATIVE;
      break;
    default:
      *val = LSM6DSL_GY_ST_ND;
      break;
  }

  return ret;
}




 







 









 
int32_t lsm6dsl_filter_settling_mask_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ctrl4_c.drdy_mask = val;
    ret = lsm6dsl_write_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_filter_settling_mask_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  *val = ctrl4_c.drdy_mask;

  return ret;
}









 
int32_t lsm6dsl_xl_hp_path_internal_set(lsm6dsl_ctx_t *ctx,
                                        lsm6dsl_slope_fds_t val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  if(ret == 0){
    tap_cfg.slope_fds = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_xl_hp_path_internal_get(lsm6dsl_ctx_t *ctx,
                                        lsm6dsl_slope_fds_t *val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  switch (tap_cfg.slope_fds) {
    case LSM6DSL_USE_SLOPE:
      *val = LSM6DSL_USE_SLOPE;
      break;
    case LSM6DSL_USE_HPF:
      *val = LSM6DSL_USE_HPF;
      break;
    default:
      *val = LSM6DSL_HP_PATH_ND;
      break;
  }

  return ret;
}




 







 









 
int32_t lsm6dsl_xl_filter_analog_set(lsm6dsl_ctx_t *ctx, lsm6dsl_bw0_xl_t val)
{
  lsm6dsl_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ctrl1_xl.bw0_xl = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_xl_filter_analog_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_bw0_xl_t *val)
{
  lsm6dsl_ctrl1_xl_t ctrl1_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  switch (ctrl1_xl.bw0_xl) {
    case LSM6DSL_XL_ANA_BW_1k5Hz:
      *val = LSM6DSL_XL_ANA_BW_1k5Hz;
      break;
    case LSM6DSL_XL_ANA_BW_400Hz:
      *val = LSM6DSL_XL_ANA_BW_400Hz;
      break;
    default:
      *val = LSM6DSL_XL_ANA_BW_ND;
      break;
  }

  return ret;
}




 







 









 
int32_t lsm6dsl_xl_lp1_bandwidth_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_lpf1_bw_sel_t val)
{
  lsm6dsl_ctrl1_xl_t ctrl1_xl;
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
  if(ret == 0){
    ctrl1_xl.lpf1_bw_sel = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
    if(ret == 0){
      ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
      if(ret == 0){
        ctrl8_xl.lpf2_xl_en = 0;
        ctrl8_xl.hp_slope_xl_en = 0;
        ret = lsm6dsl_write_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
      }
    }
  }
  return ret;
}









 
int32_t lsm6dsl_xl_lp1_bandwidth_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_lpf1_bw_sel_t *val)
{
  lsm6dsl_ctrl1_xl_t ctrl1_xl;
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  if(ret == 0){
    if ((ctrl8_xl.lpf2_xl_en != 0x00U) ||
        (ctrl8_xl.hp_slope_xl_en != 0x00U)){
      *val = LSM6DSL_XL_LP1_NA;
    }
    else{
      ret = lsm6dsl_read_reg(ctx, 0x10U, (uint8_t*)&ctrl1_xl, 1);
      switch ( ctrl1_xl.lpf1_bw_sel) {
        case LSM6DSL_XL_LP1_ODR_DIV_2:
          *val = LSM6DSL_XL_LP1_ODR_DIV_2;
          break;
        case LSM6DSL_XL_LP1_ODR_DIV_4:
          *val = LSM6DSL_XL_LP1_ODR_DIV_4;
          break;
        default:
          *val = LSM6DSL_XL_LP1_NA;
          break;
      }
    }
  }
  return ret;
}








 
int32_t lsm6dsl_xl_lp2_bandwidth_set(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_input_composite_t val)
{
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  if(ret == 0){
    ctrl8_xl.input_composite = ( (uint8_t) val & 0x10U ) >> 4;
    ctrl8_xl.hpcf_xl = (uint8_t) val & 0x03U;
    ctrl8_xl.lpf2_xl_en = 1;
    ctrl8_xl.hp_slope_xl_en = 0;
    ret = lsm6dsl_write_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_xl_lp2_bandwidth_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_input_composite_t *val)
{
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  if(ret == 0){
    if ((ctrl8_xl.lpf2_xl_en == 0x00U) ||
        (ctrl8_xl.hp_slope_xl_en != 0x00U)){
      *val = LSM6DSL_XL_LP_NA;
    }
    else{
      switch ((ctrl8_xl.input_composite << 4) + ctrl8_xl.hpcf_xl) {
        case LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_50:
          *val = LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_50;
          break;
        case LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_100:
          *val = LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_100;
          break;
        case LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9:
          *val = LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_9;
          break;
        case LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_400:
          *val = LSM6DSL_XL_LOW_LAT_LP_ODR_DIV_400;
          break;
        case LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_50:
          *val = LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_50;
          break;
        case LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100:
          *val = LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100;
          break;
        case LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_9:
          *val = LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_9;
          break;
        case LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_400:
          *val = LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_400;
          break;
        default:
          *val = LSM6DSL_XL_LP_NA;
          break;
      }
    }
  }

  return ret;
}








 
int32_t lsm6dsl_xl_reference_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  if(ret == 0){
    ctrl8_xl.hp_ref_mode = val;
    ret = lsm6dsl_write_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_xl_reference_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  *val = ctrl8_xl.hp_ref_mode;

  return ret;
}








 
int32_t lsm6dsl_xl_hp_bandwidth_set(lsm6dsl_ctx_t *ctx, lsm6dsl_hpcf_xl_t val)
{
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  if(ret == 0){
    ctrl8_xl.input_composite = 0;
    ctrl8_xl.hpcf_xl = (uint8_t)val & 0x03U;
    ctrl8_xl.hp_slope_xl_en = 1;
    ret = lsm6dsl_write_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_xl_hp_bandwidth_get(lsm6dsl_ctx_t *ctx, lsm6dsl_hpcf_xl_t *val)
{
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  if (ctrl8_xl.hp_slope_xl_en == 0x00U){
    *val = LSM6DSL_XL_HP_NA;
  }
  switch (ctrl8_xl.hpcf_xl) {
    case LSM6DSL_XL_HP_ODR_DIV_4:
      *val = LSM6DSL_XL_HP_ODR_DIV_4;
      break;
    case LSM6DSL_XL_HP_ODR_DIV_100:
      *val = LSM6DSL_XL_HP_ODR_DIV_100;
      break;
    case LSM6DSL_XL_HP_ODR_DIV_9:
      *val = LSM6DSL_XL_HP_ODR_DIV_9;
      break;
    case LSM6DSL_XL_HP_ODR_DIV_400:
      *val = LSM6DSL_XL_HP_ODR_DIV_400;
      break;
    default:
      *val = LSM6DSL_XL_HP_NA;
      break;
  }

  return ret;
}




 







 








 
int32_t lsm6dsl_gy_band_pass_set(lsm6dsl_ctx_t *ctx, lsm6dsl_lpf1_sel_g_t val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  lsm6dsl_ctrl6_c_t ctrl6_c;
  lsm6dsl_ctrl7_g_t ctrl7_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);
  if(ret == 0){
    ctrl7_g.hpm_g  =  ( (uint8_t)val & 0x30U ) >> 4;
    ctrl7_g.hp_en_g = ( (uint8_t)val & 0x80U ) >> 7;
    ret = lsm6dsl_write_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);
    if(ret == 0){
      ret = lsm6dsl_read_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
      if(ret == 0){
        ctrl6_c.ftype = (uint8_t)val & 0x03U;
        ret = lsm6dsl_write_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
        if(ret == 0){
          ret = lsm6dsl_read_reg(ctx, 0x13U,
                                 (uint8_t*)&ctrl4_c, 1);
          if(ret == 0){
            ctrl4_c.lpf1_sel_g = ( (uint8_t)val & 0x08U ) >> 3;
            ret = lsm6dsl_write_reg(ctx, 0x13U,
                                    (uint8_t*)&ctrl4_c, 1);
          }
        }
      }
    }
  }
  return ret;
}








 
int32_t lsm6dsl_gy_band_pass_get(lsm6dsl_ctx_t *ctx, lsm6dsl_lpf1_sel_g_t *val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  lsm6dsl_ctrl6_c_t ctrl6_c;
  lsm6dsl_ctrl7_g_t ctrl7_g;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
    if(ret == 0){
      ret = lsm6dsl_read_reg(ctx, 0x16U, (uint8_t*)&ctrl7_g, 1);

      switch ( ( ctrl7_g.hp_en_g << 7 ) + ( ctrl7_g.hpm_g << 4 ) +
               ( ctrl4_c.lpf1_sel_g << 3) + ctrl6_c.ftype ) {
        case LSM6DSL_HP_16mHz_LP2:
          *val = LSM6DSL_HP_16mHz_LP2;
          break;
        case LSM6DSL_HP_65mHz_LP2:
          *val = LSM6DSL_HP_65mHz_LP2;
          break;
        case LSM6DSL_HP_260mHz_LP2:
          *val = LSM6DSL_HP_260mHz_LP2;
          break;
        case LSM6DSL_HP_1Hz04_LP2:
          *val = LSM6DSL_HP_1Hz04_LP2;
          break;
        case LSM6DSL_HP_DISABLE_LP1_LIGHT:
          *val = LSM6DSL_HP_DISABLE_LP1_LIGHT;
          break;
        case LSM6DSL_HP_DISABLE_LP1_NORMAL:
          *val = LSM6DSL_HP_DISABLE_LP1_NORMAL;
          break;
        case LSM6DSL_HP_DISABLE_LP_STRONG:
          *val = LSM6DSL_HP_DISABLE_LP_STRONG;
          break;
        case LSM6DSL_HP_DISABLE_LP1_AGGRESSIVE:
          *val = LSM6DSL_HP_DISABLE_LP1_AGGRESSIVE;
          break;
        case LSM6DSL_HP_16mHz_LP1_LIGHT:
          *val = LSM6DSL_HP_16mHz_LP1_LIGHT;
          break;
        case LSM6DSL_HP_65mHz_LP1_NORMAL:
          *val = LSM6DSL_HP_65mHz_LP1_NORMAL;
          break;
        case LSM6DSL_HP_260mHz_LP1_STRONG:
          *val = LSM6DSL_HP_260mHz_LP1_STRONG;
          break;
        case LSM6DSL_HP_1Hz04_LP1_AGGRESSIVE:
          *val = LSM6DSL_HP_1Hz04_LP1_AGGRESSIVE;
          break;
        default:
          *val = LSM6DSL_HP_GY_BAND_NA;
          break;
      }
    }
  }

  return ret;
}




 







 








 
int32_t lsm6dsl_spi_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_sim_t val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.sim = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_spi_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_sim_t *val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  switch (ctrl3_c.sim) {
    case LSM6DSL_SPI_4_WIRE:
      *val = LSM6DSL_SPI_4_WIRE;
      break;
    case LSM6DSL_SPI_3_WIRE:
      *val = LSM6DSL_SPI_3_WIRE;
      break;
    default:
      *val = LSM6DSL_SPI_MODE_ND;
      break;
  }
  return ret;
}








 
int32_t lsm6dsl_i2c_interface_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_i2c_disable_t val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ctrl4_c.i2c_disable = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_i2c_interface_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_i2c_disable_t *val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  switch (ctrl4_c.i2c_disable) {
    case LSM6DSL_I2C_ENABLE:
      *val = LSM6DSL_I2C_ENABLE;
      break;
    case LSM6DSL_I2C_DISABLE:
      *val = LSM6DSL_I2C_DISABLE;
      break;
    default:
      *val = LSM6DSL_I2C_MODE_ND;
      break;
  }

  return ret;
}




 







 









 
int32_t lsm6dsl_pin_int1_route_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int1_route_t val)
{
  lsm6dsl_master_config_t master_config;
  lsm6dsl_int1_ctrl_t int1_ctrl;
  lsm6dsl_md1_cfg_t md1_cfg;
  lsm6dsl_md2_cfg_t md2_cfg;
  lsm6dsl_ctrl4_c_t ctrl4_c;
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0DU, (uint8_t*)&int1_ctrl, 1);
  if(ret == 0){
    int1_ctrl.int1_drdy_xl        = val.int1_drdy_xl;
    int1_ctrl.int1_drdy_g         = val.int1_drdy_g;
    int1_ctrl.int1_boot           = val.int1_boot;
    int1_ctrl.int1_fth            = val.int1_fth;
    int1_ctrl.int1_fifo_ovr       = val.int1_fifo_ovr;
    int1_ctrl.int1_full_flag      = val.int1_full_flag;
    int1_ctrl.int1_sign_mot       = val.int1_sign_mot;
    int1_ctrl.int1_step_detector  = val.int1_step_detector;
    ret = lsm6dsl_write_reg(ctx, 0x0DU, (uint8_t*)&int1_ctrl, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x5EU, (uint8_t*)&md1_cfg, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x5FU, (uint8_t*)&md2_cfg, 1);
  }
  if(ret == 0){
        md1_cfg.int1_timer           = val.int1_timer;
        md1_cfg.int1_tilt            = val.int1_tilt;
        md1_cfg.int1_6d              = val.int1_6d;
        md1_cfg.int1_double_tap      = val.int1_double_tap;
        md1_cfg.int1_ff              = val.int1_ff;
        md1_cfg.int1_wu              = val.int1_wu;
        md1_cfg.int1_single_tap      = val.int1_single_tap;
        md1_cfg.int1_inact_state     = val.int1_inact_state;
        ret = lsm6dsl_write_reg(ctx, 0x5EU, (uint8_t*)&md1_cfg, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  }
  if(ret == 0){
    ctrl4_c.den_drdy_int1 = val.den_drdy_int1;
    ret = lsm6dsl_write_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x1AU,
                           (uint8_t*)&master_config, 1);
  }
  if(ret == 0){
     master_config.drdy_on_int1   = val.den_drdy_int1;
     ret = lsm6dsl_write_reg(ctx, 0x1AU,
                             (uint8_t*)&master_config, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
    if ((val.int1_6d != 0x00U) ||
        (val.int1_ff != 0x00U) ||
        (val.int1_wu != 0x00U) ||
        (val.int1_single_tap != 0x00U) ||
        (val.int1_double_tap != 0x00U) ||
        (val.int1_inact_state != 0x00U)||
        (md2_cfg.int2_6d != 0x00U) ||
        (md2_cfg.int2_ff != 0x00U) ||
        (md2_cfg.int2_wu != 0x00U) ||
        (md2_cfg.int2_single_tap != 0x00U) ||
        (md2_cfg.int2_double_tap != 0x00U) ||
        (md2_cfg.int2_inact_state!= 0x00U) ){
      tap_cfg.interrupts_enable = (1U);
    }
    else{
      tap_cfg.interrupts_enable = (0U);
    }
  }
  if(ret == 0){
    ret = lsm6dsl_write_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_pin_int1_route_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int1_route_t *val)
{
  lsm6dsl_master_config_t master_config;
  lsm6dsl_int1_ctrl_t int1_ctrl;
  lsm6dsl_md1_cfg_t md1_cfg;
  lsm6dsl_ctrl4_c_t ctrl4_c;

  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0DU, (uint8_t*)&int1_ctrl, 1);
  if(ret == 0){
    val->int1_drdy_xl       = int1_ctrl.int1_drdy_xl;
    val->int1_drdy_g        = int1_ctrl.int1_drdy_g;
    val->int1_boot          = int1_ctrl.int1_boot;
    val->int1_fth           = int1_ctrl.int1_fth;
    val->int1_fifo_ovr      = int1_ctrl.int1_fifo_ovr;
    val->int1_full_flag     = int1_ctrl.int1_full_flag;
    val->int1_sign_mot      = int1_ctrl.int1_sign_mot;
    val->int1_step_detector = int1_ctrl.int1_step_detector ;

    ret = lsm6dsl_read_reg(ctx, 0x5EU, (uint8_t*)&md1_cfg, 1);
    if(ret == 0){
    val->int1_timer       = md1_cfg.int1_timer;
    val->int1_tilt        = md1_cfg.int1_tilt;
    val->int1_6d          = md1_cfg.int1_6d;
    val->int1_double_tap  = md1_cfg.int1_double_tap;
    val->int1_ff          = md1_cfg.int1_ff;
    val->int1_wu          = md1_cfg.int1_wu;
    val->int1_single_tap  = md1_cfg.int1_single_tap;
    val->int1_inact_state = md1_cfg.int1_inact_state;

    ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
      if(ret == 0){
        val->den_drdy_int1 = ctrl4_c.den_drdy_int1;
        ret = lsm6dsl_read_reg(ctx, 0x1AU,
                               (uint8_t*)&master_config, 1);
        val->den_drdy_int1 = master_config.drdy_on_int1;
      }
    }
  }
  return ret;
}








 
int32_t lsm6dsl_pin_int2_route_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_int2_route_t val)
{
  lsm6dsl_int2_ctrl_t int2_ctrl;
  lsm6dsl_md1_cfg_t md1_cfg;
  lsm6dsl_md2_cfg_t md2_cfg;
  lsm6dsl_drdy_pulse_cfg_g_t drdy_pulse_cfg_g;
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;


  ret = lsm6dsl_read_reg(ctx, 0x0EU, (uint8_t*)&int2_ctrl, 1);
  if(ret == 0){
    int2_ctrl.int2_drdy_xl        = val.int2_drdy_xl;
    int2_ctrl.int2_drdy_g         = val.int2_drdy_g;
    int2_ctrl.int2_drdy_temp      = val.int2_drdy_temp;
    int2_ctrl.int2_fth            = val.int2_fth;
    int2_ctrl.int2_fifo_ovr       = val.int2_fifo_ovr;
    int2_ctrl.int2_full_flag      = val.int2_full_flag;
    int2_ctrl.int2_step_count_ov  = val.int2_step_count_ov;
    int2_ctrl.int2_step_delta     = val.int2_step_delta;
    ret = lsm6dsl_write_reg(ctx, 0x0EU, (uint8_t*)&int2_ctrl, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x5EU, (uint8_t*)&md1_cfg, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x5FU, (uint8_t*)&md2_cfg, 1);
  }
  if(ret == 0){
    md2_cfg.int2_iron              = val.int2_iron;
    md2_cfg.int2_tilt              = val.int2_tilt;
    md2_cfg.int2_6d                = val.int2_6d;
    md2_cfg.int2_double_tap        = val.int2_double_tap;
    md2_cfg.int2_ff                = val.int2_ff;
    md2_cfg.int2_wu                = val.int2_wu;
    md2_cfg.int2_single_tap        = val.int2_single_tap;
    md2_cfg.int2_inact_state       = val.int2_inact_state;
    ret = lsm6dsl_write_reg(ctx, 0x5FU, (uint8_t*)&md2_cfg, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x0BU,
                           (uint8_t*)&drdy_pulse_cfg_g, 1);
  }
  if(ret == 0){
    drdy_pulse_cfg_g.int2_wrist_tilt = val.int2_wrist_tilt;
    ret = lsm6dsl_write_reg(ctx, 0x0BU,
                            (uint8_t*)&drdy_pulse_cfg_g, 1);
  }
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
    if ((md1_cfg.int1_6d != 0x00U) ||
        (md1_cfg.int1_ff != 0x00U) ||
        (md1_cfg.int1_wu != 0x00U) ||
        (md1_cfg.int1_single_tap != 0x00U) ||
        (md1_cfg.int1_double_tap != 0x00U) ||
        (md1_cfg.int1_inact_state != 0x00U) ||
        (val.int2_6d != 0x00U) ||
        (val.int2_ff != 0x00U) ||
        (val.int2_wu != 0x00U) ||
        (val.int2_single_tap != 0x00U) ||
        (val.int2_double_tap != 0x00U) ||
        (val.int2_inact_state!= 0x00U) ){
      tap_cfg.interrupts_enable = (1U);
    }
    else{
      tap_cfg.interrupts_enable = (0U);
    }
  }
  if(ret == 0){
    ret = lsm6dsl_write_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_pin_int2_route_get(lsm6dsl_ctx_t *ctx,
lsm6dsl_int2_route_t *val)
{
  lsm6dsl_int2_ctrl_t int2_ctrl;
  lsm6dsl_md2_cfg_t md2_cfg;
  lsm6dsl_drdy_pulse_cfg_g_t drdy_pulse_cfg_g;

  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0EU, (uint8_t*)&int2_ctrl, 1);
  if(ret == 0){
    val->int2_drdy_xl         = int2_ctrl.int2_drdy_xl;
    val->int2_drdy_g          = int2_ctrl.int2_drdy_g;
    val->int2_drdy_temp       = int2_ctrl.int2_drdy_temp;
    val->int2_fth             = int2_ctrl.int2_fth;
    val->int2_fifo_ovr        = int2_ctrl.int2_fifo_ovr;
    val->int2_full_flag       = int2_ctrl.int2_full_flag;
    val->int2_step_count_ov   = int2_ctrl.int2_step_count_ov;
    val->int2_step_delta      = int2_ctrl.int2_step_delta;

    ret = lsm6dsl_read_reg(ctx, 0x5FU, (uint8_t*)&md2_cfg, 1);
    if(ret == 0){
      val->int2_iron           = md2_cfg.int2_iron;
      val->int2_tilt           = md2_cfg.int2_tilt;
      val->int2_6d             = md2_cfg.int2_6d;
      val->int2_double_tap     = md2_cfg.int2_double_tap;
      val->int2_ff             = md2_cfg.int2_ff;
      val->int2_wu             = md2_cfg.int2_wu;
      val->int2_single_tap     = md2_cfg.int2_single_tap;
      val->int2_inact_state    = md2_cfg.int2_inact_state;

      ret = lsm6dsl_read_reg(ctx, 0x0BU,
                             (uint8_t*)&drdy_pulse_cfg_g, 1);
      val->int2_wrist_tilt = drdy_pulse_cfg_g.int2_wrist_tilt;
    }
  }
  return ret;
}








 
int32_t lsm6dsl_pin_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_pp_od_t val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.pp_od = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_pin_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_pp_od_t *val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  switch (ctrl3_c.pp_od) {
    case LSM6DSL_PUSH_PULL:
      *val = LSM6DSL_PUSH_PULL;
      break;
    case LSM6DSL_OPEN_DRAIN:
      *val = LSM6DSL_OPEN_DRAIN;
      break;
    default:
      *val = LSM6DSL_PIN_MODE_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_pin_polarity_set(lsm6dsl_ctx_t *ctx, lsm6dsl_h_lactive_t val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  if(ret == 0){
    ctrl3_c.h_lactive = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_pin_polarity_get(lsm6dsl_ctx_t *ctx, lsm6dsl_h_lactive_t *val)
{
  lsm6dsl_ctrl3_c_t ctrl3_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x12U, (uint8_t*)&ctrl3_c, 1);
  switch (ctrl3_c.h_lactive) {
    case LSM6DSL_ACTIVE_HIGH:
      *val = LSM6DSL_ACTIVE_HIGH;
      break;
    case LSM6DSL_ACTIVE_LOW:
      *val = LSM6DSL_ACTIVE_LOW;
      break;
    default:
      *val = LSM6DSL_POLARITY_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_all_on_int1_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ctrl4_c.int2_on_int1 = val;
    ret = lsm6dsl_write_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_all_on_int1_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  *val = ctrl4_c.int2_on_int1;

  return ret;
}








 
int32_t lsm6dsl_int_notification_set(lsm6dsl_ctx_t *ctx, lsm6dsl_lir_t val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  if(ret == 0){
    tap_cfg.lir = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_int_notification_get(lsm6dsl_ctx_t *ctx, lsm6dsl_lir_t *val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  switch (tap_cfg.lir) {
    case LSM6DSL_INT_PULSED:
      *val = LSM6DSL_INT_PULSED;
      break;
    case LSM6DSL_INT_LATCHED:
      *val = LSM6DSL_INT_LATCHED;
      break;
    default:
      *val = LSM6DSL_INT_MODE;
      break;
  }

  return ret;
}




 







 








 
int32_t lsm6dsl_wkup_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5BU, (uint8_t*)&wake_up_ths, 1);
  if(ret == 0){
    wake_up_ths.wk_ths = val;
    ret = lsm6dsl_write_reg(ctx, 0x5BU,
                            (uint8_t*)&wake_up_ths, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_wkup_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5BU, (uint8_t*)&wake_up_ths, 1);
  *val = wake_up_ths.wk_ths;

  return ret;
}








 
int32_t lsm6dsl_wkup_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5CU, (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    wake_up_dur.wake_dur = val;
    ret = lsm6dsl_write_reg(ctx, 0x5CU,
                            (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_wkup_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5CU, (uint8_t*)&wake_up_dur, 1);
  *val = wake_up_dur.wake_dur;

  return ret;
}




 







 








 
int32_t lsm6dsl_gy_sleep_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ctrl4_c.sleep = val;
    ret = lsm6dsl_write_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_gy_sleep_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  *val = ctrl4_c.sleep;

  return ret;
}








 
int32_t lsm6dsl_act_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_inact_en_t val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  if(ret == 0){
    tap_cfg.inact_en = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_act_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_inact_en_t *val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  switch (tap_cfg.inact_en) {
    case LSM6DSL_PROPERTY_DISABLE:
      *val = LSM6DSL_PROPERTY_DISABLE;
      break;
    case LSM6DSL_XL_12Hz5_GY_NOT_AFFECTED:
      *val = LSM6DSL_XL_12Hz5_GY_NOT_AFFECTED;
      break;
    case LSM6DSL_XL_12Hz5_GY_SLEEP:
      *val = LSM6DSL_XL_12Hz5_GY_SLEEP;
      break;
    case LSM6DSL_XL_12Hz5_GY_PD:
      *val = LSM6DSL_XL_12Hz5_GY_PD;
      break;
    default:
      *val = LSM6DSL_ACT_MODE_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_act_sleep_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5CU, (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    wake_up_dur.sleep_dur = val;
    ret = lsm6dsl_write_reg(ctx, 0x5CU,
          (uint8_t*)&wake_up_dur, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_act_sleep_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_wake_up_dur_t wake_up_dur;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5CU, (uint8_t*)&wake_up_dur, 1);
  *val = wake_up_dur.sleep_dur;

  return ret;
}




 







 








 
int32_t lsm6dsl_tap_src_get(lsm6dsl_ctx_t *ctx, lsm6dsl_tap_src_t *val)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x1CU, (uint8_t*) val, 1);
  return ret;
}







 
int32_t lsm6dsl_tap_detection_on_z_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  if(ret == 0){
    tap_cfg.tap_z_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_tap_detection_on_z_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  *val = tap_cfg.tap_z_en;

  return ret;
}








 
int32_t lsm6dsl_tap_detection_on_y_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  if(ret == 0){
    tap_cfg.tap_y_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_tap_detection_on_y_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  *val = tap_cfg.tap_y_en;

  return ret;
}








 
int32_t lsm6dsl_tap_detection_on_x_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  if(ret == 0){
    tap_cfg.tap_x_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_tap_detection_on_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_tap_cfg_t tap_cfg;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x58U, (uint8_t*)&tap_cfg, 1);
  *val = tap_cfg.tap_x_en;

  return ret;
}








 
int32_t lsm6dsl_tap_threshold_x_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_tap_ths_6d_t tap_ths_6d;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x59U, (uint8_t*)&tap_ths_6d, 1);
  if(ret == 0){
    tap_ths_6d.tap_ths = val;
    ret = lsm6dsl_write_reg(ctx, 0x59U,
                            (uint8_t*)&tap_ths_6d, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_tap_threshold_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_tap_ths_6d_t tap_ths_6d;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x59U, (uint8_t*)&tap_ths_6d, 1);
  *val = tap_ths_6d.tap_ths;

  return ret;
}













 
int32_t lsm6dsl_tap_shock_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_int_dur2_t int_dur2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  if(ret == 0){
    int_dur2.shock = val;
    ret = lsm6dsl_write_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  }
  return ret;
}













 
int32_t lsm6dsl_tap_shock_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_int_dur2_t int_dur2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  *val = int_dur2.shock;

  return ret;
}













 
int32_t lsm6dsl_tap_quiet_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_int_dur2_t int_dur2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  if(ret == 0){
    int_dur2.quiet = val;
    ret = lsm6dsl_write_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  }
  return ret;
}













 
int32_t lsm6dsl_tap_quiet_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_int_dur2_t int_dur2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  *val = int_dur2.quiet;

  return ret;
}














 
int32_t lsm6dsl_tap_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_int_dur2_t int_dur2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  if(ret == 0){
    int_dur2.dur = val;
    ret = lsm6dsl_write_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  }
  return ret;
}














 
int32_t lsm6dsl_tap_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_int_dur2_t int_dur2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5AU, (uint8_t*)&int_dur2, 1);
  *val = int_dur2.dur;

  return ret;
}









 
int32_t lsm6dsl_tap_mode_set(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_single_double_tap_t val)
{
  lsm6dsl_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5BU, (uint8_t*)&wake_up_ths, 1);
  if(ret == 0){
    wake_up_ths.single_double_tap = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x5BU,
                            (uint8_t*)&wake_up_ths, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_tap_mode_get(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_single_double_tap_t *val)
{
  lsm6dsl_wake_up_ths_t wake_up_ths;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5BU, (uint8_t*)&wake_up_ths, 1);
  switch (wake_up_ths.single_double_tap) {
    case LSM6DSL_ONLY_SINGLE:
      *val = LSM6DSL_ONLY_SINGLE;
      break;
    case LSM6DSL_BOTH_SINGLE_DOUBLE:
      *val = LSM6DSL_BOTH_SINGLE_DOUBLE;
      break;
    default:
      *val = LSM6DSL_TAP_MODE_ND;
      break;
  }

  return ret;
}




 







 









 
int32_t lsm6dsl_6d_feed_data_set(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_low_pass_on_6d_t val)
{
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  if(ret == 0){
    ctrl8_xl.low_pass_on_6d = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_6d_feed_data_get(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_low_pass_on_6d_t *val)
{
  lsm6dsl_ctrl8_xl_t ctrl8_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x17U, (uint8_t*)&ctrl8_xl, 1);
  switch (ctrl8_xl.low_pass_on_6d) {
    case LSM6DSL_ODR_DIV_2_FEED:
      *val = LSM6DSL_ODR_DIV_2_FEED;
      break;
    case LSM6DSL_LPF2_FEED:
      *val = LSM6DSL_LPF2_FEED;
      break;
    default:
      *val = LSM6DSL_6D_FEED_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_6d_threshold_set(lsm6dsl_ctx_t *ctx, lsm6dsl_sixd_ths_t val)
{
  lsm6dsl_tap_ths_6d_t tap_ths_6d;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x59U, (uint8_t*)&tap_ths_6d, 1);
  if(ret == 0){
    tap_ths_6d.sixd_ths = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x59U, (uint8_t*)&tap_ths_6d, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_6d_threshold_get(lsm6dsl_ctx_t *ctx, lsm6dsl_sixd_ths_t *val)
{
  lsm6dsl_tap_ths_6d_t tap_ths_6d;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x59U, (uint8_t*)&tap_ths_6d, 1);
  switch (tap_ths_6d.sixd_ths) {
    case LSM6DSL_DEG_80:
      *val = LSM6DSL_DEG_80;
      break;
    case LSM6DSL_DEG_70:
      *val = LSM6DSL_DEG_70;
      break;
    case LSM6DSL_DEG_60:
      *val = LSM6DSL_DEG_60;
      break;
    case LSM6DSL_DEG_50:
      *val = LSM6DSL_DEG_50;
      break;
    default:
      *val = LSM6DSL_6D_TH_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_4d_mode_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_tap_ths_6d_t tap_ths_6d;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x59U, (uint8_t*)&tap_ths_6d, 1);
  if(ret == 0){
    tap_ths_6d.d4d_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x59U,
                            (uint8_t*)&tap_ths_6d, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_4d_mode_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_tap_ths_6d_t tap_ths_6d;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x59U, (uint8_t*)&tap_ths_6d, 1);
  *val = tap_ths_6d.d4d_en;

  return ret;
}




 







 








 
int32_t lsm6dsl_ff_dur_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_wake_up_dur_t wake_up_dur;
  lsm6dsl_free_fall_t free_fall;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5DU, (uint8_t*)&free_fall, 1);
  if(ret == 0){
    free_fall.ff_dur = (val & 0x1FU);
    ret = lsm6dsl_write_reg(ctx, 0x5DU, (uint8_t*)&free_fall, 1);
    if(ret == 0){
      ret = lsm6dsl_read_reg(ctx, 0x5CU,
                             (uint8_t*)&wake_up_dur, 1);
      if(ret == 0){
        wake_up_dur.ff_dur = (val & 0x20U) >> 5;
        ret = lsm6dsl_write_reg(ctx, 0x5CU,
                                (uint8_t*)&wake_up_dur, 1);
      }
    }
  }
  return ret;
}








 
int32_t lsm6dsl_ff_dur_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_wake_up_dur_t wake_up_dur;
  lsm6dsl_free_fall_t free_fall;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5CU, (uint8_t*)&wake_up_dur, 1);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x5DU, (uint8_t*)&free_fall, 1);
  }
  *val = (wake_up_dur.ff_dur << 5) + free_fall.ff_dur;

  return ret;
}








 
int32_t lsm6dsl_ff_threshold_set(lsm6dsl_ctx_t *ctx, lsm6dsl_ff_ths_t val)
{
  lsm6dsl_free_fall_t free_fall;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5DU, (uint8_t*)&free_fall, 1);
  if(ret == 0){
    free_fall.ff_ths = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x5DU, (uint8_t*)&free_fall, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_ff_threshold_get(lsm6dsl_ctx_t *ctx, lsm6dsl_ff_ths_t *val)
{
  lsm6dsl_free_fall_t free_fall;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x5DU, (uint8_t*)&free_fall, 1);
  switch (free_fall.ff_ths) {
    case LSM6DSL_FF_TSH_156mg:
      *val = LSM6DSL_FF_TSH_156mg;
      break;
    case LSM6DSL_FF_TSH_219mg:
      *val = LSM6DSL_FF_TSH_219mg;
      break;
    case LSM6DSL_FF_TSH_250mg:
      *val = LSM6DSL_FF_TSH_250mg;
      break;
    case LSM6DSL_FF_TSH_312mg:
      *val = LSM6DSL_FF_TSH_312mg;
      break;
    case LSM6DSL_FF_TSH_344mg:
      *val = LSM6DSL_FF_TSH_344mg;
      break;
    case LSM6DSL_FF_TSH_406mg:
      *val = LSM6DSL_FF_TSH_406mg;
      break;
    case LSM6DSL_FF_TSH_469mg:
      *val = LSM6DSL_FF_TSH_469mg;
      break;
    case LSM6DSL_FF_TSH_500mg:
      *val = LSM6DSL_FF_TSH_500mg;
      break;
    default:
      *val = LSM6DSL_FF_TSH_ND;
      break;
  }

  return ret;
}




 







 








 
int32_t lsm6dsl_fifo_watermark_set(lsm6dsl_ctx_t *ctx, uint16_t val)
{
  lsm6dsl_fifo_ctrl1_t fifo_ctrl1;
  lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x07U, (uint8_t*)&fifo_ctrl2, 1);
  if(ret == 0){
    fifo_ctrl1.fth = (uint8_t) (0x00FFU & val);
    fifo_ctrl2.fth = (uint8_t) (( 0x0700U & val ) >> 8);
    ret = lsm6dsl_write_reg(ctx, 0x06U, (uint8_t*)&fifo_ctrl1, 1);
    if(ret == 0){
      ret = lsm6dsl_write_reg(ctx, 0x07U,
                              (uint8_t*)&fifo_ctrl2, 1);
    }
  }
  return ret;
}








 
int32_t lsm6dsl_fifo_watermark_get(lsm6dsl_ctx_t *ctx, uint16_t *val)
{
  lsm6dsl_fifo_ctrl1_t fifo_ctrl1;
  lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x06U, (uint8_t*)&fifo_ctrl1, 1);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x07U, (uint8_t*)&fifo_ctrl2, 1);
  }
  *val = ((uint16_t)fifo_ctrl2.fth << 8) + (uint16_t)fifo_ctrl1.fth;

  return ret;
}










 
int32_t lsm6dsl_fifo_data_level_get(lsm6dsl_ctx_t *ctx, uint16_t *val)
{
  lsm6dsl_fifo_status1_t fifo_status1;
  lsm6dsl_fifo_status2_t fifo_status2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x3AU,
                         (uint8_t*)&fifo_status1, 1);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x3BU,
                           (uint8_t*)&fifo_status2, 1);
    *val = ( (uint16_t) fifo_status2.diff_fifo << 8) +
             (uint16_t) fifo_status1.diff_fifo;
  }

  return ret;
}








 
int32_t lsm6dsl_fifo_wtm_flag_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_fifo_status2_t fifo_status2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x3BU, (uint8_t*)&fifo_status2, 1);
  *val = fifo_status2.waterm;

  return ret;
}









 
int32_t lsm6dsl_fifo_pattern_get(lsm6dsl_ctx_t *ctx, uint16_t *val)
{
  lsm6dsl_fifo_status3_t fifo_status3;
  lsm6dsl_fifo_status4_t fifo_status4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x3CU,
                         (uint8_t*)&fifo_status3, 1);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x3DU,
                           (uint8_t*)&fifo_status4, 1);
    *val = ( (uint16_t)fifo_status4.fifo_pattern << 8) +
             fifo_status3.fifo_pattern;
  }
  return ret;
}








 
int32_t lsm6dsl_fifo_temp_batch_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x07U, (uint8_t*)&fifo_ctrl2, 1);
  if(ret == 0){
    fifo_ctrl2.fifo_temp_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x07U,
                            (uint8_t*)&fifo_ctrl2, 1);
  }

  return ret;
}








 
int32_t lsm6dsl_fifo_temp_batch_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x07U, (uint8_t*)&fifo_ctrl2, 1);
  *val = fifo_ctrl2.fifo_temp_en;

  return ret;
}









 
int32_t lsm6dsl_fifo_write_trigger_set(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_trigger_fifo_t val)
{
  lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x07U, (uint8_t*)&fifo_ctrl2, 1);
  if(ret == 0){
    fifo_ctrl2.timer_pedo_fifo_drdy = (uint8_t)val & 0x01U;
    ret = lsm6dsl_write_reg(ctx, 0x07U,
                            (uint8_t*)&fifo_ctrl2, 1);
    if(ret == 0){
      ret = lsm6dsl_read_reg(ctx, 0x1AU,
                             (uint8_t*)&master_config, 1);
      if(ret == 0){
        master_config.data_valid_sel_fifo = (((uint8_t)val & 0x02U) >> 1);
        ret = lsm6dsl_write_reg(ctx, 0x1AU,
                                (uint8_t*)&master_config, 1);
      }
    }
  }

  return ret;
}









 
int32_t lsm6dsl_fifo_write_trigger_get(lsm6dsl_ctx_t *ctx,
                                       lsm6dsl_trigger_fifo_t *val)
{
  lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x07U, (uint8_t*)&fifo_ctrl2, 1);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x1AU,
                           (uint8_t*)&master_config, 1);

    switch ( ( fifo_ctrl2.timer_pedo_fifo_drdy << 1 ) +
             fifo_ctrl2. timer_pedo_fifo_drdy ) {
      case LSM6DSL_TRG_XL_GY_DRDY:
        *val = LSM6DSL_TRG_XL_GY_DRDY;
        break;
      case LSM6DSL_TRG_STEP_DETECT:
        *val = LSM6DSL_TRG_STEP_DETECT;
        break;
      case LSM6DSL_TRG_SH_DRDY:
        *val = LSM6DSL_TRG_SH_DRDY;
        break;
      default:
        *val = LSM6DSL_TRG_SH_ND;
        break;
    }
  }

  return ret;
}









 
int32_t lsm6dsl_fifo_pedo_and_timestamp_batch_set(lsm6dsl_ctx_t *ctx,
                                                  uint8_t val)
{
  lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x07U, (uint8_t*)&fifo_ctrl2, 1);
  if(ret == 0){
    fifo_ctrl2.timer_pedo_fifo_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x07U,
                            (uint8_t*)&fifo_ctrl2, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_fifo_pedo_and_timestamp_batch_get(lsm6dsl_ctx_t *ctx,
                                                  uint8_t *val)
{
  lsm6dsl_fifo_ctrl2_t fifo_ctrl2;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x07U, (uint8_t*)&fifo_ctrl2, 1);
  *val = fifo_ctrl2.timer_pedo_fifo_en;

  return ret;
}









 
int32_t lsm6dsl_fifo_xl_batch_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_xl_t val)
{
  lsm6dsl_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x08U, (uint8_t*)&fifo_ctrl3, 1);
  if(ret == 0){
    fifo_ctrl3.dec_fifo_xl = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x08U,
                            (uint8_t*)&fifo_ctrl3, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_fifo_xl_batch_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_xl_t *val)
{
  lsm6dsl_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x08U, (uint8_t*)&fifo_ctrl3, 1);
  switch (fifo_ctrl3.dec_fifo_xl) {
    case LSM6DSL_FIFO_XL_DISABLE:
      *val = LSM6DSL_FIFO_XL_DISABLE;
      break;
    case LSM6DSL_FIFO_XL_NO_DEC:
      *val = LSM6DSL_FIFO_XL_NO_DEC;
      break;
    case LSM6DSL_FIFO_XL_DEC_2:
      *val = LSM6DSL_FIFO_XL_DEC_2;
      break;
    case LSM6DSL_FIFO_XL_DEC_3:
      *val = LSM6DSL_FIFO_XL_DEC_3;
      break;
    case LSM6DSL_FIFO_XL_DEC_4:
      *val = LSM6DSL_FIFO_XL_DEC_4;
      break;
    case LSM6DSL_FIFO_XL_DEC_8:
      *val = LSM6DSL_FIFO_XL_DEC_8;
      break;
    case LSM6DSL_FIFO_XL_DEC_16:
      *val = LSM6DSL_FIFO_XL_DEC_16;
      break;
    case LSM6DSL_FIFO_XL_DEC_32:
      *val = LSM6DSL_FIFO_XL_DEC_32;
      break;
    default:
      *val = LSM6DSL_FIFO_XL_DEC_ND;
      break;
  }

  return ret;
}









 
int32_t lsm6dsl_fifo_gy_batch_set(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_gyro_t val)
{
  lsm6dsl_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x08U, (uint8_t*)&fifo_ctrl3, 1);
  if(ret == 0){
    fifo_ctrl3.dec_fifo_gyro = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x08U, (uint8_t*)&fifo_ctrl3, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_fifo_gy_batch_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_dec_fifo_gyro_t *val)
{
  lsm6dsl_fifo_ctrl3_t fifo_ctrl3;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x08U, (uint8_t*)&fifo_ctrl3, 1);
  switch (fifo_ctrl3.dec_fifo_gyro) {
    case LSM6DSL_FIFO_GY_DISABLE:
      *val = LSM6DSL_FIFO_GY_DISABLE;
      break;
    case LSM6DSL_FIFO_GY_NO_DEC:
      *val = LSM6DSL_FIFO_GY_NO_DEC;
      break;
    case LSM6DSL_FIFO_GY_DEC_2:
      *val = LSM6DSL_FIFO_GY_DEC_2;
      break;
    case LSM6DSL_FIFO_GY_DEC_3:
      *val = LSM6DSL_FIFO_GY_DEC_3;
      break;
    case LSM6DSL_FIFO_GY_DEC_4:
      *val = LSM6DSL_FIFO_GY_DEC_4;
      break;
    case LSM6DSL_FIFO_GY_DEC_8:
      *val = LSM6DSL_FIFO_GY_DEC_8;
      break;
    case LSM6DSL_FIFO_GY_DEC_16:
      *val = LSM6DSL_FIFO_GY_DEC_16;
      break;
    case LSM6DSL_FIFO_GY_DEC_32:
      *val = LSM6DSL_FIFO_GY_DEC_32;
      break;
    default:
      *val = LSM6DSL_FIFO_GY_DEC_ND;
      break;
  }

  return ret;
}









 
int32_t lsm6dsl_fifo_dataset_3_batch_set(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds3_fifo_t val)
{
  lsm6dsl_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  if(ret == 0){
    fifo_ctrl4.dec_ds3_fifo = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x09U,
                            (uint8_t*)&fifo_ctrl4, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_fifo_dataset_3_batch_get(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds3_fifo_t *val)
{
  lsm6dsl_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  switch (fifo_ctrl4.dec_ds3_fifo) {
    case LSM6DSL_FIFO_DS3_DISABLE:
      *val = LSM6DSL_FIFO_DS3_DISABLE;
      break;
    case LSM6DSL_FIFO_DS3_NO_DEC:
      *val = LSM6DSL_FIFO_DS3_NO_DEC;
      break;
    case LSM6DSL_FIFO_DS3_DEC_2:
      *val = LSM6DSL_FIFO_DS3_DEC_2;
      break;
    case LSM6DSL_FIFO_DS3_DEC_3:
      *val = LSM6DSL_FIFO_DS3_DEC_3;
      break;
    case LSM6DSL_FIFO_DS3_DEC_4:
      *val = LSM6DSL_FIFO_DS3_DEC_4;
      break;
    case LSM6DSL_FIFO_DS3_DEC_8:
      *val = LSM6DSL_FIFO_DS3_DEC_8;
      break;
    case LSM6DSL_FIFO_DS3_DEC_16:
      *val = LSM6DSL_FIFO_DS3_DEC_16;
      break;
    case LSM6DSL_FIFO_DS3_DEC_32:
      *val = LSM6DSL_FIFO_DS3_DEC_32;
      break;
    default:
      *val = LSM6DSL_FIFO_DS3_DEC_ND;
      break;
  }

  return ret;
}









 
int32_t lsm6dsl_fifo_dataset_4_batch_set(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds4_fifo_t val)
{
  lsm6dsl_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  if(ret == 0){
    fifo_ctrl4.dec_ds4_fifo = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x09U,
                            (uint8_t*)&fifo_ctrl4, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_fifo_dataset_4_batch_get(lsm6dsl_ctx_t *ctx,
                                         lsm6dsl_dec_ds4_fifo_t *val)
{
  lsm6dsl_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  switch (fifo_ctrl4.dec_ds4_fifo) {
    case LSM6DSL_FIFO_DS4_DISABLE:
      *val = LSM6DSL_FIFO_DS4_DISABLE;
      break;
    case LSM6DSL_FIFO_DS4_NO_DEC:
      *val = LSM6DSL_FIFO_DS4_NO_DEC;
      break;
    case LSM6DSL_FIFO_DS4_DEC_2:
      *val = LSM6DSL_FIFO_DS4_DEC_2;
      break;
    case LSM6DSL_FIFO_DS4_DEC_3:
      *val = LSM6DSL_FIFO_DS4_DEC_3;
      break;
    case LSM6DSL_FIFO_DS4_DEC_4:
      *val = LSM6DSL_FIFO_DS4_DEC_4;
      break;
    case LSM6DSL_FIFO_DS4_DEC_8:
      *val = LSM6DSL_FIFO_DS4_DEC_8;
      break;
    case LSM6DSL_FIFO_DS4_DEC_16:
      *val = LSM6DSL_FIFO_DS4_DEC_16;
      break;
    case LSM6DSL_FIFO_DS4_DEC_32:
      *val = LSM6DSL_FIFO_DS4_DEC_32;
      break;
    default:
      *val = LSM6DSL_FIFO_DS4_DEC_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_fifo_xl_gy_8bit_format_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  if(ret == 0){
    fifo_ctrl4.only_high_data = val;
    ret = lsm6dsl_write_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_fifo_xl_gy_8bit_format_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  *val = fifo_ctrl4.only_high_data;

  return ret;
}









 
int32_t lsm6dsl_fifo_stop_on_wtm_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  if(ret == 0){
    fifo_ctrl4.stop_on_fth = val;
    ret = lsm6dsl_write_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_fifo_stop_on_wtm_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_fifo_ctrl4_t fifo_ctrl4;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x09U, (uint8_t*)&fifo_ctrl4, 1);
  *val = fifo_ctrl4.stop_on_fth;

  return ret;
}








 
int32_t lsm6dsl_fifo_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_fifo_mode_t val)
{
  lsm6dsl_fifo_ctrl5_t fifo_ctrl5;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0AU, (uint8_t*)&fifo_ctrl5, 1);
  if(ret == 0){
    fifo_ctrl5.fifo_mode = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x0AU, (uint8_t*)&fifo_ctrl5, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_fifo_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_fifo_mode_t *val)
{
  lsm6dsl_fifo_ctrl5_t fifo_ctrl5;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0AU, (uint8_t*)&fifo_ctrl5, 1);
  switch (fifo_ctrl5.fifo_mode) {
    case LSM6DSL_BYPASS_MODE:
      *val = LSM6DSL_BYPASS_MODE;
      break;
    case LSM6DSL_FIFO_MODE:
      *val = LSM6DSL_FIFO_MODE;
      break;
    case LSM6DSL_STREAM_TO_FIFO_MODE:
      *val = LSM6DSL_STREAM_TO_FIFO_MODE;
      break;
    case LSM6DSL_BYPASS_TO_STREAM_MODE:
      *val = LSM6DSL_BYPASS_TO_STREAM_MODE;
      break;
    case LSM6DSL_STREAM_MODE:
      *val = LSM6DSL_STREAM_MODE;
      break;
    default:
      *val = LSM6DSL_FIFO_MODE_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_fifo_data_rate_set(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_fifo_t val)
{
  lsm6dsl_fifo_ctrl5_t fifo_ctrl5;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0AU, (uint8_t*)&fifo_ctrl5, 1);
  if(ret == 0){
    fifo_ctrl5.odr_fifo = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x0AU, (uint8_t*)&fifo_ctrl5, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_fifo_data_rate_get(lsm6dsl_ctx_t *ctx, lsm6dsl_odr_fifo_t *val)
{
  lsm6dsl_fifo_ctrl5_t fifo_ctrl5;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x0AU, (uint8_t*)&fifo_ctrl5, 1);
  switch (fifo_ctrl5.odr_fifo) {
    case LSM6DSL_FIFO_DISABLE:
      *val = LSM6DSL_FIFO_DISABLE;
      break;
    case LSM6DSL_FIFO_12Hz5:
      *val = LSM6DSL_FIFO_12Hz5;
      break;
    case LSM6DSL_FIFO_26Hz:
      *val = LSM6DSL_FIFO_26Hz;
      break;
    case LSM6DSL_FIFO_52Hz:
      *val = LSM6DSL_FIFO_52Hz;
      break;
    case LSM6DSL_FIFO_104Hz:
      *val = LSM6DSL_FIFO_104Hz;
      break;
    case LSM6DSL_FIFO_208Hz:
      *val = LSM6DSL_FIFO_208Hz;
      break;
    case LSM6DSL_FIFO_416Hz:
      *val = LSM6DSL_FIFO_416Hz;
      break;
    case LSM6DSL_FIFO_833Hz:
      *val = LSM6DSL_FIFO_833Hz;
      break;
    case LSM6DSL_FIFO_1k66Hz:
      *val = LSM6DSL_FIFO_1k66Hz;
      break;
    case LSM6DSL_FIFO_3k33Hz:
      *val = LSM6DSL_FIFO_3k33Hz;
      break;
    case LSM6DSL_FIFO_6k66Hz:
      *val = LSM6DSL_FIFO_6k66Hz;
      break;
    default:
      *val = LSM6DSL_FIFO_RATE_ND;
      break;
  }

  return ret;
}




 







 








 
 int32_t lsm6dsl_den_polarity_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_lh_t val)
{
  lsm6dsl_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  if(ret == 0){
    ctrl5_c.den_lh = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_den_polarity_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_lh_t *val)
{
  lsm6dsl_ctrl5_c_t ctrl5_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x14U, (uint8_t*)&ctrl5_c, 1);
  switch (ctrl5_c.den_lh) {
    case LSM6DSL_DEN_ACT_LOW:
      *val = LSM6DSL_DEN_ACT_LOW;
      break;
    case LSM6DSL_DEN_ACT_HIGH:
      *val = LSM6DSL_DEN_ACT_HIGH;
      break;
    default:
      *val = LSM6DSL_DEN_POL_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_den_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_mode_t val)
{
  lsm6dsl_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  if(ret == 0){
    ctrl6_c.den_mode = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_den_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_mode_t *val)
{
  lsm6dsl_ctrl6_c_t ctrl6_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x15U, (uint8_t*)&ctrl6_c, 1);
  switch (ctrl6_c.den_mode) {
    case LSM6DSL_DEN_DISABLE:
      *val = LSM6DSL_DEN_DISABLE;
      break;
    case LSM6DSL_LEVEL_LETCHED:
      *val = LSM6DSL_LEVEL_LETCHED;
      break;
    case LSM6DSL_LEVEL_TRIGGER:
      *val = LSM6DSL_LEVEL_TRIGGER;
      break;
    case LSM6DSL_EDGE_TRIGGER:
      *val = LSM6DSL_EDGE_TRIGGER;
      break;
    default:
      *val = LSM6DSL_DEN_MODE_ND;
      break;
  }

  return ret;
}









 
int32_t lsm6dsl_den_enable_set(lsm6dsl_ctx_t *ctx, lsm6dsl_den_xl_en_t val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  if(ret == 0){
    ctrl9_xl.den_xl_g = (uint8_t)val & 0x01U;
    ret = lsm6dsl_write_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
    if(ret == 0){
      ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
      if(ret == 0){
        ctrl4_c.den_xl_en = (uint8_t)val & 0x02U;
        ret = lsm6dsl_write_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
      }
    }
  }
  return ret;
}









 
int32_t lsm6dsl_den_enable_get(lsm6dsl_ctx_t *ctx, lsm6dsl_den_xl_en_t *val)
{
  lsm6dsl_ctrl4_c_t ctrl4_c;
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x13U, (uint8_t*)&ctrl4_c, 1);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
    switch ( ( ctrl4_c.den_xl_en << 1) + ctrl9_xl.den_xl_g ) {
      case LSM6DSL_STAMP_IN_GY_DATA:
        *val = LSM6DSL_STAMP_IN_GY_DATA;
        break;
      case LSM6DSL_STAMP_IN_XL_DATA:
        *val = LSM6DSL_STAMP_IN_XL_DATA;
        break;
      case LSM6DSL_STAMP_IN_GY_XL_DATA:
        *val = LSM6DSL_STAMP_IN_GY_XL_DATA;
        break;
      default:
        *val = LSM6DSL_DEN_STAMP_ND;
        break;
    }
  }

  return ret;
}








 
int32_t lsm6dsl_den_mark_axis_z_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  if(ret == 0){
    ctrl9_xl.den_z = val;
    ret = lsm6dsl_write_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_den_mark_axis_z_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  *val = ctrl9_xl.den_z;

  return ret;
}








 
int32_t lsm6dsl_den_mark_axis_y_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  if(ret == 0){
    ctrl9_xl.den_y = val;
    ret = lsm6dsl_write_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_den_mark_axis_y_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  *val = ctrl9_xl.den_y;

  return ret;
}








 
int32_t lsm6dsl_den_mark_axis_x_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  if(ret == 0){
    ctrl9_xl.den_x = val;
    ret = lsm6dsl_write_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_den_mark_axis_x_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  *val = ctrl9_xl.den_x;

  return ret;
}




 






 








 
int32_t lsm6dsl_pedo_step_reset_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  if(ret == 0){
    ctrl10_c.pedo_rst_step = val;
    ret = lsm6dsl_write_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_pedo_step_reset_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  *val = ctrl10_c.pedo_rst_step;

  return ret;
}








 
int32_t lsm6dsl_pedo_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  if(ret == 0){
    ctrl10_c.pedo_en = val;
    if (val != 0x00U) {
      ctrl10_c.func_en = val;
    }
    ret = lsm6dsl_write_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  }

  return ret;
}








 
int32_t lsm6dsl_pedo_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  *val = ctrl10_c.pedo_en;

  return ret;
}









 
int32_t lsm6dsl_pedo_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_config_pedo_ths_min_t config_pedo_ths_min;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x0FU,
                           (uint8_t*)&config_pedo_ths_min, 1);
      if(ret == 0){
       config_pedo_ths_min.ths_min = val;
      ret = lsm6dsl_write_reg(ctx, 0x0FU,
                              (uint8_t*)&config_pedo_ths_min, 1);
        if(ret == 0){
          ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
        }
      }
    }
  return ret;
}








 
int32_t lsm6dsl_pedo_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_config_pedo_ths_min_t config_pedo_ths_min;
  int32_t ret;

    ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
    if(ret == 0){
      ret = lsm6dsl_read_reg(ctx, 0x0FU,
                             (uint8_t*)&config_pedo_ths_min, 1);
      if(ret == 0){
        *val =  config_pedo_ths_min.ths_min;
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  return ret;
}









 
int32_t lsm6dsl_pedo_full_scale_set(lsm6dsl_ctx_t *ctx, lsm6dsl_pedo_fs_t val)
{
  lsm6dsl_config_pedo_ths_min_t config_pedo_ths_min;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x0FU,
                           (uint8_t*)&config_pedo_ths_min, 1);
    if(ret == 0){
       config_pedo_ths_min.pedo_fs = (uint8_t) val;
      ret = lsm6dsl_write_reg(ctx, 0x0FU,
                              (uint8_t*)&config_pedo_ths_min, 1);
      if(ret == 0){
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  }
  return ret;
}









 
int32_t lsm6dsl_pedo_full_scale_get(lsm6dsl_ctx_t *ctx, lsm6dsl_pedo_fs_t *val)
{
  lsm6dsl_config_pedo_ths_min_t config_pedo_ths_min;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x0FU,
                           (uint8_t*)&config_pedo_ths_min, 1);
    if(ret == 0){
      switch (config_pedo_ths_min.pedo_fs) {
         case LSM6DSL_PEDO_AT_2g:
          *val = LSM6DSL_PEDO_AT_2g;
          break;
        case LSM6DSL_PEDO_AT_4g:
          *val = LSM6DSL_PEDO_AT_4g;
          break;
        default:
          *val = LSM6DSL_PEDO_FS_ND;
          break;
      }
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}








 
int32_t lsm6dsl_pedo_debounce_steps_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_pedo_deb_reg_t pedo_deb_reg;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x14U,
                           (uint8_t*)&pedo_deb_reg, 1);
    if(ret == 0){
      pedo_deb_reg.deb_step = val;
      ret = lsm6dsl_write_reg(ctx, 0x14U,
                              (uint8_t*)&pedo_deb_reg, 1);
        if(ret == 0){
          ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
        }
      }
    }
  return ret;
}








 
int32_t lsm6dsl_pedo_debounce_steps_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_pedo_deb_reg_t pedo_deb_reg;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x14U,
                           (uint8_t*)&pedo_deb_reg, 1);
      if(ret == 0){
        *val = pedo_deb_reg.deb_step;
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }

  return ret;
}










 
int32_t lsm6dsl_pedo_timeout_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_pedo_deb_reg_t pedo_deb_reg;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x14U,
                           (uint8_t*)&pedo_deb_reg, 1);
    if(ret == 0){
      pedo_deb_reg.deb_time = val;
      ret = lsm6dsl_write_reg(ctx, 0x14U,
                              (uint8_t*)&pedo_deb_reg, 1);
      if(ret == 0){
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  }
  return ret;
}










 
int32_t lsm6dsl_pedo_timeout_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_pedo_deb_reg_t pedo_deb_reg;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x14U,
                           (uint8_t*)&pedo_deb_reg, 1);
    if(ret == 0){
      *val = pedo_deb_reg.deb_time;
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}








 
int32_t lsm6dsl_pedo_steps_period_set(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_write_reg(ctx, 0x15U, buff, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}








 
int32_t lsm6dsl_pedo_steps_period_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x15U, buff, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}




 







 








 
int32_t lsm6dsl_motion_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  if(ret == 0){
    ctrl10_c.sign_motion_en = val;
    if (val != 0x00U) {
      ctrl10_c.func_en = val;
      ret = lsm6dsl_write_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
    }
  }
  return ret;
}








 
int32_t lsm6dsl_motion_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  *val = ctrl10_c.sign_motion_en;

  return ret;
}








 
int32_t lsm6dsl_motion_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_write_reg(ctx, 0x13U, buff, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}








 
int32_t lsm6dsl_motion_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x13U, buff, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}




 







 








 
int32_t lsm6dsl_tilt_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  if(ret == 0){
    ctrl10_c.tilt_en = val;
    if (val != 0x00U) {
      ctrl10_c.func_en = val;
    }
    ret = lsm6dsl_write_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_tilt_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  *val = ctrl10_c.tilt_en;

  return ret;
}








 
int32_t lsm6dsl_wrist_tilt_sens_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  if(ret == 0){
    ctrl10_c.wrist_tilt_en = val;
    if (val != 0x00U) {
      ctrl10_c.func_en = val;
    }
    ret = lsm6dsl_write_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_wrist_tilt_sens_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  *val = ctrl10_c.wrist_tilt_en;

  return ret;
}










 
int32_t lsm6dsl_tilt_latency_set(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_B);
  if(ret == 0){
    ret = lsm6dsl_write_reg(ctx, 0x50U, buff, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}










 
int32_t lsm6dsl_tilt_latency_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_B);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x50U, buff, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}










 
int32_t lsm6dsl_tilt_threshold_set(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_B);
  if(ret == 0){
    ret = lsm6dsl_write_reg(ctx, 0x54U, buff, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}










 
int32_t lsm6dsl_tilt_threshold_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_B);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x54U, buff, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}








 
int32_t lsm6dsl_tilt_src_set(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_a_wrist_tilt_mask_t *val)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_B);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x59U,
                           (uint8_t*) val, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}








 
int32_t lsm6dsl_tilt_src_get(lsm6dsl_ctx_t *ctx,
                             lsm6dsl_a_wrist_tilt_mask_t *val)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_B);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x59U,
                           (uint8_t*) val, 1);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;

}




 







 








 
int32_t lsm6dsl_mag_soft_iron_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  if(ret == 0){
    ctrl9_xl.soft_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_mag_soft_iron_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_ctrl9_xl_t ctrl9_xl;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x18U, (uint8_t*)&ctrl9_xl, 1);
  *val = ctrl9_xl.soft_en;

  return ret;
}








 
int32_t lsm6dsl_mag_hard_iron_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_master_config_t master_config;
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  if(ret == 0){
    master_config.iron_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x1AU,
                            (uint8_t*)&master_config, 1);
    if(ret == 0){
      ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
      if(ret == 0){
        if (val != 0x00U) {
          ctrl10_c.func_en = val;
        }
        ret = lsm6dsl_write_reg(ctx, 0x19U,
                                (uint8_t*)&ctrl10_c, 1);
      }
    }
  }
  return ret;
}








 
int32_t lsm6dsl_mag_hard_iron_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  *val = master_config.iron_en;

  return ret;
}









 
int32_t lsm6dsl_mag_soft_iron_mat_set(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_write_reg(ctx, 0x24U, buff, 9);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}









 
int32_t lsm6dsl_mag_soft_iron_mat_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x24U, buff, 9);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}









 
int32_t lsm6dsl_mag_offset_set(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_write_reg(ctx, 0x2DU, buff, 6);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}









 
int32_t lsm6dsl_mag_offset_get(lsm6dsl_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x2DU, buff, 6);
    if(ret == 0){
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }
  return ret;
}




 







 

  






 
int32_t lsm6dsl_func_en_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_ctrl10_c_t ctrl10_c;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  if(ret == 0){
    ctrl10_c.func_en = val;
    ret = lsm6dsl_write_reg(ctx, 0x19U, (uint8_t*)&ctrl10_c, 1);
  }

  return ret;
}









 
int32_t lsm6dsl_sh_sync_sens_frame_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_sensor_sync_time_frame_t sensor_sync_time_frame;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x04U,
                         (uint8_t*)&sensor_sync_time_frame, 1);
  if(ret == 0){
     sensor_sync_time_frame.tph = val;
    ret = lsm6dsl_write_reg(ctx, 0x04U,
                            (uint8_t*)&sensor_sync_time_frame, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_sh_sync_sens_frame_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_sensor_sync_time_frame_t sensor_sync_time_frame;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x04U,
                         (uint8_t*)&sensor_sync_time_frame, 1);
  *val =  sensor_sync_time_frame.tph;

  return ret;
}








 
int32_t lsm6dsl_sh_sync_sens_ratio_set(lsm6dsl_ctx_t *ctx, lsm6dsl_rr_t val)
{
  lsm6dsl_sensor_sync_res_ratio_t sensor_sync_res_ratio;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x05U,
                              (uint8_t*)&sensor_sync_res_ratio, 1);
  if(ret == 0){
    sensor_sync_res_ratio.rr = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x05U,
                            (uint8_t*)&sensor_sync_res_ratio, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_sh_sync_sens_ratio_get(lsm6dsl_ctx_t *ctx, lsm6dsl_rr_t *val)
{
  lsm6dsl_sensor_sync_res_ratio_t sensor_sync_res_ratio;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x05U,
                         (uint8_t*)&sensor_sync_res_ratio, 1);

  switch ( sensor_sync_res_ratio.rr) {
    case LSM6DSL_RES_RATIO_2_11:
      *val = LSM6DSL_RES_RATIO_2_11;
      break;
    case LSM6DSL_RES_RATIO_2_12:
      *val = LSM6DSL_RES_RATIO_2_12;
      break;
    case LSM6DSL_RES_RATIO_2_13:
      *val = LSM6DSL_RES_RATIO_2_13;
      break;
    case LSM6DSL_RES_RATIO_2_14:
      *val = LSM6DSL_RES_RATIO_2_14;
      break;
    default:
      *val = LSM6DSL_RES_RATIO_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_sh_master_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  if(ret == 0){
    master_config.master_on = val;
    ret = lsm6dsl_write_reg(ctx, 0x1AU,
                            (uint8_t*)&master_config, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_sh_master_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  *val = master_config.master_on;

  return ret;
}








 
int32_t lsm6dsl_sh_pass_through_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  if(ret == 0){
    master_config.pass_through_mode = val;
    ret = lsm6dsl_write_reg(ctx, 0x1AU,
                            (uint8_t*)&master_config, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_sh_pass_through_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  *val = master_config.pass_through_mode;

  return ret;
}








 
int32_t lsm6dsl_sh_pin_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_pull_up_en_t val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  if(ret == 0){
    master_config.pull_up_en = (uint8_t) val;
    ret = lsm6dsl_write_reg(ctx, 0x1AU,
                            (uint8_t*)&master_config, 1);
  }

  return ret;
}








 
int32_t lsm6dsl_sh_pin_mode_get(lsm6dsl_ctx_t *ctx, lsm6dsl_pull_up_en_t *val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  switch (master_config.pull_up_en) {
    case LSM6DSL_EXT_PULL_UP:
      *val = LSM6DSL_EXT_PULL_UP;
      break;
    case LSM6DSL_INTERNAL_PULL_UP:
      *val = LSM6DSL_INTERNAL_PULL_UP;
      break;
    default:
      *val = LSM6DSL_SH_PIN_MODE;
      break;
  }
  return ret;
}








 
int32_t lsm6dsl_sh_syncro_mode_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_start_config_t val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  if(ret == 0){
    master_config.start_config = (uint8_t)val;
    ret = lsm6dsl_write_reg(ctx, 0x1AU,
                            (uint8_t*)&master_config, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_sh_syncro_mode_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_start_config_t *val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  switch (master_config.start_config) {
    case LSM6DSL_XL_GY_DRDY:
      *val = LSM6DSL_XL_GY_DRDY;
      break;
    case LSM6DSL_EXT_ON_INT2_PIN:
      *val = LSM6DSL_EXT_ON_INT2_PIN;
      break;
    default:
      *val = LSM6DSL_SH_SYNCRO_ND;
      break;
  }

  return ret;
}








 
int32_t lsm6dsl_sh_drdy_on_int1_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  if(ret == 0){
    master_config.drdy_on_int1 = val;
    ret = lsm6dsl_write_reg(ctx, 0x1AU,
                            (uint8_t*)&master_config, 1);
  }
  return ret;
}








 
int32_t lsm6dsl_sh_drdy_on_int1_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_master_config_t master_config;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x1AU,
                         (uint8_t*)&master_config, 1);
  *val = master_config.drdy_on_int1;

  return ret;
}








 
int32_t lsm6dsl_sh_read_data_raw_get(lsm6dsl_ctx_t *ctx,
                                     lsm6dsl_emb_sh_read_t *val)
{
  int32_t ret;
  ret = lsm6dsl_read_reg(ctx, 0x2EU,
                         (uint8_t*)&(val->sh_byte_1), 12);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x4DU,
                           (uint8_t*)&(val->sh_byte_13), 6);
  }
  return ret;
}









 
int32_t lsm6dsl_sh_cmd_sens_sync_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_master_cmd_code_t master_cmd_code;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x60U,
                         (uint8_t*)&master_cmd_code, 1);
  if(ret == 0){
    master_cmd_code.master_cmd_code = val;
    ret = lsm6dsl_write_reg(ctx, 0x60U,
                            (uint8_t*)&master_cmd_code, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_sh_cmd_sens_sync_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_master_cmd_code_t master_cmd_code;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x60U,
                         (uint8_t*)&master_cmd_code, 1);
  *val = master_cmd_code.master_cmd_code;

  return ret;
}









 
int32_t lsm6dsl_sh_spi_sync_error_set(lsm6dsl_ctx_t *ctx, uint8_t val)
{
  lsm6dsl_sens_sync_spi_error_code_t sens_sync_spi_error_code;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x61U,
                         (uint8_t*)&sens_sync_spi_error_code, 1);
  if(ret == 0){
     sens_sync_spi_error_code.error_code = val;
    ret = lsm6dsl_write_reg(ctx, 0x61U,
                            (uint8_t*)&sens_sync_spi_error_code, 1);
  }
  return ret;
}









 
int32_t lsm6dsl_sh_spi_sync_error_get(lsm6dsl_ctx_t *ctx, uint8_t *val)
{
  lsm6dsl_sens_sync_spi_error_code_t sens_sync_spi_error_code;
  int32_t ret;

  ret = lsm6dsl_read_reg(ctx, 0x61U,
                         (uint8_t*)&sens_sync_spi_error_code, 1);
  *val =  sens_sync_spi_error_code.error_code;

  return ret;
}








 
int32_t lsm6dsl_sh_num_of_dev_connected_set(lsm6dsl_ctx_t *ctx,
                                            lsm6dsl_aux_sens_on_t val)
{
  lsm6dsl_slave0_config_t slave0_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x04U,
                           (uint8_t*)&slave0_config, 1);
    if(ret == 0){
      slave0_config.aux_sens_on = (uint8_t) val;
      ret = lsm6dsl_write_reg(ctx, 0x04U,
                              (uint8_t*)&slave0_config, 1);
      if(ret == 0){
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  }
  return ret;
}








 
int32_t lsm6dsl_sh_num_of_dev_connected_get(lsm6dsl_ctx_t *ctx,
                                            lsm6dsl_aux_sens_on_t *val)
{
  lsm6dsl_slave0_config_t slave0_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x04U,
                           (uint8_t*)&slave0_config, 1);
    if(ret == 0){
      switch (slave0_config.aux_sens_on) {
        case LSM6DSL_SLV_0:
          *val = LSM6DSL_SLV_0;
          break;
        case LSM6DSL_SLV_0_1:
          *val = LSM6DSL_SLV_0_1;
          break;
        case LSM6DSL_SLV_0_1_2:
          *val = LSM6DSL_SLV_0_1_2;
          break;
        case LSM6DSL_SLV_0_1_2_3:
          *val = LSM6DSL_SLV_0_1_2_3;
          break;
        default:
          *val = LSM6DSL_SLV_EN_ND;
          break;
      }
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }

  return ret;
}











 
int32_t lsm6dsl_sh_cfg_write(lsm6dsl_ctx_t *ctx, lsm6dsl_sh_cfg_write_t *val)
{
  lsm6dsl_slv0_add_t slv0_add;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    slv0_add.slave0_add = val->slv0_add;
    slv0_add.rw_0 = 0;
    ret = lsm6dsl_write_reg(ctx, 0x02U, (uint8_t*)&slv0_add, 1);
    if(ret == 0){
      ret = lsm6dsl_write_reg(ctx, 0x03U,
                              &(val->slv0_subadd), 1);
      if(ret == 0){
        ret = lsm6dsl_write_reg(ctx, 0x0EU,
                                &(val->slv0_data), 1);
        if(ret == 0){
          ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
        }
      }
    }
  }
  return ret;
}











 
int32_t lsm6dsl_sh_slv0_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val)
{
  lsm6dsl_slave0_config_t slave0_config;
  lsm6dsl_slv0_add_t slv0_add;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    slv0_add.slave0_add = val->slv_add;
    slv0_add.rw_0 = 1;
    ret = lsm6dsl_write_reg(ctx, 0x02U, (uint8_t*)&slv0_add, 1);
    if(ret == 0){
      ret = lsm6dsl_write_reg(ctx, 0x03U,
                              &(val->slv_subadd), 1);
      if(ret == 0){
        ret = lsm6dsl_read_reg(ctx, 0x04U,
                               (uint8_t*)&slave0_config, 1);
        slave0_config.slave0_numop = val->slv_len;
        if(ret == 0){
          ret = lsm6dsl_write_reg(ctx, 0x04U,
                                  (uint8_t*)&slave0_config, 1);
          if(ret == 0){
            ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
          }
        }
      }
    }
  }
  return ret;
}











 
int32_t lsm6dsl_sh_slv1_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val)
{
  lsm6dsl_slave1_config_t slave1_config;
  lsm6dsl_slv1_add_t slv1_add;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    slv1_add.slave1_add  = val->slv_add;
    slv1_add.r_1 = 1;
    ret = lsm6dsl_write_reg(ctx, 0x05U, (uint8_t*)&slv1_add, 1);
    if(ret == 0){
      ret = lsm6dsl_write_reg(ctx, 0x06U,
                                   &(val->slv_subadd), 1);
      if(ret == 0){
        ret = lsm6dsl_read_reg(ctx, 0x07U,
                               (uint8_t*)&slave1_config, 1);
        slave1_config.slave1_numop = val->slv_len;
        if(ret == 0){
          ret = lsm6dsl_write_reg(ctx, 0x07U,
                                  (uint8_t*)&slave1_config, 1);
          if(ret == 0){
            ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
          }
        }
      }
    }
  }
  return ret;
}











 
int32_t lsm6dsl_sh_slv2_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val)
{
  lsm6dsl_slv2_add_t slv2_add;
  lsm6dsl_slave2_config_t slave2_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    slv2_add.slave2_add  = val->slv_add;
    slv2_add.r_2 = 1;
    ret = lsm6dsl_write_reg(ctx, 0x08U, (uint8_t*)&slv2_add, 1);
    if(ret == 0){
      ret = lsm6dsl_write_reg(ctx, 0x09U,
                              &(val->slv_subadd), 1);
      if(ret == 0){
        ret = lsm6dsl_read_reg(ctx, 0x0AU,
                               (uint8_t*)&slave2_config, 1);
        if(ret == 0){
          slave2_config.slave2_numop = val->slv_len;
          ret = lsm6dsl_write_reg(ctx, 0x0AU,
                                  (uint8_t*)&slave2_config, 1);
          if(ret == 0){
            ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
          }
        }
      }
    }
  }

  return ret;
}











 
int32_t lsm6dsl_sh_slv3_cfg_read(lsm6dsl_ctx_t *ctx,
                                 lsm6dsl_sh_cfg_read_t *val)
{
  lsm6dsl_slave3_config_t slave3_config;
  lsm6dsl_slv3_add_t slv3_add;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    slv3_add.slave3_add  = val->slv_add;
    slv3_add.r_3 = 1;
    ret = lsm6dsl_write_reg(ctx, 0x0BU, (uint8_t*)&slv3_add, 1);
    if(ret == 0){
      ret = lsm6dsl_write_reg(ctx, 0x0CU,
                              (uint8_t*)&(val->slv_subadd), 1);
      if(ret == 0){
        ret = lsm6dsl_read_reg(ctx, 0x0DU,
                               (uint8_t*)&slave3_config, 1);
        if(ret == 0){
          slave3_config.slave3_numop = val->slv_len;
          ret = lsm6dsl_write_reg(ctx, 0x0DU,
                                  (uint8_t*)&slave3_config, 1);
          if(ret == 0){
            ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
          }
        }
      }
    }
  }
  return ret;
}









 
int32_t lsm6dsl_sh_slave_0_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave0_rate_t val)
{
  lsm6dsl_slave0_config_t slave0_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x04U,
                           (uint8_t*)&slave0_config, 1);
    if(ret == 0){
      slave0_config.slave0_rate = (uint8_t) val;
      ret = lsm6dsl_write_reg(ctx, 0x04U,
                              (uint8_t*)&slave0_config, 1);
      if(ret == 0){
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  }
  return ret;
}









 
int32_t lsm6dsl_sh_slave_0_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave0_rate_t *val)
{
  lsm6dsl_slave0_config_t slave0_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x04U,
                           (uint8_t*)&slave0_config, 1);
    if(ret == 0){
      switch (slave0_config.slave0_rate) {
        case LSM6DSL_SL0_NO_DEC:
          *val = LSM6DSL_SL0_NO_DEC;
          break;
        case LSM6DSL_SL0_DEC_2:
          *val = LSM6DSL_SL0_DEC_2;
          break;
        case LSM6DSL_SL0_DEC_4:
          *val = LSM6DSL_SL0_DEC_4;
          break;
        case LSM6DSL_SL0_DEC_8:
          *val = LSM6DSL_SL0_DEC_8;
          break;
        default:
          *val = LSM6DSL_SL0_DEC_ND;
          break;
      }
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }

  return ret;
}











 
int32_t lsm6dsl_sh_write_mode_set(lsm6dsl_ctx_t *ctx, lsm6dsl_write_once_t val)
{
  lsm6dsl_slave1_config_t slave1_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x07U,
                           (uint8_t*)&slave1_config, 1);
    slave1_config.write_once = (uint8_t) val;
    if(ret == 0){
      ret = lsm6dsl_write_reg(ctx, 0x07U,
                              (uint8_t*)&slave1_config, 1);
      if(ret == 0){
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  }
  return ret;
}











 
int32_t lsm6dsl_sh_write_mode_get(lsm6dsl_ctx_t *ctx,
                                  lsm6dsl_write_once_t *val)
{
  lsm6dsl_slave1_config_t slave1_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x07U,
                           (uint8_t*)&slave1_config, 1);
    if(ret == 0){
      switch (slave1_config.write_once) {
        case LSM6DSL_EACH_SH_CYCLE:
          *val = LSM6DSL_EACH_SH_CYCLE;
          break;
        case LSM6DSL_ONLY_FIRST_CYCLE:
          *val = LSM6DSL_ONLY_FIRST_CYCLE;
          break;
        default:
          *val = LSM6DSL_SH_WR_MODE_ND;
          break;
      }
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }

  return ret;
}









 
int32_t lsm6dsl_sh_slave_1_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave1_rate_t val)
{
  lsm6dsl_slave1_config_t slave1_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x07U,
                           (uint8_t*)&slave1_config, 1);
    if(ret == 0){
      slave1_config.slave1_rate = (uint8_t) val;
      ret = lsm6dsl_write_reg(ctx, 0x07U,
                              (uint8_t*)&slave1_config, 1);
      if(ret == 0){
          ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  }
  return ret;
}








 
int32_t lsm6dsl_sh_slave_1_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave1_rate_t *val)
{
  lsm6dsl_slave1_config_t slave1_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x07U,
                           (uint8_t*)&slave1_config, 1);
    if(ret == 0){
      switch (slave1_config.slave1_rate) {
        case LSM6DSL_SL1_NO_DEC:
          *val = LSM6DSL_SL1_NO_DEC;
          break;
        case LSM6DSL_SL1_DEC_2:
          *val = LSM6DSL_SL1_DEC_2;
          break;
        case LSM6DSL_SL1_DEC_4:
          *val = LSM6DSL_SL1_DEC_4;
          break;
        case LSM6DSL_SL1_DEC_8:
          *val = LSM6DSL_SL1_DEC_8;
          break;
        default:
          *val = LSM6DSL_SL1_DEC_ND;
          break;
      }
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }

  return ret;
}









 
int32_t lsm6dsl_sh_slave_2_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave2_rate_t val)
{
  lsm6dsl_slave2_config_t slave2_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x0AU,
                           (uint8_t*)&slave2_config, 1);
    if(ret == 0){
      slave2_config.slave2_rate =(uint8_t) val;
      ret = lsm6dsl_write_reg(ctx, 0x0AU,
                              (uint8_t*)&slave2_config, 1);
      if(ret == 0){
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  }
  return ret;
}









 
int32_t lsm6dsl_sh_slave_2_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave2_rate_t *val)
{
  lsm6dsl_slave2_config_t slave2_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x0AU,
                           (uint8_t*)&slave2_config, 1);
    if(ret == 0){
      switch (slave2_config.slave2_rate) {
        case LSM6DSL_SL2_NO_DEC:
          *val = LSM6DSL_SL2_NO_DEC;
          break;
        case LSM6DSL_SL2_DEC_2:
          *val = LSM6DSL_SL2_DEC_2;
          break;
        case LSM6DSL_SL2_DEC_4:
          *val = LSM6DSL_SL2_DEC_4;
          break;
        case LSM6DSL_SL2_DEC_8:
          *val = LSM6DSL_SL2_DEC_8;
          break;
        default:
          *val = LSM6DSL_SL2_DEC_ND;
          break;
      }
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }

  return ret;
}









 
int32_t lsm6dsl_sh_slave_3_dec_set(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave3_rate_t val)
{
  lsm6dsl_slave3_config_t slave3_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x0DU,
                           (uint8_t*)&slave3_config, 1);
    slave3_config.slave3_rate = (uint8_t)val;
    if(ret == 0){
      ret = lsm6dsl_write_reg(ctx, 0x0DU,
                              (uint8_t*)&slave3_config, 1);
      if(ret == 0){
        ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
      }
    }
  }
  return ret;
}









 
int32_t lsm6dsl_sh_slave_3_dec_get(lsm6dsl_ctx_t *ctx,
                                   lsm6dsl_slave3_rate_t *val)
{
  lsm6dsl_slave3_config_t slave3_config;
  int32_t ret;

  ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_BANK_A);
  if(ret == 0){
    ret = lsm6dsl_read_reg(ctx, 0x0DU,
                           (uint8_t*)&slave3_config, 1);
    if(ret == 0){
      switch (slave3_config.slave3_rate) {
        case LSM6DSL_SL3_NO_DEC:
          *val = LSM6DSL_SL3_NO_DEC;
          break;
        case LSM6DSL_SL3_DEC_2:
          *val = LSM6DSL_SL3_DEC_2;
          break;
        case LSM6DSL_SL3_DEC_4:
          *val = LSM6DSL_SL3_DEC_4;
          break;
        case LSM6DSL_SL3_DEC_8:
          *val = LSM6DSL_SL3_DEC_8;
          break;
        default:
          *val = LSM6DSL_SL3_DEC_ND;
          break;
      }
      ret = lsm6dsl_mem_bank_set(ctx, LSM6DSL_USER_BANK);
    }
  }

  return ret;
}

 

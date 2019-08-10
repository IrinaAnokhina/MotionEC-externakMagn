#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.c"


































 

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"



































 

 







 
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






 
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"
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





 
#line 49 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"




 




 














 

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









 







 

typedef int32_t (*hts221_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*hts221_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
   
  hts221_write_ptr  write_reg;
  hts221_read_ptr   read_reg;
   
  void *handle;
} hts221_ctx_t;




 




 

 


 





 



typedef struct {
  uint8_t avgh                 : 3;
  uint8_t avgt                 : 3;
  uint8_t not_used_01          : 2;
} hts221_av_conf_t;


typedef struct {
  uint8_t odr                  : 2;
  uint8_t bdu                  : 1;
  uint8_t not_used_01          : 4;
  uint8_t pd                   : 1;
} hts221_ctrl_reg1_t;


typedef struct {
  uint8_t one_shot             : 1;
  uint8_t heater               : 1;
  uint8_t not_used_01          : 5;
  uint8_t boot                 : 1;
} hts221_ctrl_reg2_t;


typedef struct {
  uint8_t not_used_01          : 2;
  uint8_t drdy                 : 1;
  uint8_t not_used_02          : 3;
  uint8_t pp_od                : 1;
  uint8_t drdy_h_l             : 1;
} hts221_ctrl_reg3_t;


typedef struct {
  uint8_t t_da                 : 1;
  uint8_t h_da                 : 1;
  uint8_t not_used_01          : 6;
} hts221_status_reg_t;

#line 210 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"
typedef struct {
  uint8_t t0_msb               : 2;
  uint8_t t1_msb               : 2;
  uint8_t not_used_01          : 4;
} hts221_t1_t0_msb_t;

#line 224 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.h"












 
typedef union{
  hts221_av_conf_t        av_conf;
  hts221_ctrl_reg1_t      ctrl_reg1;
  hts221_ctrl_reg2_t      ctrl_reg2;
  hts221_ctrl_reg3_t      ctrl_reg3;
  hts221_status_reg_t     status_reg;
  hts221_t1_t0_msb_t      t1_t0_msb;
  bitwise_t               bitwise;
  uint8_t                 byte;
} hts221_reg_t;




 

int32_t hts221_read_reg(hts221_ctx_t *ctx, uint8_t reg, uint8_t* data,
                        uint16_t len);
int32_t hts221_write_reg(hts221_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);

typedef enum {
  HTS221_H_AVG_4    = 0,
  HTS221_H_AVG_8    = 1,
  HTS221_H_AVG_16   = 2,
  HTS221_H_AVG_32   = 3,
  HTS221_H_AVG_64   = 4,
  HTS221_H_AVG_128  = 5,
  HTS221_H_AVG_256  = 6,
  HTS221_H_AVG_512  = 7,
  HTS221_H_AVG_ND   = 8,
} hts221_avgh_t;
int32_t hts221_humidity_avg_set(hts221_ctx_t *ctx, hts221_avgh_t val);
int32_t hts221_humidity_avg_get(hts221_ctx_t *ctx, hts221_avgh_t *val);

typedef enum {
  HTS221_T_AVG_2   = 0,
  HTS221_T_AVG_4   = 1,
  HTS221_T_AVG_8   = 2,
  HTS221_T_AVG_16  = 3,
  HTS221_T_AVG_32  = 4,
  HTS221_T_AVG_64  = 5,
  HTS221_T_AVG_128 = 6,
  HTS221_T_AVG_256 = 7,
  HTS221_T_AVG_ND  = 8,
} hts221_avgt_t;
int32_t hts221_temperature_avg_set(hts221_ctx_t *ctx, hts221_avgt_t val);
int32_t hts221_temperature_avg_get(hts221_ctx_t *ctx, hts221_avgt_t *val);

typedef enum {
  HTS221_ONE_SHOT  = 0,
  HTS221_ODR_1Hz   = 1,
  HTS221_ODR_7Hz   = 2,
  HTS221_ODR_12Hz5 = 3,
  HTS221_ODR_ND    = 4,
} hts221_odr_t;
int32_t hts221_data_rate_set(hts221_ctx_t *ctx, hts221_odr_t val);
int32_t hts221_data_rate_get(hts221_ctx_t *ctx, hts221_odr_t *val);

int32_t hts221_block_data_update_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_block_data_update_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_one_shoot_trigger_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_one_shoot_trigger_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_temp_data_ready_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_hum_data_ready_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_humidity_raw_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temperature_raw_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_device_id_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_power_on_set(hts221_ctx_t *ctx, uint8_t val);

int32_t hts221_power_on_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_heater_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_heater_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_boot_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_boot_get(hts221_ctx_t *ctx, uint8_t *val);

int32_t hts221_status_get(hts221_ctx_t *ctx, hts221_status_reg_t *val);

int32_t hts221_drdy_on_int_set(hts221_ctx_t *ctx, uint8_t val);
int32_t hts221_drdy_on_int_get(hts221_ctx_t *ctx, uint8_t *val);

typedef enum {
  HTS221_PUSH_PULL   = 0,
  HTS221_OPEN_DRAIN  = 1,
  HTS221_PIN_MODE_ND = 2,
} hts221_pp_od_t;
int32_t hts221_pin_mode_set(hts221_ctx_t *ctx, hts221_pp_od_t val);
int32_t hts221_pin_mode_get(hts221_ctx_t *ctx, hts221_pp_od_t *val);

typedef enum {
  HTS221_ACTIVE_HIGH = 0,
  HTS221_ACTIVE_LOW  = 1,
  HTS221_ACTIVE_ND   = 2,
} hts221_drdy_h_l_t;
int32_t hts221_int_polarity_set(hts221_ctx_t *ctx, hts221_drdy_h_l_t val);
int32_t hts221_int_polarity_get(hts221_ctx_t *ctx, hts221_drdy_h_l_t *val);

int32_t hts221_hum_rh_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_hum_rh_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temp_deg_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_temp_deg_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_hum_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_hum_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);

int32_t hts221_temp_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff);
int32_t hts221_temp_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff);




 







 
#line 38 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\hts221\\hts221_reg.c"







 







 










 
int32_t hts221_read_reg(hts221_ctx_t* ctx, uint8_t reg, uint8_t* data,
                        uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}










 
int32_t hts221_write_reg(hts221_ctx_t* ctx, uint8_t reg, uint8_t* data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}




 






 








 
int32_t hts221_humidity_avg_set(hts221_ctx_t *ctx, hts221_avgh_t val)
{
  hts221_av_conf_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x10U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.avgh = (uint8_t)val;
    ret = hts221_write_reg(ctx, 0x10U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_humidity_avg_get(hts221_ctx_t *ctx, hts221_avgh_t *val)
{
  hts221_av_conf_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x10U, (uint8_t*) &reg, 1);

  switch (reg.avgh) {
    case HTS221_H_AVG_4:
      *val = HTS221_H_AVG_4;
      break;
    case HTS221_H_AVG_8:
      *val = HTS221_H_AVG_8;
      break;
    case HTS221_H_AVG_16:
      *val = HTS221_H_AVG_16;
      break;
    case HTS221_H_AVG_32:
      *val = HTS221_H_AVG_32;
      break;
    case HTS221_H_AVG_64:
      *val = HTS221_H_AVG_64;
      break;
    case HTS221_H_AVG_128:
      *val = HTS221_H_AVG_128;
      break;
    case HTS221_H_AVG_256:
      *val = HTS221_H_AVG_256;
      break;
    case HTS221_H_AVG_512:
      *val = HTS221_H_AVG_512;
      break;
    default:
      *val = HTS221_H_AVG_ND;
      break;
  }

  return ret;
}








 
int32_t hts221_temperature_avg_set(hts221_ctx_t *ctx, hts221_avgt_t val)
{
  hts221_av_conf_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x10U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.avgt = (uint8_t)val;
    ret = hts221_write_reg(ctx, 0x10U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_temperature_avg_get(hts221_ctx_t *ctx, hts221_avgt_t *val)
{
  hts221_av_conf_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x10U, (uint8_t*) &reg, 1);

  switch (reg.avgh) {
    case HTS221_T_AVG_2:
      *val = HTS221_T_AVG_2;
      break;
    case HTS221_T_AVG_4:
      *val = HTS221_T_AVG_4;
      break;
    case HTS221_T_AVG_8:
      *val = HTS221_T_AVG_8;
      break;
    case HTS221_T_AVG_16:
      *val = HTS221_T_AVG_16;
      break;
    case HTS221_T_AVG_32:
      *val = HTS221_T_AVG_32;
      break;
    case HTS221_T_AVG_64:
      *val = HTS221_T_AVG_64;
      break;
    case HTS221_T_AVG_128:
      *val = HTS221_T_AVG_128;
      break;
    case HTS221_T_AVG_256:
      *val = HTS221_T_AVG_256;
      break;
    default:
      *val = HTS221_T_AVG_ND;
      break;
  }

  return ret;
}








 
int32_t hts221_data_rate_set(hts221_ctx_t *ctx, hts221_odr_t val)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x20U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.odr = (uint8_t)val;
    ret = hts221_write_reg(ctx, 0x20U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_data_rate_get(hts221_ctx_t *ctx, hts221_odr_t *val)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x20U, (uint8_t*) &reg, 1);

  switch (reg.odr) {
    case HTS221_ONE_SHOT:
      *val = HTS221_ONE_SHOT;
      break;
    case HTS221_ODR_1Hz:
      *val = HTS221_ODR_1Hz;
      break;
    case HTS221_ODR_7Hz:
      *val = HTS221_ODR_7Hz;
      break;
    case HTS221_ODR_12Hz5:
      *val = HTS221_ODR_12Hz5;
      break;
    default:
      *val = HTS221_ODR_ND;
      break;
  }

  return ret;
}








 
int32_t hts221_block_data_update_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x20U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.bdu = val;
    ret = hts221_write_reg(ctx, 0x20U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_block_data_update_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x20U, (uint8_t*) &reg, 1);
  *val = reg.bdu;

  return ret;
}








 
int32_t hts221_one_shoot_trigger_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x21U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.one_shot = val;
    ret = hts221_write_reg(ctx, 0x21U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_one_shoot_trigger_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x21U, (uint8_t*) &reg, 1);
  *val = reg.one_shot;

  return ret;
}








 
int32_t hts221_temp_data_ready_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_status_reg_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x27U, (uint8_t*) &reg, 1);
  *val = reg.t_da;

  return ret;
}








 
int32_t hts221_hum_data_ready_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_status_reg_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x27U, (uint8_t*) &reg, 1);
  *val = reg.h_da;

  return ret;
}








 
int32_t hts221_humidity_raw_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, 0x28U, buff, 2);
  return ret;
}








 
int32_t hts221_temperature_raw_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, 0x2AU, buff, 2);
  return ret;
}




 






 








 
int32_t hts221_device_id_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, 0x0FU, buff, 1);
  return ret;
}








 
int32_t hts221_power_on_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x20U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.pd = val;
    ret = hts221_write_reg(ctx, 0x20U, (uint8_t*) &reg, 1);
  }
  return ret;
}








 
int32_t hts221_power_on_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_ctrl_reg1_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, 0x20U, (uint8_t*) &reg, 1);
  *val = reg.pd;

  return mm_error;
}








 
int32_t hts221_heater_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x21U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.heater = val;
    ret = hts221_write_reg(ctx, 0x21U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_heater_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x21U, (uint8_t*) &reg, 1);
  *val = reg.heater;

  return ret;
}








 
int32_t hts221_boot_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x21U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.boot = val;
    ret = hts221_write_reg(ctx, 0x21U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_boot_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x21U, (uint8_t*) &reg, 1);
  *val = reg.boot;

  return ret;
}








 
int32_t hts221_status_get(hts221_ctx_t *ctx, hts221_status_reg_t *val)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, 0x27U, (uint8_t*) val, 1);
  return ret;
}




 






 








 
int32_t hts221_drdy_on_int_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x22U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.drdy = val;
    ret = hts221_write_reg(ctx, 0x22U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_drdy_on_int_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x22U, (uint8_t*) &reg, 1);
  *val = reg.drdy;

  return ret;
}







 
int32_t hts221_pin_mode_set(hts221_ctx_t *ctx, hts221_pp_od_t val)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x22U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.pp_od = (uint8_t)val;
    ret = hts221_write_reg(ctx, 0x22U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_pin_mode_get(hts221_ctx_t *ctx, hts221_pp_od_t *val)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x22U, (uint8_t*) &reg, 1);

  switch (reg.pp_od) {
    case HTS221_PUSH_PULL:
      *val = HTS221_PUSH_PULL;
      break;
    case HTS221_OPEN_DRAIN:
      *val = HTS221_OPEN_DRAIN;
      break;
    default:
      *val = HTS221_PIN_MODE_ND;
      break;
  }

  return ret;
}








 
int32_t hts221_int_polarity_set(hts221_ctx_t *ctx, hts221_drdy_h_l_t val)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x22U, (uint8_t*) &reg, 1);

  if(ret == 0){
    reg.drdy_h_l = (uint8_t)val;
    ret = hts221_write_reg(ctx, 0x22U, (uint8_t*) &reg, 1);
  }

  return ret;
}








 
int32_t hts221_int_polarity_get(hts221_ctx_t *ctx, hts221_drdy_h_l_t *val)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x22U, (uint8_t*) &reg, 1);

  switch (reg.drdy_h_l) {
    case HTS221_ACTIVE_HIGH:
      *val = HTS221_ACTIVE_HIGH;
      break;
    case HTS221_ACTIVE_LOW:
      *val = HTS221_ACTIVE_LOW;
      break;
    default:
      *val = HTS221_ACTIVE_ND;
      break;
  }

  return ret;
}




 







 








 
int32_t hts221_hum_rh_point_0_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x30U, buff, 1);
  *buff = (uint8_t)(((uint16_t)(*buff) >> 1) & 0x7FFFu);

  return ret;
}








 
int32_t hts221_hum_rh_point_1_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x31U, buff, 1);
  *buff = (uint8_t)(((uint16_t)(*buff) >> 1) & 0x7FFFu);

  return ret;
}








 
int32_t hts221_temp_deg_point_0_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  hts221_t1_t0_msb_t reg;
  uint8_t coeff_h, coeff_l;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x32U, &coeff_l, 1);

  if(ret == 0){
    ret = hts221_read_reg(ctx, 0x35U, (uint8_t*) &reg, 1);
    coeff_h = reg.t0_msb;
    *(buff) = (uint8_t)(((coeff_h << 8) + coeff_l) >> 3);
  }

  return ret;
}








 
int32_t hts221_temp_deg_point_1_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  hts221_t1_t0_msb_t reg;
  uint8_t coeff_h, coeff_l;
  int32_t ret;

  ret = hts221_read_reg(ctx, 0x33U, &coeff_l, 1);

  if(ret == 0){
    ret = hts221_read_reg(ctx, 0x35U, (uint8_t*) &reg, 1);
    coeff_h = reg.t1_msb;
    *(buff) = (uint8_t)(((coeff_h << 8) + coeff_l) >> 3);
  }

  return ret;
}








 
int32_t hts221_hum_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, 0x36U, buff, 2);
  return ret;
}








 
int32_t hts221_hum_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, 0x3AU, buff, 2);
  return ret;
}








 
int32_t hts221_temp_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, 0x3CU, buff, 2);
  return ret;
}








 
int32_t hts221_temp_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, 0x3EU, buff, 2);
  return ret;
}




 

 

#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.c"

































 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.h"

































 

 








 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"



































 
 







 
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






 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"
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





 
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"




 




 














 

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









 

  





 

typedef int32_t (*lsm303agr_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lsm303agr_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
   
  lsm303agr_write_ptr  write_reg;
  lsm303agr_read_ptr   read_reg;
   
  void *handle;
} lsm303agr_ctx_t;




 




 

   



 






 


typedef struct {
  uint8_t not_used_01            : 2;
  uint8_t tda                    : 1;
  uint8_t not_used_02            : 3;
  uint8_t tor                    : 1;
  uint8_t not_used_03            : 1;
} lsm303agr_status_reg_aux_a_t;







typedef struct {
  uint8_t not_used_01            : 6;
  uint8_t temp_en                : 2;
} lsm303agr_temp_cfg_reg_a_t;


typedef struct {
  uint8_t xen                    : 1;
  uint8_t yen                    : 1;
  uint8_t zen                    : 1;
  uint8_t lpen                   : 1;
  uint8_t odr                    : 4;
} lsm303agr_ctrl_reg1_a_t;


typedef struct {
  uint8_t hp                     : 3;  
  uint8_t fds                    : 1;
  uint8_t hpcf                   : 2;
  uint8_t hpm                    : 2;
} lsm303agr_ctrl_reg2_a_t;


typedef struct {
  uint8_t not_used_01            : 1;
  uint8_t i1_overrun             : 1;
  uint8_t i1_wtm                 : 1;
  uint8_t i1_drdy2               : 1;
  uint8_t i1_drdy1               : 1;
  uint8_t i1_aoi2                : 1;
  uint8_t i1_aoi1                : 1;
  uint8_t i1_click               : 1;
} lsm303agr_ctrl_reg3_a_t;


typedef struct {
  uint8_t spi_enable             : 1;
  uint8_t st                     : 2;
  uint8_t hr                     : 1;
  uint8_t fs                     : 2;
  uint8_t ble                    : 1;
  uint8_t bdu                    : 1;
} lsm303agr_ctrl_reg4_a_t;


typedef struct {
  uint8_t d4d_int2               : 1;
  uint8_t lir_int2               : 1;
  uint8_t d4d_int1               : 1;
  uint8_t lir_int1               : 1;
  uint8_t not_used_01            : 2;
  uint8_t fifo_en                : 1;
  uint8_t boot                   : 1;
} lsm303agr_ctrl_reg5_a_t;


typedef struct {
  uint8_t not_used_01            : 1;
  uint8_t h_lactive              : 1;
  uint8_t not_used_02            : 1;
  uint8_t p2_act                 : 1;
  uint8_t boot_i2                : 1;
  uint8_t i2_int2                : 1;
  uint8_t i2_int1                : 1;
  uint8_t i2_clicken             : 1;
} lsm303agr_ctrl_reg6_a_t;



typedef struct {
  uint8_t xda                    : 1;
  uint8_t yda                    : 1;
  uint8_t zda                    : 1;
  uint8_t zyxda                  : 1;
  uint8_t _xor                   : 1;
  uint8_t yor                    : 1;
  uint8_t zor                    : 1;
  uint8_t zyxor                  : 1;
} lsm303agr_status_reg_a_t;

#line 264 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"
typedef struct {
  uint8_t fth                    : 5;
  uint8_t tr                     : 1;
  uint8_t fm                     : 2;
} lsm303agr_fifo_ctrl_reg_a_t;


typedef struct {
  uint8_t fss                    : 5;
  uint8_t empty                  : 1;
  uint8_t ovrn_fifo              : 1;
  uint8_t wtm                    : 1;
} lsm303agr_fifo_src_reg_a_t;


typedef struct {
  uint8_t xlie                   : 1;  
  uint8_t xhie                   : 1;  
  uint8_t ylie                   : 1;  
  uint8_t yhie                   : 1;  
  uint8_t zlie                   : 1;  
  uint8_t zhie                   : 1;  
  uint8_t _6d                    : 1;
  uint8_t aoi                    : 1;
} lsm303agr_int1_cfg_a_t;


typedef struct {
  uint8_t xl                     : 1;
  uint8_t xh                     : 1;
  uint8_t yl                     : 1;
  uint8_t yh                     : 1;
  uint8_t zl                     : 1;
  uint8_t zh                     : 1;
  uint8_t ia                     : 1;
  uint8_t not_used_01            : 1;
} lsm303agr_int1_src_a_t;


typedef struct {
  uint8_t ths                    : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_int1_ths_a_t;


typedef struct {
  uint8_t d                      : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_int1_duration_a_t;


typedef struct {
  uint8_t xlie                   : 1;
  uint8_t xhie                   : 1;
  uint8_t ylie                   : 1;
  uint8_t yhie                   : 1;
  uint8_t zlie                   : 1;
  uint8_t zhie                   : 1;
  uint8_t _6d                    : 1;
  uint8_t aoi                    : 1;
} lsm303agr_int2_cfg_a_t;


typedef struct {
  uint8_t xl                     : 1;
  uint8_t xh                     : 1;
  uint8_t yl                     : 1;
  uint8_t yh                     : 1;
  uint8_t zl                     : 1;
  uint8_t zh                     : 1;
  uint8_t ia                     : 1;
  uint8_t not_used_01            : 1;
} lsm303agr_int2_src_a_t;


typedef struct {
  uint8_t ths                    : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_int2_ths_a_t;


typedef struct {
  uint8_t d                      : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_int2_duration_a_t;


typedef struct {
  uint8_t xs                     : 1;
  uint8_t xd                     : 1;
  uint8_t ys                     : 1;
  uint8_t yd                     : 1;
  uint8_t zs                     : 1;
  uint8_t zd                     : 1;
  uint8_t not_used_01            : 2;
} lsm303agr_click_cfg_a_t;


typedef struct {
  uint8_t x                      : 1;
  uint8_t y                      : 1;
  uint8_t z                      : 1;
  uint8_t sign                   : 1;
  uint8_t sclick                 : 1;
  uint8_t dclick                 : 1;
  uint8_t ia                     : 1;
  uint8_t not_used_01            : 1;
} lsm303agr_click_src_a_t;


typedef struct {
  uint8_t ths                    : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_click_ths_a_t;


typedef struct {
  uint8_t tli                    : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_time_limit_a_t;


typedef struct {
  uint8_t tla                    : 8;
} lsm303agr_time_latency_a_t;


typedef struct {
  uint8_t tw                     : 8;
} lsm303agr_time_window_a_t;


typedef struct {
  uint8_t acth                   : 7;
  uint8_t not_used_01            : 1;
} lsm303agr_act_ths_a_t;


typedef struct {
  uint8_t actd                   : 8;
} lsm303agr_act_dur_a_t;

#line 414 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"
typedef struct {
  uint8_t md                     : 2;
  uint8_t odr                    : 2;
  uint8_t lp                     : 1;
  uint8_t soft_rst               : 1;
  uint8_t reboot                 : 1;
  uint8_t comp_temp_en           : 1;
} lsm303agr_cfg_reg_a_m_t;


typedef struct {
  uint8_t lpf                    : 1;
  uint8_t set_rst                : 2;  
  uint8_t int_on_dataoff         : 1;
  uint8_t off_canc_one_shot      : 1;
  uint8_t not_used_01            : 3;
} lsm303agr_cfg_reg_b_m_t;


typedef struct {
  uint8_t int_mag                : 1;
  uint8_t self_test              : 1;
  uint8_t not_used_01            : 1;
  uint8_t ble                    : 1;
  uint8_t bdu                    : 1;
  uint8_t i2c_dis                : 1;
  uint8_t int_mag_pin            : 1;
  uint8_t not_used_02            : 1;
} lsm303agr_cfg_reg_c_m_t;


typedef struct {
  uint8_t ien                    : 1;
  uint8_t iel                    : 1;
  uint8_t iea                    : 1;
  uint8_t not_used_01            : 2;
  uint8_t zien                   : 1;
  uint8_t yien                   : 1;
  uint8_t xien                   : 1;
} lsm303agr_int_crtl_reg_m_t;


typedef struct {
  uint8_t _int                    : 1;
  uint8_t mroi                   : 1;
  uint8_t n_th_s_z               : 1;
  uint8_t n_th_s_y               : 1;
  uint8_t n_th_s_x               : 1;
  uint8_t p_th_s_z               : 1;
  uint8_t p_th_s_y               : 1;
  uint8_t p_th_s_x               : 1;
} lsm303agr_int_source_reg_m_t;




typedef struct {
  uint8_t xda                    : 1;
  uint8_t yda                    : 1;
  uint8_t zda                    : 1;
  uint8_t zyxda                  : 1;
  uint8_t _xor                   : 1;
  uint8_t yor                    : 1;
  uint8_t zor                    : 1;
  uint8_t zyxor                  : 1;
} lsm303agr_status_reg_m_t;

#line 487 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.h"












 
typedef union{
  lsm303agr_status_reg_aux_a_t       status_reg_aux_a;
  lsm303agr_temp_cfg_reg_a_t         temp_cfg_reg_a;
  lsm303agr_ctrl_reg1_a_t            ctrl_reg1_a;
  lsm303agr_ctrl_reg2_a_t            ctrl_reg2_a;
  lsm303agr_ctrl_reg3_a_t            ctrl_reg3_a;
  lsm303agr_ctrl_reg4_a_t            ctrl_reg4_a;
  lsm303agr_ctrl_reg5_a_t            ctrl_reg5_a;
  lsm303agr_ctrl_reg6_a_t            ctrl_reg6_a;
  lsm303agr_status_reg_a_t           status_reg_a;
  lsm303agr_fifo_ctrl_reg_a_t        fifo_ctrl_reg_a;
  lsm303agr_fifo_src_reg_a_t         fifo_src_reg_a;
  lsm303agr_int1_cfg_a_t             int1_cfg_a;
  lsm303agr_int1_src_a_t             int1_src_a;
  lsm303agr_int1_ths_a_t             int1_ths_a;
  lsm303agr_int1_duration_a_t        int1_duration_a;
  lsm303agr_int2_cfg_a_t             int2_cfg_a;
  lsm303agr_int2_src_a_t             int2_src_a;
  lsm303agr_int2_ths_a_t             int2_ths_a;
  lsm303agr_int2_duration_a_t        int2_duration_a;
  lsm303agr_click_cfg_a_t            click_cfg_a;
  lsm303agr_click_src_a_t            click_src_a;
  lsm303agr_click_ths_a_t            click_ths_a;
  lsm303agr_time_limit_a_t           time_limit_a;
  lsm303agr_time_latency_a_t         time_latency_a;
  lsm303agr_time_window_a_t          time_window_a;
  lsm303agr_act_ths_a_t              act_ths_a;
  lsm303agr_act_dur_a_t              act_dur_a;
  lsm303agr_cfg_reg_a_m_t            cfg_reg_a_m;
  lsm303agr_cfg_reg_b_m_t            cfg_reg_b_m;
  lsm303agr_cfg_reg_c_m_t            cfg_reg_c_m;
  lsm303agr_int_crtl_reg_m_t         int_crtl_reg_m;
  lsm303agr_int_source_reg_m_t       int_source_reg_m;
  lsm303agr_status_reg_m_t           status_reg_m;
  bitwise_t                          bitwise;
  uint8_t                            byte;
} lsm303agr_reg_t;




 

int32_t lsm303agr_read_reg(lsm303agr_ctx_t *ctx, uint8_t reg, uint8_t* data,
                           uint16_t len);
int32_t lsm303agr_write_reg(lsm303agr_ctx_t *ctx, uint8_t reg, uint8_t* data,
                           uint16_t len);

extern float_t lsm303agr_from_fs_2g_hr_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_4g_hr_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_8g_hr_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_16g_hr_to_mg(int16_t lsb);
extern float_t lsm303agr_from_lsb_hr_to_celsius(int16_t lsb);

extern float_t lsm303agr_from_fs_2g_nm_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_4g_nm_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_8g_nm_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_16g_nm_to_mg(int16_t lsb);
extern float_t lsm303agr_from_lsb_nm_to_celsius(int16_t lsb);

extern float_t lsm303agr_from_fs_2g_lp_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_4g_lp_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_8g_lp_to_mg(int16_t lsb);
extern float_t lsm303agr_from_fs_16g_lp_to_mg(int16_t lsb);
extern float_t lsm303agr_from_lsb_lp_to_celsius(int16_t lsb);

extern float_t lsm303agr_from_lsb_to_mgauss(int16_t lsb);

int32_t lsm303agr_temp_status_reg_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

int32_t lsm303agr_temp_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_temp_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_temperature_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LSM303AGR_TEMP_DISABLE  = 0,
  LSM303AGR_TEMP_ENABLE   = 3,
} lsm303agr_temp_en_a_t;
int32_t lsm303agr_temperature_meas_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_temp_en_a_t val);
int32_t lsm303agr_temperature_meas_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_temp_en_a_t *val);

typedef enum {
  LSM303AGR_HR_12bit   = 0,
  LSM303AGR_NM_10bit   = 1,
  LSM303AGR_LP_8bit    = 2,
} lsm303agr_op_md_a_t;
int32_t lsm303agr_xl_operating_mode_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_op_md_a_t val);
int32_t lsm303agr_xl_operating_mode_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_op_md_a_t *val);

typedef enum {
  LSM303AGR_XL_POWER_DOWN                      = 0,
  LSM303AGR_XL_ODR_1Hz                         = 1,
  LSM303AGR_XL_ODR_10Hz                        = 2,
  LSM303AGR_XL_ODR_25Hz                        = 3,
  LSM303AGR_XL_ODR_50Hz                        = 4,
  LSM303AGR_XL_ODR_100Hz                       = 5,
  LSM303AGR_XL_ODR_200Hz                       = 6,
  LSM303AGR_XL_ODR_400Hz                       = 7,
  LSM303AGR_XL_ODR_1kHz620_LP                  = 8,
  LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP    = 9,
} lsm303agr_odr_a_t;
int32_t lsm303agr_xl_data_rate_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_odr_a_t val);
int32_t lsm303agr_xl_data_rate_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_odr_a_t *val);

int32_t lsm303agr_xl_high_pass_on_outputs_set(lsm303agr_ctx_t *ctx,
                                              uint8_t val);
int32_t lsm303agr_xl_high_pass_on_outputs_get(lsm303agr_ctx_t *ctx,
                                              uint8_t *val);

typedef enum {
  LSM303AGR_AGGRESSIVE  = 0,
  LSM303AGR_STRONG      = 1,
  LSM303AGR_MEDIUM      = 2,
  LSM303AGR_LIGHT       = 3,
} lsm303agr_hpcf_a_t;
int32_t lsm303agr_xl_high_pass_bandwidth_set(lsm303agr_ctx_t *ctx,
                                             lsm303agr_hpcf_a_t val);
int32_t lsm303agr_xl_high_pass_bandwidth_get(lsm303agr_ctx_t *ctx,
                                             lsm303agr_hpcf_a_t *val);

typedef enum {
  LSM303AGR_NORMAL_WITH_RST  = 0,
  LSM303AGR_REFERENCE_MODE   = 1,
  LSM303AGR_NORMAL           = 2,
  LSM303AGR_AUTORST_ON_INT   = 3,
} lsm303agr_hpm_a_t;
int32_t lsm303agr_xl_high_pass_mode_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hpm_a_t val);
int32_t lsm303agr_xl_high_pass_mode_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hpm_a_t *val);

typedef enum {
  LSM303AGR_2g   = 0,
  LSM303AGR_4g   = 1,
  LSM303AGR_8g   = 2,
  LSM303AGR_16g  = 3,
} lsm303agr_fs_a_t;
int32_t lsm303agr_xl_full_scale_set(lsm303agr_ctx_t *ctx,
                                    lsm303agr_fs_a_t val);
int32_t lsm303agr_xl_full_scale_get(lsm303agr_ctx_t *ctx,
                                    lsm303agr_fs_a_t *val);

int32_t lsm303agr_xl_block_data_update_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm303agr_xl_block_data_update_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val);

int32_t lsm303agr_xl_filter_reference_set(lsm303agr_ctx_t *ctx,
                                          uint8_t *buff);
int32_t lsm303agr_xl_filter_reference_get(lsm303agr_ctx_t *ctx,
                                          uint8_t *buff);

int32_t lsm303agr_xl_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_acceleration_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

int32_t lsm303agr_xl_device_id_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LSM303AGR_ST_DISABLE   = 0,
  LSM303AGR_ST_POSITIVE  = 1,
  LSM303AGR_ST_NEGATIVE  = 2,
} lsm303agr_st_a_t;
int32_t lsm303agr_xl_self_test_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_st_a_t val);
int32_t lsm303agr_xl_self_test_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_st_a_t *val);

typedef enum {
  LSM303AGR_XL_LSB_AT_LOW_ADD = 0,
  LSM303AGR_XL_MSB_AT_LOW_ADD = 1,
} lsm303agr_ble_a_t;
int32_t lsm303agr_xl_data_format_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ble_a_t val);
int32_t lsm303agr_xl_data_format_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ble_a_t *val);

int32_t lsm303agr_xl_boot_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_xl_boot_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_status_get(lsm303agr_ctx_t *ctx,
                                lsm303agr_status_reg_a_t *val);

int32_t lsm303agr_xl_int1_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int1_cfg_a_t *val);
int32_t lsm303agr_xl_int1_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int1_cfg_a_t *val);

int32_t lsm303agr_xl_int1_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int1_src_a_t *val);

int32_t lsm303agr_xl_int1_gen_threshold_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_xl_int1_gen_threshold_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm303agr_xl_int1_gen_duration_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm303agr_xl_int1_gen_duration_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val);

int32_t lsm303agr_xl_int2_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int2_cfg_a_t *val);
int32_t lsm303agr_xl_int2_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int2_cfg_a_t *val);

int32_t lsm303agr_xl_int2_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int2_src_a_t *val);

int32_t lsm303agr_xl_int2_gen_threshold_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_xl_int2_gen_threshold_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm303agr_xl_int2_gen_duration_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm303agr_xl_int2_gen_duration_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val);

typedef enum {
  LSM303AGR_DISC_FROM_INT_GENERATOR  = 0,
  LSM303AGR_ON_INT1_GEN              = 1,
  LSM303AGR_ON_INT2_GEN              = 2,
  LSM303AGR_ON_TAP_GEN               = 4,
  LSM303AGR_ON_INT1_INT2_GEN         = 3,
  LSM303AGR_ON_INT1_TAP_GEN          = 5,
  LSM303AGR_ON_INT2_TAP_GEN          = 6,
  LSM303AGR_ON_INT1_INT2_TAP_GEN     = 7,
} lsm303agr_hp_a_t;
int32_t lsm303agr_xl_high_pass_int_conf_set(lsm303agr_ctx_t *ctx,
                                            lsm303agr_hp_a_t val);
int32_t lsm303agr_xl_high_pass_int_conf_get(lsm303agr_ctx_t *ctx,
                                            lsm303agr_hp_a_t *val);

int32_t lsm303agr_xl_pin_int1_config_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg3_a_t *val);
int32_t lsm303agr_xl_pin_int1_config_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg3_a_t *val);

int32_t lsm303agr_xl_int2_pin_detect_4d_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_xl_int2_pin_detect_4d_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

typedef enum {
  LSM303AGR_INT2_PULSED   = 0,
  LSM303AGR_INT2_LATCHED  = 1,
} lsm303agr_lir_int2_a_t;
int32_t lsm303agr_xl_int2pin_notification_mode_set(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int2_a_t val);
int32_t lsm303agr_xl_int2pin_notification_mode_get(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int2_a_t *val);

int32_t lsm303agr_xl_int1_pin_detect_4d_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_xl_int1_pin_detect_4d_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

typedef enum {
  LSM303AGR_INT1_PULSED   = 0,
  LSM303AGR_INT1_LATCHED  = 1,
} lsm303agr_lir_int1_a_t;
int32_t lsm303agr_xl_int1pin_notification_mode_set(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int1_a_t val);
int32_t lsm303agr_xl_int1pin_notification_mode_get(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int1_a_t *val);

int32_t lsm303agr_xl_pin_int2_config_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg6_a_t *val);
int32_t lsm303agr_xl_pin_int2_config_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg6_a_t *val);

int32_t lsm303agr_xl_fifo_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_xl_fifo_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_fifo_watermark_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_xl_fifo_watermark_get(lsm303agr_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM303AGR_INT1_GEN = 0,
  LSM303AGR_INT2_GEN = 1,
} lsm303agr_tr_a_t;
int32_t lsm303agr_xl_fifo_trigger_event_set(lsm303agr_ctx_t *ctx,
                                            lsm303agr_tr_a_t val);
int32_t lsm303agr_xl_fifo_trigger_event_get(lsm303agr_ctx_t *ctx,
                                            lsm303agr_tr_a_t *val);

typedef enum {
  LSM303AGR_BYPASS_MODE           = 0,
  LSM303AGR_FIFO_MODE             = 1,
  LSM303AGR_DYNAMIC_STREAM_MODE   = 2,
  LSM303AGR_STREAM_TO_FIFO_MODE   = 3,
} lsm303agr_fm_a_t;
int32_t lsm303agr_xl_fifo_mode_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_fm_a_t val);
int32_t lsm303agr_xl_fifo_mode_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_fm_a_t *val);

int32_t lsm303agr_xl_fifo_status_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_fifo_src_reg_a_t *val);

int32_t lsm303agr_xl_fifo_data_level_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_fifo_empty_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_fifo_ovr_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_xl_fifo_fth_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_tap_conf_set(lsm303agr_ctx_t *ctx,
                               lsm303agr_click_cfg_a_t *val);
int32_t lsm303agr_tap_conf_get(lsm303agr_ctx_t *ctx,
                               lsm303agr_click_cfg_a_t *val);

int32_t lsm303agr_tap_source_get(lsm303agr_ctx_t *ctx,
                                lsm303agr_click_src_a_t *val);

int32_t lsm303agr_tap_threshold_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_tap_threshold_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_shock_dur_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_shock_dur_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_quiet_dur_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_quiet_dur_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_double_tap_timeout_set(lsm303agr_ctx_t *ctx,
                                         uint8_t val);
int32_t lsm303agr_double_tap_timeout_get(lsm303agr_ctx_t *ctx,
                                         uint8_t *val);

int32_t lsm303agr_act_threshold_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_act_threshold_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_act_timeout_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_act_timeout_get(lsm303agr_ctx_t *ctx, uint8_t *val);

typedef enum {
  LSM303AGR_SPI_4_WIRE = 0,
  LSM303AGR_SPI_3_WIRE = 1,
} lsm303agr_sim_a_t;
int32_t lsm303agr_xl_spi_mode_set(lsm303agr_ctx_t *ctx,
                                  lsm303agr_sim_a_t val);
int32_t lsm303agr_xl_spi_mode_get(lsm303agr_ctx_t *ctx,
                                  lsm303agr_sim_a_t *val);

int32_t lsm303agr_mag_user_offset_set(lsm303agr_ctx_t *ctx,
                                      uint8_t *buff);
int32_t lsm303agr_mag_user_offset_get(lsm303agr_ctx_t *ctx,
                                      uint8_t *buff);

typedef enum {
  LSM303AGR_CONTINUOUS_MODE  = 0,
  LSM303AGR_SINGLE_TRIGGER   = 1,
  LSM303AGR_POWER_DOWN       = 2,
} lsm303agr_md_m_t;
int32_t lsm303agr_mag_operating_mode_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_md_m_t val);
int32_t lsm303agr_mag_operating_mode_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_md_m_t *val);

typedef enum {
  LSM303AGR_MG_ODR_10Hz   = 0,
  LSM303AGR_MG_ODR_20Hz   = 1,
  LSM303AGR_MG_ODR_50Hz   = 2,
  LSM303AGR_MG_ODR_100Hz  = 3,
} lsm303agr_mg_odr_m_t;
int32_t lsm303agr_mag_data_rate_set(lsm303agr_ctx_t *ctx,
                                    lsm303agr_mg_odr_m_t val);
int32_t lsm303agr_mag_data_rate_get(lsm303agr_ctx_t *ctx,
                                    lsm303agr_mg_odr_m_t *val);

typedef enum {
  LSM303AGR_HIGH_RESOLUTION  = 0,
  LSM303AGR_LOW_POWER        = 1,
} lsm303agr_lp_m_t;
int32_t lsm303agr_mag_power_mode_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_lp_m_t val);
int32_t lsm303agr_mag_power_mode_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_lp_m_t *val);

int32_t lsm303agr_mag_offset_temp_comp_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val);
int32_t lsm303agr_mag_offset_temp_comp_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val);

typedef enum {
  LSM303AGR_ODR_DIV_2  = 0,
  LSM303AGR_ODR_DIV_4  = 1,
} lsm303agr_lpf_m_t;
int32_t lsm303agr_mag_low_pass_bandwidth_set(lsm303agr_ctx_t *ctx,
                                             lsm303agr_lpf_m_t val);
int32_t lsm303agr_mag_low_pass_bandwidth_get(lsm303agr_ctx_t *ctx,
                                             lsm303agr_lpf_m_t *val);

typedef enum {
  LSM303AGR_SET_SENS_ODR_DIV_63        = 0,
  LSM303AGR_SENS_OFF_CANC_EVERY_ODR    = 1,
  LSM303AGR_SET_SENS_ONLY_AT_POWER_ON  = 2,
} lsm303agr_set_rst_m_t;
int32_t lsm303agr_mag_set_rst_mode_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_set_rst_m_t val);
int32_t lsm303agr_mag_set_rst_mode_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_set_rst_m_t *val);

int32_t lsm303agr_mag_set_rst_sensor_single_set(lsm303agr_ctx_t *ctx,
                                                uint8_t val);
int32_t lsm303agr_mag_set_rst_sensor_single_get(lsm303agr_ctx_t *ctx,
                                                uint8_t *val);

int32_t lsm303agr_mag_block_data_update_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val);
int32_t lsm303agr_mag_block_data_update_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val);

int32_t lsm303agr_mag_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_magnetic_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

int32_t lsm303agr_mag_device_id_get(lsm303agr_ctx_t *ctx, uint8_t *buff);

int32_t lsm303agr_mag_reset_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_mag_reset_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_boot_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_mag_boot_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_self_test_set(lsm303agr_ctx_t *ctx,
                                    uint8_t val);
int32_t lsm303agr_mag_self_test_get(lsm303agr_ctx_t *ctx,
                                    uint8_t *val);

typedef enum {
  LSM303AGR_MG_LSB_AT_LOW_ADD  = 0,
  LSM303AGR_MG_MSB_AT_LOW_ADD  = 1,
} lsm303agr_ble_m_t;
int32_t lsm303agr_mag_data_format_set(lsm303agr_ctx_t *ctx,
                                      lsm303agr_ble_m_t val);
int32_t lsm303agr_mag_data_format_get(lsm303agr_ctx_t *ctx,
                                      lsm303agr_ble_m_t *val);

int32_t lsm303agr_mag_status_get(lsm303agr_ctx_t *ctx,
                                 lsm303agr_status_reg_m_t *val);

typedef enum {
  LSM303AGR_CHECK_BEFORE  = 0,
  LSM303AGR_CHECK_AFTER   = 1,
} lsm303agr_int_on_dataoff_m_t;
int32_t lsm303agr_mag_offset_int_conf_set(lsm303agr_ctx_t *ctx,
                                          lsm303agr_int_on_dataoff_m_t val);
int32_t lsm303agr_mag_offset_int_conf_get(lsm303agr_ctx_t *ctx,
                                          lsm303agr_int_on_dataoff_m_t *val);

int32_t lsm303agr_mag_drdy_on_pin_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_mag_drdy_on_pin_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_int_on_pin_set(lsm303agr_ctx_t *ctx, uint8_t val);
int32_t lsm303agr_mag_int_on_pin_get(lsm303agr_ctx_t *ctx, uint8_t *val);

int32_t lsm303agr_mag_int_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int_crtl_reg_m_t *val);
int32_t lsm303agr_mag_int_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int_crtl_reg_m_t *val);

int32_t lsm303agr_mag_int_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int_source_reg_m_t *val);

int32_t lsm303agr_mag_int_gen_treshold_set(lsm303agr_ctx_t *ctx,
                                           uint8_t *buff);
int32_t lsm303agr_mag_int_gen_treshold_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *buff);
typedef enum {
  LSM303AGR_I2C_ENABLE   = 0,
  LSM303AGR_I2C_DISABLE  = 1,
} lsm303agr_i2c_dis_m_t;
int32_t lsm303agr_mag_i2c_interface_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_i2c_dis_m_t val);
int32_t lsm303agr_mag_i2c_interface_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_i2c_dis_m_t *val);




 







 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.h"



 



 



 



 

typedef int32_t (*LSM303AGR_Init_Func)(void);
typedef int32_t (*LSM303AGR_DeInit_Func)(void);
typedef int32_t (*LSM303AGR_GetTick_Func)(void);
typedef int32_t (*LSM303AGR_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LSM303AGR_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LSM303AGR_Init_Func          Init;
  LSM303AGR_DeInit_Func        DeInit;
  uint32_t                     BusType;  
  uint8_t                      Address;
  LSM303AGR_WriteReg_Func      WriteReg;
  LSM303AGR_ReadReg_Func       ReadReg;
  LSM303AGR_GetTick_Func       GetTick;
} LSM303AGR_IO_t;


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} LSM303AGR_AxesRaw_t;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LSM303AGR_Axes_t;

typedef struct
{
  LSM303AGR_IO_t        IO;
  lsm303agr_ctx_t       Ctx;
  uint8_t               is_initialized;
  uint8_t               acc_is_enabled;
  lsm303agr_odr_a_t     acc_odr;
} LSM303AGR_ACC_Object_t;

typedef struct
{
  LSM303AGR_IO_t        IO;
  lsm303agr_ctx_t       Ctx;
  uint8_t               is_initialized;
  uint8_t               mag_is_enabled;
} LSM303AGR_MAG_Object_t;

typedef struct
{
  uint8_t   Acc;
  uint8_t   Gyro;
  uint8_t   Magneto;
  uint8_t   LowPower;
  uint32_t  GyroMaxFS;
  uint32_t  AccMaxFS;
  uint32_t  MagMaxFS;
  float     GyroMaxOdr;
  float     AccMaxOdr;
  float     MagMaxOdr;
} LSM303AGR_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LSM303AGR_ACC_Object_t *);
  int32_t (*DeInit)(LSM303AGR_ACC_Object_t *);
  int32_t (*ReadID)(LSM303AGR_ACC_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LSM303AGR_ACC_Object_t *, LSM303AGR_Capabilities_t *);
} LSM303AGR_ACC_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LSM303AGR_ACC_Object_t *);
  int32_t (*Disable)(LSM303AGR_ACC_Object_t *);
  int32_t (*GetSensitivity)(LSM303AGR_ACC_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM303AGR_ACC_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM303AGR_ACC_Object_t *, float);
  int32_t (*GetFullScale)(LSM303AGR_ACC_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM303AGR_ACC_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM303AGR_ACC_Object_t *, LSM303AGR_Axes_t *);
  int32_t (*GetAxesRaw)(LSM303AGR_ACC_Object_t *, LSM303AGR_AxesRaw_t *);
} LSM303AGR_ACC_Drv_t;

typedef struct
{
  int32_t (*Init)(LSM303AGR_MAG_Object_t *);
  int32_t (*DeInit)(LSM303AGR_MAG_Object_t *);
  int32_t (*ReadID)(LSM303AGR_MAG_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LSM303AGR_MAG_Object_t *, LSM303AGR_Capabilities_t *);
} LSM303AGR_MAG_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LSM303AGR_MAG_Object_t *);
  int32_t (*Disable)(LSM303AGR_MAG_Object_t *);
  int32_t (*GetSensitivity)(LSM303AGR_MAG_Object_t *, float *);
  int32_t (*GetOutputDataRate)(LSM303AGR_MAG_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LSM303AGR_MAG_Object_t *, float);
  int32_t (*GetFullScale)(LSM303AGR_MAG_Object_t *, int32_t *);
  int32_t (*SetFullScale)(LSM303AGR_MAG_Object_t *, int32_t);
  int32_t (*GetAxes)(LSM303AGR_MAG_Object_t *, LSM303AGR_Axes_t *);
  int32_t (*GetAxesRaw)(LSM303AGR_MAG_Object_t *, LSM303AGR_AxesRaw_t *);
} LSM303AGR_MAG_Drv_t;



 



 







#line 196 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.h"





 



 

int32_t LSM303AGR_ACC_RegisterBusIO(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_IO_t *pIO);
int32_t LSM303AGR_ACC_Init(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_DeInit(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_ReadID(LSM303AGR_ACC_Object_t *pObj, uint8_t *Id);
int32_t LSM303AGR_ACC_GetCapabilities(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities);

int32_t LSM303AGR_ACC_Enable(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_Disable(LSM303AGR_ACC_Object_t *pObj);
int32_t LSM303AGR_ACC_GetSensitivity(LSM303AGR_ACC_Object_t *pObj, float *sensitivity);
int32_t LSM303AGR_ACC_GetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float *odr);
int32_t LSM303AGR_ACC_SetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float odr);
int32_t LSM303AGR_ACC_GetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t *fullscale);
int32_t LSM303AGR_ACC_SetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t fullscale);
int32_t LSM303AGR_ACC_GetAxes(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Axes_t *acceleration);
int32_t LSM303AGR_ACC_GetAxesRaw(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_AxesRaw_t *value);

int32_t LSM303AGR_ACC_Read_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t reg, uint8_t *data);
int32_t LSM303AGR_ACC_Write_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t reg, uint8_t data);

int32_t LSM303AGR_ACC_Get_DRDY_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *status);
int32_t LSM303AGR_ACC_Get_Init_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *status);

int32_t LSM303AGR_MAG_RegisterBusIO(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_IO_t *pIO);
int32_t LSM303AGR_MAG_Init(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_DeInit(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_ReadID(LSM303AGR_MAG_Object_t *pObj, uint8_t *id);
int32_t LSM303AGR_MAG_GetCapabilities(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities);

int32_t LSM303AGR_MAG_Enable(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_Disable(LSM303AGR_MAG_Object_t *pObj);
int32_t LSM303AGR_MAG_GetSensitivity(LSM303AGR_MAG_Object_t *pObj, float *sensitivity);
int32_t LSM303AGR_MAG_GetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float *odr);
int32_t LSM303AGR_MAG_SetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float odr);
int32_t LSM303AGR_MAG_GetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t *fullscale);
int32_t LSM303AGR_MAG_SetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t fullscale);
int32_t LSM303AGR_MAG_GetAxes(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Axes_t *magnetic_field);
int32_t LSM303AGR_MAG_GetAxesRaw(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_AxesRaw_t *value);

int32_t LSM303AGR_MAG_Read_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t reg, uint8_t *data);
int32_t LSM303AGR_MAG_Write_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t reg, uint8_t data);

int32_t LSM303AGR_MAG_Get_DRDY_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *status);
int32_t LSM303AGR_MAG_Get_Init_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *status);



 



 

extern LSM303AGR_ACC_CommonDrv_t LSM303AGR_ACC_COMMON_Driver;
extern LSM303AGR_ACC_Drv_t LSM303AGR_ACC_Driver;
extern LSM303AGR_MAG_CommonDrv_t LSM303AGR_MAG_COMMON_Driver;
extern LSM303AGR_MAG_Drv_t LSM303AGR_MAG_Driver;



 









 



 



 

 
#line 38 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr.c"



 



 



 



 

LSM303AGR_ACC_CommonDrv_t LSM303AGR_ACC_COMMON_Driver =
{
  LSM303AGR_ACC_Init,
  LSM303AGR_ACC_DeInit,
  LSM303AGR_ACC_ReadID,
  LSM303AGR_ACC_GetCapabilities,
};

LSM303AGR_ACC_Drv_t LSM303AGR_ACC_Driver =
{
  LSM303AGR_ACC_Enable,
  LSM303AGR_ACC_Disable,
  LSM303AGR_ACC_GetSensitivity,
  LSM303AGR_ACC_GetOutputDataRate,
  LSM303AGR_ACC_SetOutputDataRate,
  LSM303AGR_ACC_GetFullScale,
  LSM303AGR_ACC_SetFullScale,
  LSM303AGR_ACC_GetAxes,
  LSM303AGR_ACC_GetAxesRaw,
};

LSM303AGR_MAG_CommonDrv_t LSM303AGR_MAG_COMMON_Driver =
{
  LSM303AGR_MAG_Init,
  LSM303AGR_MAG_DeInit,
  LSM303AGR_MAG_ReadID,
  LSM303AGR_MAG_GetCapabilities,
};

LSM303AGR_MAG_Drv_t LSM303AGR_MAG_Driver =
{
  LSM303AGR_MAG_Enable,
  LSM303AGR_MAG_Disable,
  LSM303AGR_MAG_GetSensitivity,
  LSM303AGR_MAG_GetOutputDataRate,
  LSM303AGR_MAG_SetOutputDataRate,
  LSM303AGR_MAG_GetFullScale,
  LSM303AGR_MAG_SetFullScale,
  LSM303AGR_MAG_GetAxes,
  LSM303AGR_MAG_GetAxesRaw,
};



 



 

static int32_t ReadAccRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteAccRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t ReadMagRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteMagRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LSM303AGR_ACC_GetSensitivityHR(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity);
static int32_t LSM303AGR_ACC_GetSensitivityNM(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity);
static int32_t LSM303AGR_ACC_GetSensitivityLP(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity);
static int32_t LSM303AGR_ACC_SetOutputDataRate_When_Enabled(LSM303AGR_ACC_Object_t *pObj, float Odr);
static int32_t LSM303AGR_ACC_SetOutputDataRate_When_Disabled(LSM303AGR_ACC_Object_t *pObj, float Odr);



 



 





 
int32_t LSM303AGR_ACC_RegisterBusIO(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_IO_t *pIO)
{
  int32_t ret = 0;

  if (pObj == 0)
  {
    ret = -1;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadAccRegWrap;
    pObj->Ctx.write_reg = WriteAccRegWrap;
    pObj->Ctx.handle    = pObj;

    if (pObj->IO.Init == 0)
    {
      ret = -1;
    }
    else if (pObj->IO.Init() != 0)
    {
      ret = -1;
    }
    else
    {
      if (pObj->IO.BusType == 1U)  
      {
         
        if (pObj->is_initialized == 0U)
        {
           
          uint8_t data = 0x01;

          if (LSM303AGR_ACC_Write_Reg(pObj, 0x23U, data) != 0)
          {
            return -1;
          }
        }
      }
    }
  }

  return ret;
}





 
int32_t LSM303AGR_ACC_Init(LSM303AGR_ACC_Object_t *pObj)
{
   
  if (lsm303agr_xl_block_data_update_set(&(pObj->Ctx), (1U)) != 0)
  {
    return -1;
  }

   
  if (lsm303agr_xl_fifo_mode_set(&(pObj->Ctx), LSM303AGR_BYPASS_MODE) != 0)
  {
    return -1;
  }

   
  pObj->acc_odr = LSM303AGR_XL_ODR_100Hz;

   
  if (lsm303agr_xl_data_rate_set(&(pObj->Ctx), LSM303AGR_XL_POWER_DOWN) != 0)
  {
    return -1;
  }

   
  if (lsm303agr_xl_full_scale_set(&(pObj->Ctx), LSM303AGR_2g) != 0)
  {
    return -1;
  }

  pObj->is_initialized = 1;

  return 0;
}





 
int32_t LSM303AGR_ACC_DeInit(LSM303AGR_ACC_Object_t *pObj)
{
   
  if (LSM303AGR_ACC_Disable(pObj) != 0)
  {
    return -1;
  }

   
  pObj->acc_odr = LSM303AGR_XL_POWER_DOWN;
  pObj->is_initialized = 0;

  return 0;
}






 
int32_t LSM303AGR_ACC_ReadID(LSM303AGR_ACC_Object_t *pObj, uint8_t *Id)
{
  if (lsm303agr_xl_device_id_get(&(pObj->Ctx), Id) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LSM303AGR_ACC_GetCapabilities(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities)
{
   
  (void)(pObj);

  Capabilities->Acc          = 1;
  Capabilities->Gyro         = 0;
  Capabilities->Magneto      = 0;
  Capabilities->LowPower     = 0;
  Capabilities->GyroMaxFS    = 0;
  Capabilities->AccMaxFS     = 16;
  Capabilities->MagMaxFS     = 0;
  Capabilities->GyroMaxOdr   = 0.0f;
  Capabilities->AccMaxOdr    = 400.0f;
  Capabilities->MagMaxOdr    = 0.0f;
  return 0;
}





 
int32_t LSM303AGR_ACC_Enable(LSM303AGR_ACC_Object_t *pObj)
{
   
  if (pObj->acc_is_enabled == 1U)
  {
    return 0;
  }

   
  if (lsm303agr_xl_data_rate_set(&(pObj->Ctx), pObj->acc_odr) != 0)
  {
    return -1;
  }

  pObj->acc_is_enabled = 1;

  return 0;
}





 
int32_t LSM303AGR_ACC_Disable(LSM303AGR_ACC_Object_t *pObj)
{
   
  if (pObj->acc_is_enabled == 0U)
  {
    return 0;
  }

   
  if (lsm303agr_xl_data_rate_get(&(pObj->Ctx), &pObj->acc_odr) != 0)
  {
    return -1;
  }

   
  if (lsm303agr_xl_data_rate_set(&(pObj->Ctx), LSM303AGR_XL_POWER_DOWN) != 0)
  {
    return -1;
  }

  pObj->acc_is_enabled = 0;

  return 0;
}






 
int32_t LSM303AGR_ACC_GetSensitivity(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = 0;
  lsm303agr_op_md_a_t op_mode;

   
  if (lsm303agr_xl_operating_mode_get(&(pObj->Ctx), &op_mode) != 0)
  {
    return -1;
  }

   
  switch (op_mode)
  {
    case LSM303AGR_HR_12bit:
      if (LSM303AGR_ACC_GetSensitivityHR(pObj, Sensitivity) != 0)
      {
        return -1;
      }
      break;

    case LSM303AGR_NM_10bit:
      if (LSM303AGR_ACC_GetSensitivityNM(pObj, Sensitivity) != 0)
      {
        return -1;
      }
      break;

    case LSM303AGR_LP_8bit:
      if (LSM303AGR_ACC_GetSensitivityLP(pObj, Sensitivity) != 0)
      {
        return -1;
      }
      break;

    default:
      ret = -1;
      break;
  }

  return ret;
}






 
int32_t LSM303AGR_ACC_GetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float *Odr)
{
  int32_t ret = 0;
  lsm303agr_op_md_a_t op_mode;
  lsm303agr_odr_a_t odr_low_level;

   
  if (lsm303agr_xl_operating_mode_get(&(pObj->Ctx), &op_mode) != 0)
  {
    return -1;
  }

   
  if (lsm303agr_xl_data_rate_get(&(pObj->Ctx), &odr_low_level) != 0)
  {
    return -1;
  }

  if (op_mode == LSM303AGR_LP_8bit)  
  {
    switch (odr_low_level)
    {
      case LSM303AGR_XL_POWER_DOWN:
        *Odr = 0.0f;
        break;

      case LSM303AGR_XL_ODR_1Hz:
        *Odr = 1.0f;
        break;

      case LSM303AGR_XL_ODR_10Hz:
        *Odr = 10.0f;
        break;

      case LSM303AGR_XL_ODR_25Hz:
        *Odr = 25.0f;
        break;

      case LSM303AGR_XL_ODR_50Hz:
        *Odr = 50.0f;
        break;

      case LSM303AGR_XL_ODR_100Hz:
        *Odr = 100.0f;
        break;

      case LSM303AGR_XL_ODR_200Hz:
        *Odr = 200.0f;
        break;

      case LSM303AGR_XL_ODR_400Hz:
        *Odr = 400.0f;
        break;

      case LSM303AGR_XL_ODR_1kHz620_LP:
        *Odr = 1620.0f;
        break;

      case LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP:
        *Odr = 5376.0f;
        break;

      default:
        ret = -1;
        break;
    }
  }
  else  
  {
    switch (odr_low_level)
    {
      case LSM303AGR_XL_POWER_DOWN:
        *Odr = 0.0f;
        break;

      case LSM303AGR_XL_ODR_1Hz:
        *Odr = 1.0f;
        break;

      case LSM303AGR_XL_ODR_10Hz:
        *Odr = 10.0f;
        break;

      case LSM303AGR_XL_ODR_25Hz:
        *Odr = 25.0f;
        break;

      case LSM303AGR_XL_ODR_50Hz:
        *Odr = 50.0f;
        break;

      case LSM303AGR_XL_ODR_100Hz:
        *Odr = 100.0f;
        break;

      case LSM303AGR_XL_ODR_200Hz:
        *Odr = 200.0f;
        break;

      case LSM303AGR_XL_ODR_400Hz:
        *Odr = 400.0f;
        break;

      case LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP:
        *Odr = 1344.0f;
        break;

      default:
        ret = -1;
        break;
    }
  }

  return ret;
}






 
int32_t LSM303AGR_ACC_SetOutputDataRate(LSM303AGR_ACC_Object_t *pObj, float Odr)
{
   
  if (pObj->acc_is_enabled == 1U)
  {
    return LSM303AGR_ACC_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LSM303AGR_ACC_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}






 
int32_t LSM303AGR_ACC_GetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t *FullScale)
{
  int32_t ret = 0;
  lsm303agr_fs_a_t fs_low_level;

   
  if (lsm303agr_xl_full_scale_get(&(pObj->Ctx), &fs_low_level) != 0)
  {
    return -1;
  }

  switch (fs_low_level)
  {
    case LSM303AGR_2g:
      *FullScale =  2;
      break;

    case LSM303AGR_4g:
      *FullScale =  4;
      break;

    case LSM303AGR_8g:
      *FullScale =  8;
      break;

    case LSM303AGR_16g:
      *FullScale = 16;
      break;

    default:
      ret = -1;
      break;
  }

  return ret;
}






 
int32_t LSM303AGR_ACC_SetFullScale(LSM303AGR_ACC_Object_t *pObj, int32_t FullScale)
{
  lsm303agr_fs_a_t new_fs;

  new_fs = (FullScale <= 2) ? LSM303AGR_2g
           : (FullScale <= 4) ? LSM303AGR_4g
           : (FullScale <= 8) ? LSM303AGR_8g
           :                    LSM303AGR_16g;

  if (lsm303agr_xl_full_scale_set(&(pObj->Ctx), new_fs) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LSM303AGR_ACC_GetAxesRaw(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_AxesRaw_t *Value)
{
  int16_t divisor = 1;
  axis3bit16_t data_raw;
  int32_t ret = 0;
  lsm303agr_op_md_a_t op_mode;

   
  if (lsm303agr_xl_operating_mode_get(&(pObj->Ctx), &op_mode) != 0)
  {
    return -1;
  }

   
  switch (op_mode)
  {
    case LSM303AGR_HR_12bit:
      divisor = 16;
      break;

    case LSM303AGR_NM_10bit:
      divisor = 64;
      break;

    case LSM303AGR_LP_8bit:
      divisor = 256;
      break;

    default:
      ret = -1;
      break;
  }

  if (ret == -1)
  {
    return ret;
  }

   
  if (lsm303agr_acceleration_raw_get(&(pObj->Ctx), data_raw.u8bit) != 0)
  {
    return -1;
  }

   
  Value->x = (data_raw.i16bit[0] / divisor);
  Value->y = (data_raw.i16bit[1] / divisor);
  Value->z = (data_raw.i16bit[2] / divisor);

  return ret;
}






 
int32_t LSM303AGR_ACC_GetAxes(LSM303AGR_ACC_Object_t *pObj, LSM303AGR_Axes_t *Acceleration)
{
  LSM303AGR_AxesRaw_t data_raw;
  float sensitivity = 0.0f;

   
  if (LSM303AGR_ACC_GetAxesRaw(pObj, &data_raw) != 0)
  {
    return -1;
  }

   
  if (LSM303AGR_ACC_GetSensitivity(pObj, &sensitivity) != 0)
  {
    return -1;
  }

   
  Acceleration->x = (int32_t)((float)((float)data_raw.x * sensitivity));
  Acceleration->y = (int32_t)((float)((float)data_raw.y * sensitivity));
  Acceleration->z = (int32_t)((float)((float)data_raw.z * sensitivity));

  return 0;
}







 
int32_t LSM303AGR_ACC_Read_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lsm303agr_read_reg(&(pObj->Ctx), Reg, Data, 1) != 0)
  {
    return -1;
  }

  return 0;
}







 
int32_t LSM303AGR_ACC_Write_Reg(LSM303AGR_ACC_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lsm303agr_write_reg(&(pObj->Ctx), Reg, &Data, 1) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LSM303AGR_ACC_Get_DRDY_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *Status)
{
  if (lsm303agr_xl_data_ready_get(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LSM303AGR_ACC_Get_Init_Status(LSM303AGR_ACC_Object_t *pObj, uint8_t *Status)
{
  if (pObj == 0)
  {
    return -1;
  }

  *Status = pObj->is_initialized;

  return 0;
}





 
int32_t LSM303AGR_MAG_RegisterBusIO(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_IO_t *pIO)
{
  int32_t ret = 0;

  if (pObj == 0)
  {
    ret = -1;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->IO.GetTick   = pIO->GetTick;

    pObj->Ctx.read_reg  = ReadMagRegWrap;
    pObj->Ctx.write_reg = WriteMagRegWrap;
    pObj->Ctx.handle    = pObj;

    if (pObj->IO.Init == 0)
    {
      ret = -1;
    }
    else if (pObj->IO.Init() != 0)
    {
      ret = -1;
    }
    else
    {
      if (pObj->IO.BusType != 0U)  
      {
         
        if (pObj->is_initialized == 0U)
        {
           
          if (lsm303agr_mag_i2c_interface_set(&(pObj->Ctx), LSM303AGR_I2C_DISABLE) != 0)
          {
            return -1;
          }
        }
      }
    }
  }

  return ret;
}





 
int32_t LSM303AGR_MAG_Init(LSM303AGR_MAG_Object_t *pObj)
{
   
  if (lsm303agr_mag_block_data_update_set(&(pObj->Ctx), (1U)) != 0)
  {
    return -1;
  }

   
  if (lsm303agr_mag_operating_mode_set(&(pObj->Ctx), LSM303AGR_POWER_DOWN) != 0)
  {
    return -1;
  }

   
  if (lsm303agr_mag_data_rate_set(&(pObj->Ctx), LSM303AGR_MG_ODR_100Hz) != 0)
  {
    return -1;
  }

   
  if (lsm303agr_mag_self_test_set(&(pObj->Ctx), (0U)) != 0)
  {
    return -1;
  }

  pObj->is_initialized = 1;

  return 0;
}





 
int32_t LSM303AGR_MAG_DeInit(LSM303AGR_MAG_Object_t *pObj)
{
   
  if (LSM303AGR_MAG_Disable(pObj) != 0)
  {
    return -1;
  }

  pObj->is_initialized = 0;

  return 0;
}






 
int32_t LSM303AGR_MAG_ReadID(LSM303AGR_MAG_Object_t *pObj, uint8_t *Id)
{
  if (lsm303agr_mag_device_id_get(&(pObj->Ctx), Id) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LSM303AGR_MAG_GetCapabilities(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Capabilities_t *Capabilities)
{
   
  (void)(pObj);

  Capabilities->Acc          = 0;
  Capabilities->Gyro         = 0;
  Capabilities->Magneto      = 1;
  Capabilities->LowPower     = 0;
  Capabilities->GyroMaxFS    = 0;
  Capabilities->AccMaxFS     = 0;
  Capabilities->MagMaxFS     = 50;
  Capabilities->GyroMaxOdr   = 0.0f;
  Capabilities->AccMaxOdr    = 0.0f;
  Capabilities->MagMaxOdr    = 100.0f;
  return 0;
}





 
int32_t LSM303AGR_MAG_Enable(LSM303AGR_MAG_Object_t *pObj)
{
   
  if (pObj->mag_is_enabled == 1U)
  {
    return 0;
  }

   
  if (lsm303agr_mag_operating_mode_set(&(pObj->Ctx), LSM303AGR_CONTINUOUS_MODE) != 0)
  {
    return -1;
  }

  pObj->mag_is_enabled = 1;

  return 0;
}





 
int32_t LSM303AGR_MAG_Disable(LSM303AGR_MAG_Object_t *pObj)
{
   
  if (pObj->mag_is_enabled == 0U)
  {
    return 0;
  }

   
  if (lsm303agr_mag_operating_mode_set(&(pObj->Ctx), LSM303AGR_POWER_DOWN) != 0)
  {
    return -1;
  }

  pObj->mag_is_enabled = 0;

  return 0;
}






 
int32_t LSM303AGR_MAG_GetSensitivity(LSM303AGR_MAG_Object_t *pObj, float *Sensitivity)
{
  *Sensitivity = 1.500f;

  return 0;
}






 
int32_t LSM303AGR_MAG_GetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float *Odr)
{
  int32_t ret = 0;
  lsm303agr_mg_odr_m_t odr_low_level;

   
  if (lsm303agr_mag_data_rate_get(&(pObj->Ctx), &odr_low_level) != 0)
  {
    return -1;
  }

  switch (odr_low_level)
  {
    case LSM303AGR_MG_ODR_10Hz:
      *Odr = 10.0f;
      break;

    case LSM303AGR_MG_ODR_20Hz:
      *Odr = 20.0f;
      break;

    case LSM303AGR_MG_ODR_50Hz:
      *Odr = 50.0f;
      break;

    case LSM303AGR_MG_ODR_100Hz:
      *Odr = 100.0f;
      break;

    default:
      ret = -1;
      break;
  }

  return ret;
}






 
int32_t LSM303AGR_MAG_SetOutputDataRate(LSM303AGR_MAG_Object_t *pObj, float Odr)
{
  lsm303agr_mg_odr_m_t new_odr;

  new_odr = (Odr <= 10.000f) ? LSM303AGR_MG_ODR_10Hz
            : (Odr <= 20.000f) ? LSM303AGR_MG_ODR_20Hz
            : (Odr <= 50.000f) ? LSM303AGR_MG_ODR_50Hz
            :                    LSM303AGR_MG_ODR_100Hz;

  if (lsm303agr_mag_data_rate_set(&(pObj->Ctx), new_odr) != 0)
  {
    return -1;
  }

  return 0;
}







 
int32_t LSM303AGR_MAG_GetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t *FullScale)
{
  *FullScale = 50;

  return 0;
}






 
int32_t LSM303AGR_MAG_SetFullScale(LSM303AGR_MAG_Object_t *pObj, int32_t FullScale)
{
  return 0;
}






 
int32_t LSM303AGR_MAG_GetAxesRaw(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_AxesRaw_t *Value)
{
  axis3bit16_t data_raw;

   
  if (lsm303agr_magnetic_raw_get(&(pObj->Ctx), data_raw.u8bit) != 0)
  {
    return -1;
  }

   
  Value->x = data_raw.i16bit[0];
  Value->y = data_raw.i16bit[1];
  Value->z = data_raw.i16bit[2];

  return 0;
}






 
int32_t LSM303AGR_MAG_GetAxes(LSM303AGR_MAG_Object_t *pObj, LSM303AGR_Axes_t *MagneticField)
{
  axis3bit16_t data_raw;
  float sensitivity;

   
  if (lsm303agr_magnetic_raw_get(&(pObj->Ctx), data_raw.u8bit) != 0)
  {
    return -1;
  }

   
  (void)LSM303AGR_MAG_GetSensitivity(pObj, &sensitivity);

   
  MagneticField->x = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
  MagneticField->y = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
  MagneticField->z = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

  return 0;
}







 
int32_t LSM303AGR_MAG_Read_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lsm303agr_read_reg(&(pObj->Ctx), Reg, Data, 1) != 0)
  {
    return -1;
  }

  return 0;
}







 
int32_t LSM303AGR_MAG_Write_Reg(LSM303AGR_MAG_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lsm303agr_write_reg(&(pObj->Ctx), Reg, &Data, 1) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LSM303AGR_MAG_Get_DRDY_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *Status)
{
  if (lsm303agr_mag_data_ready_get(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LSM303AGR_MAG_Get_Init_Status(LSM303AGR_MAG_Object_t *pObj, uint8_t *Status)
{
  if (pObj == 0)
  {
    return -1;
  }

  *Status = pObj->is_initialized;

  return 0;
}



 



 






 
static int32_t LSM303AGR_ACC_GetSensitivityHR(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = 0;
  lsm303agr_fs_a_t fullscale;

   
  if (lsm303agr_xl_full_scale_get(&(pObj->Ctx), &fullscale) != 0)
  {
    return -1;
  }

   
  switch (fullscale)
  {
    case LSM303AGR_2g:
      *Sensitivity = (float)0.980f;
      break;

    case LSM303AGR_4g:
      *Sensitivity = (float)1.950f;
      break;

    case LSM303AGR_8g:
      *Sensitivity = (float)3.900f;
      break;

    case LSM303AGR_16g:
      *Sensitivity = (float)11.720f;
      break;

    default:
      ret = -1;
      break;
  }

  return ret;
}






 
static int32_t LSM303AGR_ACC_GetSensitivityNM(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = 0;
  lsm303agr_fs_a_t fullscale;

   
  if (lsm303agr_xl_full_scale_get(&(pObj->Ctx), &fullscale) != 0)
  {
    return -1;
  }

   
  switch (fullscale)
  {
    case LSM303AGR_2g:
      *Sensitivity = (float)3.900f;
      break;

    case LSM303AGR_4g:
      *Sensitivity = (float)7.820f;
      break;

    case LSM303AGR_8g:
      *Sensitivity = (float)15.630f;
      break;

    case LSM303AGR_16g:
      *Sensitivity = (float)46.900f;
      break;

    default:
      ret = -1;
      break;
  }

  return ret;
}






 
static int32_t LSM303AGR_ACC_GetSensitivityLP(LSM303AGR_ACC_Object_t *pObj, float *Sensitivity)
{
  int32_t ret = 0;
  lsm303agr_fs_a_t fullscale;

   
  if (lsm303agr_xl_full_scale_get(&(pObj->Ctx), &fullscale) != 0)
  {
    return -1;
  }

   
  switch (fullscale)
  {
    case LSM303AGR_2g:
      *Sensitivity = (float)15.630f;
      break;

    case LSM303AGR_4g:
      *Sensitivity = (float)31.260f;
      break;

    case LSM303AGR_8g:
      *Sensitivity = (float)62.520f;
      break;

    case LSM303AGR_16g:
      *Sensitivity = (float)187.580f;
      break;

    default:
      ret = -1;
      break;
  }

  return ret;
}






 
static int32_t LSM303AGR_ACC_SetOutputDataRate_When_Enabled(LSM303AGR_ACC_Object_t *pObj, float Odr)
{
  lsm303agr_odr_a_t new_odr;

  new_odr = (Odr <=    1.0f) ? LSM303AGR_XL_ODR_1Hz
            : (Odr <=   10.0f) ? LSM303AGR_XL_ODR_10Hz
            : (Odr <=   25.0f) ? LSM303AGR_XL_ODR_25Hz
            : (Odr <=   50.0f) ? LSM303AGR_XL_ODR_50Hz
            : (Odr <=  100.0f) ? LSM303AGR_XL_ODR_100Hz
            : (Odr <=  200.0f) ? LSM303AGR_XL_ODR_200Hz
            :                    LSM303AGR_XL_ODR_400Hz;

   
  if (lsm303agr_xl_data_rate_set(&(pObj->Ctx), new_odr) != 0)
  {
    return -1;
  }

  return 0;
}






 
static int32_t LSM303AGR_ACC_SetOutputDataRate_When_Disabled(LSM303AGR_ACC_Object_t *pObj, float Odr)
{
  pObj->acc_odr = (Odr <=    1.0f) ? LSM303AGR_XL_ODR_1Hz
                  : (Odr <=   10.0f) ? LSM303AGR_XL_ODR_10Hz
                  : (Odr <=   25.0f) ? LSM303AGR_XL_ODR_25Hz
                  : (Odr <=   50.0f) ? LSM303AGR_XL_ODR_50Hz
                  : (Odr <=  100.0f) ? LSM303AGR_XL_ODR_100Hz
                  : (Odr <=  200.0f) ? LSM303AGR_XL_ODR_200Hz
                  :                    LSM303AGR_XL_ODR_400Hz;

  return 0;
}








 
static int32_t ReadAccRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM303AGR_ACC_Object_t *pObj = (LSM303AGR_ACC_Object_t *)Handle;

  if (pObj->IO.BusType == 0U)  
  {
     
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }
  else    
  {
     
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x40U), pData, Length);
  }
}








 
static int32_t WriteAccRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM303AGR_ACC_Object_t *pObj = (LSM303AGR_ACC_Object_t *)Handle;

  if (pObj->IO.BusType == 0U)  
  {
     
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }
  else    
  {
     
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x40U), pData, Length);
  }
}








 
static int32_t ReadMagRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM303AGR_MAG_Object_t *pObj = (LSM303AGR_MAG_Object_t *)Handle;

  if (pObj->IO.BusType == 0U)  
  {
     
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }
  else    
  {
     
    return pObj->IO.ReadReg(pObj->IO.Address, (Reg | 0x40U), pData, Length);
  }
}








 
static int32_t WriteMagRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  LSM303AGR_MAG_Object_t *pObj = (LSM303AGR_MAG_Object_t *)Handle;

  if (pObj->IO.BusType == 0U)  
  {
     
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x80U), pData, Length);
  }
  else    
  {
     
    return pObj->IO.WriteReg(pObj->IO.Address, (Reg | 0x40U), pData, Length);
  }
}



 



 



 



 

 

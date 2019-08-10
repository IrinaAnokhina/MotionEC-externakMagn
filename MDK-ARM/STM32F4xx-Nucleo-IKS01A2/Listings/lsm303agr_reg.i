#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.c"


































 

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




 







 
#line 38 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lsm303agr\\lsm303agr_reg.c"







 








 










 
int32_t lsm303agr_read_reg(lsm303agr_ctx_t* ctx, uint8_t reg, uint8_t* data,
                         uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}










 
int32_t lsm303agr_write_reg(lsm303agr_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}




 






 

float_t lsm303agr_from_fs_2g_hr_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 16.0f ) * 0.98f;
}

float_t lsm303agr_from_fs_4g_hr_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 16.0f ) * 1.95f;
}

float_t lsm303agr_from_fs_8g_hr_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 16.0f ) * 3.9f;
}

float_t lsm303agr_from_fs_16g_hr_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 16.0f ) * 11.72f;
}

float_t lsm303agr_from_lsb_hr_to_celsius(int16_t lsb)
{
  return ( ( (float_t)lsb / 64.0f ) / 4.0f ) + 25.0f;
}

float_t lsm303agr_from_fs_2g_nm_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 64.0f ) * 3.9f;
}

float_t lsm303agr_from_fs_4g_nm_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 64.0f ) * 7.82f;
}

float_t lsm303agr_from_fs_8g_nm_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 64.0f ) * 15.63f;
}

float_t lsm303agr_from_fs_16g_nm_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 64.0f ) * 46.9f;
}

float_t lsm303agr_from_lsb_nm_to_celsius(int16_t lsb)
{
  return ( ( (float_t)lsb / 64.0f ) / 4.0f ) + 25.0f;
}

float_t lsm303agr_from_fs_2g_lp_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 256.0f ) * 15.63f;
}

float_t lsm303agr_from_fs_4g_lp_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 256.0f ) * 31.26f;
}

float_t lsm303agr_from_fs_8g_lp_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 256.0f ) * 62.52f;
}

float_t lsm303agr_from_fs_16g_lp_to_mg(int16_t lsb)
{
  return ((float_t)lsb / 256.0f ) * 187.58f;
}

float_t lsm303agr_from_lsb_lp_to_celsius(int16_t lsb)
{
  return ( ( (float_t)lsb / 256.0f ) * 1.0f ) + 25.0f;
}

float_t lsm303agr_from_lsb_to_mgauss(int16_t lsb)
{
  return (float_t)lsb * 1.5f;
}




 






 








 
int32_t lsm303agr_temp_status_reg_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x07U, buff, 1);
  return ret;
}








 
int32_t lsm303agr_temp_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_status_reg_aux_a_t status_reg_aux_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x07U,
                           (uint8_t*)&status_reg_aux_a, 1);
  *val = status_reg_aux_a.tda;

  return ret;
}








 
int32_t lsm303agr_temp_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_status_reg_aux_a_t status_reg_aux_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x07U,
                           (uint8_t*)&status_reg_aux_a, 1);
  *val = status_reg_aux_a.tor;

  return ret;
}








 
int32_t lsm303agr_temperature_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x0CU, buff, 2);
  return ret;
}








 
int32_t lsm303agr_temperature_meas_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_temp_en_a_t val)
{
  lsm303agr_temp_cfg_reg_a_t temp_cfg_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x1FU,
                           (uint8_t*)&temp_cfg_reg_a, 1);
  if(ret == 0){
    temp_cfg_reg_a.temp_en = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x1FU,
                              (uint8_t*)&temp_cfg_reg_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_temperature_meas_get(lsm303agr_ctx_t *ctx,
                                      lsm303agr_temp_en_a_t *val)
{
  lsm303agr_temp_cfg_reg_a_t temp_cfg_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x1FU,
                           (uint8_t*)&temp_cfg_reg_a, 1);
  switch (temp_cfg_reg_a.temp_en){
    case LSM303AGR_TEMP_DISABLE:
      *val = LSM303AGR_TEMP_DISABLE;
      break;
    case LSM303AGR_TEMP_ENABLE:
      *val = LSM303AGR_TEMP_ENABLE;
      break;
    default:
      *val = LSM303AGR_TEMP_DISABLE;
      break;
  }

  return ret;
}









 
int32_t lsm303agr_xl_operating_mode_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_op_md_a_t val)
{
  lsm303agr_ctrl_reg1_a_t ctrl_reg1_a;
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;
  uint8_t lpen, hr;

  if ( val == LSM303AGR_HR_12bit ){
    lpen = 0;
    hr   = 1;
  } else if (val == LSM303AGR_NM_10bit) {
    lpen = 0;
    hr   = 0;
  } else {
    lpen = 1;
    hr   = 0;
  }

  ret = lsm303agr_read_reg(ctx, 0x20U,
                           (uint8_t*)&ctrl_reg1_a, 1);
  ctrl_reg1_a.lpen = (uint8_t)lpen;
  if(ret == 0){
    ret = lsm303agr_write_reg(ctx, 0x20U,
                              (uint8_t*)&ctrl_reg1_a, 1);
  }
  if(ret == 0){
    ret = lsm303agr_read_reg(ctx, 0x23U,
                             (uint8_t*)&ctrl_reg4_a, 1);
  }
  if(ret == 0){
    ctrl_reg4_a.hr = hr;
    ret = lsm303agr_write_reg(ctx, 0x23U,
                              (uint8_t*)&ctrl_reg4_a, 1);
  }

  return ret;
}









 
int32_t lsm303agr_xl_operating_mode_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_op_md_a_t *val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  lsm303agr_ctrl_reg1_a_t ctrl_reg1_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x20U,
                           (uint8_t*)&ctrl_reg1_a, 1);
  if(ret == 0){
    ret = lsm303agr_read_reg(ctx, 0x23U,
                             (uint8_t*)&ctrl_reg4_a, 1);
  }

  if ( ctrl_reg1_a.lpen != (0U) ){
    *val = LSM303AGR_LP_8bit;
  } else if (ctrl_reg4_a.hr  != (0U) ) {
    *val = LSM303AGR_HR_12bit;
  } else{
    *val = LSM303AGR_NM_10bit;
  }

  return ret;
}








 
int32_t lsm303agr_xl_data_rate_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_odr_a_t val)
{
  lsm303agr_ctrl_reg1_a_t ctrl_reg1_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x20U,
                           (uint8_t*)&ctrl_reg1_a, 1);
  if(ret == 0){
    ctrl_reg1_a.odr = (uint8_t)val;
   ret = lsm303agr_write_reg(ctx, 0x20U,
                             (uint8_t*)&ctrl_reg1_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_data_rate_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_odr_a_t *val)
{
  lsm303agr_ctrl_reg1_a_t ctrl_reg1_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x20U,
                           (uint8_t*)&ctrl_reg1_a, 1);

  switch (ctrl_reg1_a.odr){
    case LSM303AGR_XL_POWER_DOWN:
      *val = LSM303AGR_XL_POWER_DOWN;
      break;
    case LSM303AGR_XL_ODR_1Hz:
      *val = LSM303AGR_XL_ODR_1Hz;
      break;
    case LSM303AGR_XL_ODR_10Hz:
      *val = LSM303AGR_XL_ODR_10Hz;
      break;
    case LSM303AGR_XL_ODR_25Hz:
      *val = LSM303AGR_XL_ODR_25Hz;
      break;
    case LSM303AGR_XL_ODR_50Hz:
      *val = LSM303AGR_XL_ODR_50Hz;
      break;
    case LSM303AGR_XL_ODR_100Hz:
      *val = LSM303AGR_XL_ODR_100Hz;
      break;
    case LSM303AGR_XL_ODR_200Hz:
      *val = LSM303AGR_XL_ODR_200Hz;
      break;
    case LSM303AGR_XL_ODR_400Hz:
      *val = LSM303AGR_XL_ODR_400Hz;
      break;
    case LSM303AGR_XL_ODR_1kHz620_LP:
      *val = LSM303AGR_XL_ODR_1kHz620_LP;
      break;
    case LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP:
      *val = LSM303AGR_XL_ODR_1kHz344_NM_HP_5kHz376_LP;
      break;
    default:
      *val = LSM303AGR_XL_POWER_DOWN;
      break;
  }

  return ret;
}








 
int32_t lsm303agr_xl_high_pass_on_outputs_set(lsm303agr_ctx_t *ctx,
                                              uint8_t val)
{
  lsm303agr_ctrl_reg2_a_t ctrl_reg2_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x21U,
                           (uint8_t*)&ctrl_reg2_a, 1);
  if(ret == 0){
    ctrl_reg2_a.fds = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x21U,
                              (uint8_t*)&ctrl_reg2_a, 1);
  }

  return ret;
}









 
int32_t lsm303agr_xl_high_pass_on_outputs_get(lsm303agr_ctx_t *ctx,
                                              uint8_t *val)
{
  lsm303agr_ctrl_reg2_a_t ctrl_reg2_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x21U,
                           (uint8_t*)&ctrl_reg2_a, 1);
  *val = ctrl_reg2_a.fds;

  return ret;
}















 
int32_t lsm303agr_xl_high_pass_bandwidth_set(lsm303agr_ctx_t *ctx,
                                             lsm303agr_hpcf_a_t val)
{
  lsm303agr_ctrl_reg2_a_t ctrl_reg2_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x21U,
                           (uint8_t*)&ctrl_reg2_a, 1);
  if(ret == 0){
    ctrl_reg2_a.hpcf = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x21U,
                              (uint8_t*)&ctrl_reg2_a, 1);
  }

  return ret;
}















 
int32_t lsm303agr_xl_high_pass_bandwidth_get(lsm303agr_ctx_t *ctx,
                                             lsm303agr_hpcf_a_t *val)
{
  lsm303agr_ctrl_reg2_a_t ctrl_reg2_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x21U,
                           (uint8_t*)&ctrl_reg2_a, 1);

  switch (ctrl_reg2_a.hpcf){
    case LSM303AGR_AGGRESSIVE:
      *val = LSM303AGR_AGGRESSIVE;
      break;
    case LSM303AGR_STRONG:
      *val = LSM303AGR_STRONG;
      break;
    case LSM303AGR_MEDIUM:
      *val = LSM303AGR_MEDIUM;
      break;
    case LSM303AGR_LIGHT:
      *val = LSM303AGR_LIGHT;
      break;
    default:
      *val = LSM303AGR_AGGRESSIVE;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_high_pass_mode_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hpm_a_t val)
{
  lsm303agr_ctrl_reg2_a_t ctrl_reg2_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x21U,
                           (uint8_t*)&ctrl_reg2_a, 1);
  if(ret == 0){
    ctrl_reg2_a.hpm = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x21U,
                              (uint8_t*)&ctrl_reg2_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_high_pass_mode_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_hpm_a_t *val)
{
  lsm303agr_ctrl_reg2_a_t ctrl_reg2_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x21U,
                           (uint8_t*)&ctrl_reg2_a, 1);

  switch (ctrl_reg2_a.hpm){
    case LSM303AGR_NORMAL_WITH_RST:
      *val = LSM303AGR_NORMAL_WITH_RST;
      break;
    case LSM303AGR_REFERENCE_MODE:
      *val = LSM303AGR_REFERENCE_MODE;
      break;
    case LSM303AGR_NORMAL:
      *val = LSM303AGR_NORMAL;
      break;
    case LSM303AGR_AUTORST_ON_INT:
      *val = LSM303AGR_AUTORST_ON_INT;
      break;
    default:
      *val = LSM303AGR_NORMAL_WITH_RST;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_full_scale_set(lsm303agr_ctx_t *ctx,
                                    lsm303agr_fs_a_t val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);
  if(ret == 0){
    ctrl_reg4_a.fs = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x23U,
                              (uint8_t*)&ctrl_reg4_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_full_scale_get(lsm303agr_ctx_t *ctx,
                                    lsm303agr_fs_a_t *val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);

  switch (ctrl_reg4_a.fs){
    case LSM303AGR_2g:
      *val = LSM303AGR_2g;
      break;
    case LSM303AGR_4g:
      *val = LSM303AGR_4g;
      break;
    case LSM303AGR_8g:
      *val = LSM303AGR_8g;
      break;
    case LSM303AGR_16g:
      *val = LSM303AGR_16g;
      break;
    default:
      *val = LSM303AGR_2g;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_block_data_update_set(lsm303agr_ctx_t *ctx,
                                           uint8_t val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);
  if(ret == 0){
    ctrl_reg4_a.bdu = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x23U,
                              (uint8_t*)&ctrl_reg4_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_block_data_update_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);
  *val = ctrl_reg4_a.bdu;

  return ret;
}









 
int32_t lsm303agr_xl_filter_reference_set(lsm303agr_ctx_t *ctx,
                                          uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x26U, buff, 1);
  return ret;
}









 
int32_t lsm303agr_xl_filter_reference_get(lsm303agr_ctx_t *ctx,
                                          uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x26U, buff, 1);
  return ret;
}








 
int32_t lsm303agr_xl_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_status_reg_a_t status_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x27U,
                           (uint8_t*)&status_reg_a, 1);
  *val = status_reg_a.zyxda;

  return ret;
}








 
int32_t lsm303agr_xl_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_status_reg_a_t status_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x27U,
                           (uint8_t*)&status_reg_a, 1);
  *val = status_reg_a.zyxor;

  return ret;
}








 
int32_t lsm303agr_acceleration_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x28U, buff, 6);
  return ret;
}















 
int32_t lsm303agr_mag_user_offset_set(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x45U, buff, 6);
  return ret;
}















 
int32_t lsm303agr_mag_user_offset_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x45U, buff, 6);
  return ret;
}








 
int32_t lsm303agr_mag_operating_mode_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_md_m_t val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  if(ret == 0){
    cfg_reg_a_m.md = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x60U,
                              (uint8_t*)&cfg_reg_a_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_operating_mode_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_md_m_t *val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);

    switch (cfg_reg_a_m.md){
    case LSM303AGR_CONTINUOUS_MODE:
      *val = LSM303AGR_CONTINUOUS_MODE;
      break;
    case LSM303AGR_SINGLE_TRIGGER:
      *val = LSM303AGR_SINGLE_TRIGGER;
      break;
    case LSM303AGR_POWER_DOWN:
      *val = LSM303AGR_POWER_DOWN;
      break;
    default:
      *val = LSM303AGR_CONTINUOUS_MODE;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_mag_data_rate_set(lsm303agr_ctx_t *ctx,
                                    lsm303agr_mg_odr_m_t val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  if(ret == 0){
    cfg_reg_a_m.odr = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x60U,
                              (uint8_t*)&cfg_reg_a_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_data_rate_get(lsm303agr_ctx_t *ctx,
                                    lsm303agr_mg_odr_m_t *val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);

    switch (cfg_reg_a_m.odr){
    case LSM303AGR_MG_ODR_10Hz:
      *val = LSM303AGR_MG_ODR_10Hz;
      break;
    case LSM303AGR_MG_ODR_20Hz:
      *val = LSM303AGR_MG_ODR_20Hz;
      break;
    case LSM303AGR_MG_ODR_50Hz:
      *val = LSM303AGR_MG_ODR_50Hz;
      break;
    case LSM303AGR_MG_ODR_100Hz:
      *val = LSM303AGR_MG_ODR_100Hz;
      break;
    default:
      *val = LSM303AGR_MG_ODR_10Hz;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_mag_power_mode_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_lp_m_t val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  if(ret == 0){
    cfg_reg_a_m.lp = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x60U,
                              (uint8_t*)&cfg_reg_a_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_power_mode_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_lp_m_t *val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);

    switch (cfg_reg_a_m.lp){
    case LSM303AGR_HIGH_RESOLUTION:
      *val = LSM303AGR_HIGH_RESOLUTION;
      break;
    case LSM303AGR_LOW_POWER:
      *val = LSM303AGR_LOW_POWER;
      break;
    default:
      *val = LSM303AGR_HIGH_RESOLUTION;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_mag_offset_temp_comp_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  if(ret == 0){
    cfg_reg_a_m.comp_temp_en = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x60U,
                              (uint8_t*)&cfg_reg_a_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_offset_temp_comp_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  *val = cfg_reg_a_m.comp_temp_en;

  return ret;
}








 
int32_t lsm303agr_mag_low_pass_bandwidth_set(lsm303agr_ctx_t *ctx,
                                             lsm303agr_lpf_m_t val)
{
  lsm303agr_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x61U,
                           (uint8_t*)&cfg_reg_b_m, 1);
  if(ret == 0){
    cfg_reg_b_m.lpf = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x61U,
                              (uint8_t*)&cfg_reg_b_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_low_pass_bandwidth_get(lsm303agr_ctx_t *ctx,
                                             lsm303agr_lpf_m_t *val)
{
  lsm303agr_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x61U,
                           (uint8_t*)&cfg_reg_b_m, 1);

    switch (cfg_reg_b_m.lpf){
    case LSM303AGR_ODR_DIV_2:
      *val = LSM303AGR_ODR_DIV_2;
      break;
    case LSM303AGR_ODR_DIV_4:
      *val = LSM303AGR_ODR_DIV_4;
      break;
    default:
      *val = LSM303AGR_ODR_DIV_2;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_mag_set_rst_mode_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_set_rst_m_t val)
{
  lsm303agr_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x61U,
                           (uint8_t*)&cfg_reg_b_m, 1);
  if(ret == 0){
    cfg_reg_b_m.set_rst = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x61U,
                              (uint8_t*)&cfg_reg_b_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_set_rst_mode_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_set_rst_m_t *val)
{
  lsm303agr_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x61U,
                           (uint8_t*)&cfg_reg_b_m, 1);

    switch (cfg_reg_b_m.set_rst){
    case LSM303AGR_SET_SENS_ODR_DIV_63:
      *val = LSM303AGR_SET_SENS_ODR_DIV_63;
      break;
    case LSM303AGR_SENS_OFF_CANC_EVERY_ODR:
      *val = LSM303AGR_SENS_OFF_CANC_EVERY_ODR;
      break;
    case LSM303AGR_SET_SENS_ONLY_AT_POWER_ON:
      *val = LSM303AGR_SET_SENS_ONLY_AT_POWER_ON;
      break;
    default:
      *val = LSM303AGR_SET_SENS_ODR_DIV_63;
      break;
  }
  return ret;
}














 
int32_t lsm303agr_mag_set_rst_sensor_single_set(lsm303agr_ctx_t *ctx,
                                                uint8_t val)
{
  lsm303agr_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x61U,
                           (uint8_t*)&cfg_reg_b_m, 1);
  if(ret == 0){
    cfg_reg_b_m.off_canc_one_shot = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x61U,
                              (uint8_t*)&cfg_reg_b_m, 1);
  }

  return ret;
}















 
int32_t lsm303agr_mag_set_rst_sensor_single_get(lsm303agr_ctx_t *ctx,
                                                uint8_t *val)
{
  lsm303agr_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x61U,
                           (uint8_t*)&cfg_reg_b_m, 1);
  *val = cfg_reg_b_m.off_canc_one_shot;

  return ret;
}








 
int32_t lsm303agr_mag_block_data_update_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  if(ret == 0){
    cfg_reg_c_m.bdu = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x62U,
                              (uint8_t*)&cfg_reg_c_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_block_data_update_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  *val = cfg_reg_c_m.bdu;

  return ret;
}








 
int32_t lsm303agr_mag_data_ready_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_status_reg_m_t status_reg_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x67U,
                           (uint8_t*)&status_reg_m, 1);
  *val = status_reg_m.zyxda;

  return ret;
}








 
int32_t lsm303agr_mag_data_ovr_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_status_reg_m_t status_reg_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x67U,
                           (uint8_t*)&status_reg_m, 1);
  *val = status_reg_m.zyxor;

  return ret;
}








 
int32_t lsm303agr_magnetic_raw_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x68U, buff, 6);
  return ret;
}




 






 








 
int32_t lsm303agr_xl_device_id_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x0FU, buff, 1);
  return ret;
}








 
int32_t lsm303agr_xl_self_test_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_st_a_t val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);
  if(ret == 0){
    ctrl_reg4_a.st = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x23U,
                              (uint8_t*)&ctrl_reg4_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_self_test_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_st_a_t *val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);

    switch (ctrl_reg4_a.st){
    case LSM303AGR_ST_DISABLE:
      *val = LSM303AGR_ST_DISABLE;
      break;
    case LSM303AGR_ST_POSITIVE:
      *val = LSM303AGR_ST_POSITIVE;
      break;
    case LSM303AGR_ST_NEGATIVE:
      *val = LSM303AGR_ST_NEGATIVE;
      break;
    default:
      *val = LSM303AGR_ST_DISABLE;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_data_format_set(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ble_a_t val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);
  if(ret == 0){
    ctrl_reg4_a.ble = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x23U,
                              (uint8_t*)&ctrl_reg4_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_data_format_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_ble_a_t *val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);

    switch (ctrl_reg4_a.ble){
    case LSM303AGR_XL_LSB_AT_LOW_ADD:
      *val = LSM303AGR_XL_LSB_AT_LOW_ADD;
      break;
    case LSM303AGR_XL_MSB_AT_LOW_ADD:
      *val = LSM303AGR_XL_MSB_AT_LOW_ADD;
      break;
    default:
      *val = LSM303AGR_XL_LSB_AT_LOW_ADD;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_boot_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  if(ret == 0){
    ctrl_reg5_a.boot = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x24U,
                              (uint8_t*)&ctrl_reg5_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_boot_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  *val = ctrl_reg5_a.boot;

  return ret;
}








 
int32_t lsm303agr_xl_status_get(lsm303agr_ctx_t *ctx,
                                lsm303agr_status_reg_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x27U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_mag_device_id_get(lsm303agr_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x4FU, buff, 1);
  return ret;
}








 
int32_t lsm303agr_mag_reset_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  if(ret == 0){
    cfg_reg_a_m.soft_rst = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x60U,
                              (uint8_t*)&cfg_reg_a_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_reset_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  *val = cfg_reg_a_m.soft_rst;

  return ret;
}








 
int32_t lsm303agr_mag_boot_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  if(ret == 0){
    cfg_reg_a_m.reboot = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x60U,
                              (uint8_t*)&cfg_reg_a_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_boot_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_cfg_reg_a_m_t cfg_reg_a_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x60U,
                           (uint8_t*)&cfg_reg_a_m, 1);
  *val = cfg_reg_a_m.reboot;

  return ret;
}








 
int32_t lsm303agr_mag_self_test_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  if(ret == 0){
    cfg_reg_c_m.self_test = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x62U,
                              (uint8_t*)&cfg_reg_c_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_self_test_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  *val = cfg_reg_c_m.self_test;

  return ret;
}








 
int32_t lsm303agr_mag_data_format_set(lsm303agr_ctx_t *ctx,
                                      lsm303agr_ble_m_t val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  if(ret == 0){
    cfg_reg_c_m.ble = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x62U,
                              (uint8_t*)&cfg_reg_c_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_data_format_get(lsm303agr_ctx_t *ctx,
                                      lsm303agr_ble_m_t *val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);

    switch (cfg_reg_c_m.ble){
    case LSM303AGR_MG_LSB_AT_LOW_ADD:
      *val = LSM303AGR_MG_LSB_AT_LOW_ADD;
      break;
    case LSM303AGR_MG_MSB_AT_LOW_ADD:
      *val = LSM303AGR_MG_MSB_AT_LOW_ADD;
      break;
    default:
      *val = LSM303AGR_MG_LSB_AT_LOW_ADD;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_mag_status_get(lsm303agr_ctx_t *ctx,
                                 lsm303agr_status_reg_m_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x67U, (uint8_t*) val, 1);
  return ret;
}




 







 








 
int32_t lsm303agr_xl_int1_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int1_cfg_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x30U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_xl_int1_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int1_cfg_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x30U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_xl_int1_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int1_src_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x31U, (uint8_t*) val, 1);
  return ret;
}










 
int32_t lsm303agr_xl_int1_gen_threshold_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303agr_int1_ths_a_t int1_ths_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x32U,
                           (uint8_t*)&int1_ths_a, 1);
  if(ret == 0){
    int1_ths_a.ths = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x32U,
                              (uint8_t*)&int1_ths_a, 1);
  }

  return ret;
}










 
int32_t lsm303agr_xl_int1_gen_threshold_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303agr_int1_ths_a_t int1_ths_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x32U,
                           (uint8_t*)&int1_ths_a, 1);
  *val = int1_ths_a.ths;

  return ret;
}









 
int32_t lsm303agr_xl_int1_gen_duration_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_int1_duration_a_t int1_duration_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x33U,
                           (uint8_t*)&int1_duration_a, 1);
  if(ret == 0){
    int1_duration_a.d = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x33U,
                              (uint8_t*)&int1_duration_a, 1);
  }

  return ret;
}









 
int32_t lsm303agr_xl_int1_gen_duration_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_int1_duration_a_t int1_duration_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x33U,
                           (uint8_t*)&int1_duration_a, 1);
  *val = int1_duration_a.d;

  return ret;
}




 







 








 
int32_t lsm303agr_xl_int2_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int2_cfg_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x34U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_xl_int2_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int2_cfg_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x34U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_xl_int2_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int2_src_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x35U, (uint8_t*) val, 1);
  return ret;
}










 
int32_t lsm303agr_xl_int2_gen_threshold_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303agr_int2_ths_a_t int2_ths_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x36U,
                           (uint8_t*)&int2_ths_a, 1);
  if(ret == 0){
    int2_ths_a.ths = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x36U,
                              (uint8_t*)&int2_ths_a, 1);
  }

  return ret;
}










 
int32_t lsm303agr_xl_int2_gen_threshold_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303agr_int2_ths_a_t int2_ths_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x36U,
                           (uint8_t*)&int2_ths_a, 1);
  *val = int2_ths_a.ths;

  return ret;
}









 
int32_t lsm303agr_xl_int2_gen_duration_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_int2_duration_a_t int2_duration_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x37U,
                           (uint8_t*)&int2_duration_a, 1);
  if(ret == 0){
    int2_duration_a.d = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x37U,
                              (uint8_t*)&int2_duration_a, 1);
  }

  return ret;
}









 
int32_t lsm303agr_xl_int2_gen_duration_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *val)
{
  lsm303agr_int2_duration_a_t int2_duration_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x37U,
                           (uint8_t*)&int2_duration_a, 1);
  *val = int2_duration_a.d;

  return ret;
}




 







 








 
int32_t lsm303agr_xl_high_pass_int_conf_set(lsm303agr_ctx_t *ctx,
                                            lsm303agr_hp_a_t val)
{
  lsm303agr_ctrl_reg2_a_t ctrl_reg2_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x21U,
                           (uint8_t*)&ctrl_reg2_a, 1);
  if(ret == 0){
    ctrl_reg2_a.hp = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x21U,
                              (uint8_t*)&ctrl_reg2_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_high_pass_int_conf_get(lsm303agr_ctx_t *ctx,
                                            lsm303agr_hp_a_t *val)
{
  lsm303agr_ctrl_reg2_a_t ctrl_reg2_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x21U,
                           (uint8_t*)&ctrl_reg2_a, 1);

    switch (ctrl_reg2_a.hp){
    case LSM303AGR_DISC_FROM_INT_GENERATOR:
      *val = LSM303AGR_DISC_FROM_INT_GENERATOR;
      break;
    case LSM303AGR_ON_INT1_GEN:
      *val = LSM303AGR_ON_INT1_GEN;
      break;
    case LSM303AGR_ON_INT2_GEN:
      *val = LSM303AGR_ON_INT2_GEN;
      break;
    case LSM303AGR_ON_TAP_GEN:
      *val = LSM303AGR_ON_TAP_GEN;
      break;
    case LSM303AGR_ON_INT1_INT2_GEN:
      *val = LSM303AGR_ON_INT1_INT2_GEN;
      break;
    case LSM303AGR_ON_INT1_TAP_GEN:
      *val = LSM303AGR_ON_INT1_TAP_GEN;
      break;
    case LSM303AGR_ON_INT2_TAP_GEN:
      *val = LSM303AGR_ON_INT2_TAP_GEN;
      break;
    case LSM303AGR_ON_INT1_INT2_TAP_GEN:
      *val = LSM303AGR_ON_INT1_INT2_TAP_GEN;
      break;
    default:
      *val = LSM303AGR_DISC_FROM_INT_GENERATOR;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_pin_int1_config_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg3_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x22U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_xl_pin_int1_config_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg3_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x22U, (uint8_t*) val, 1);
  return ret;
}









 
int32_t lsm303agr_xl_int2_pin_detect_4d_set(lsm303agr_ctx_t *ctx,
                                            uint8_t val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  if(ret == 0){
    ctrl_reg5_a.d4d_int2 = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x24U,
                              (uint8_t*)&ctrl_reg5_a, 1);
  }

  return ret;
}









 
int32_t lsm303agr_xl_int2_pin_detect_4d_get(lsm303agr_ctx_t *ctx,
                                            uint8_t *val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  *val = ctrl_reg5_a.d4d_int2;

  return ret;
}










 
int32_t lsm303agr_xl_int2pin_notification_mode_set(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int2_a_t val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  if(ret == 0){
    ctrl_reg5_a.lir_int2 = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x24U,
                              (uint8_t*)&ctrl_reg5_a, 1);
  }

  return ret;
}










 
int32_t lsm303agr_xl_int2pin_notification_mode_get(lsm303agr_ctx_t *ctx,
                                                lsm303agr_lir_int2_a_t *val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);

    switch (ctrl_reg5_a.lir_int2){
    case LSM303AGR_INT2_PULSED:
      *val = LSM303AGR_INT2_PULSED;
      break;
    case LSM303AGR_INT2_LATCHED:
      *val = LSM303AGR_INT2_LATCHED;
      break;
    default:
      *val = LSM303AGR_INT2_PULSED;
      break;
  }
  return ret;
}









 
int32_t lsm303agr_xl_int1_pin_detect_4d_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  if(ret == 0){
    ctrl_reg5_a.d4d_int1 = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x24U,
                              (uint8_t*)&ctrl_reg5_a, 1);
  }

  return ret;
}









 
int32_t lsm303agr_xl_int1_pin_detect_4d_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  *val = ctrl_reg5_a.d4d_int1;

  return ret;
}










 
int32_t lsm303agr_xl_int1pin_notification_mode_set(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int1_a_t val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  if(ret == 0){
    ctrl_reg5_a.lir_int1 = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x24U,
                              (uint8_t*)&ctrl_reg5_a, 1);
  }

  return ret;
}










 
int32_t lsm303agr_xl_int1pin_notification_mode_get(lsm303agr_ctx_t *ctx,
                                                   lsm303agr_lir_int1_a_t *val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);

    switch (ctrl_reg5_a.lir_int1){
    case LSM303AGR_INT1_PULSED:
      *val = LSM303AGR_INT1_PULSED;
      break;
    case LSM303AGR_INT1_LATCHED:
      *val = LSM303AGR_INT1_LATCHED;
      break;
    default:
      *val = LSM303AGR_INT1_PULSED;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_pin_int2_config_set(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg6_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x25U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_xl_pin_int2_config_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_ctrl_reg6_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x25U, (uint8_t*) val, 1);
  return ret;
}




 

  





 










 
int32_t lsm303agr_mag_offset_int_conf_set(lsm303agr_ctx_t *ctx,
                                          lsm303agr_int_on_dataoff_m_t val)
{
  lsm303agr_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x61U,
                           (uint8_t*)&cfg_reg_b_m, 1);
  if(ret == 0){
    cfg_reg_b_m.int_on_dataoff = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x61U,
                              (uint8_t*)&cfg_reg_b_m, 1);
  }

  return ret;
}










 
int32_t lsm303agr_mag_offset_int_conf_get(lsm303agr_ctx_t *ctx,
                                          lsm303agr_int_on_dataoff_m_t *val)
{
  lsm303agr_cfg_reg_b_m_t cfg_reg_b_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x61U,
                           (uint8_t*)&cfg_reg_b_m, 1);

    switch (cfg_reg_b_m.int_on_dataoff){
    case LSM303AGR_CHECK_BEFORE:
      *val = LSM303AGR_CHECK_BEFORE;
      break;
    case LSM303AGR_CHECK_AFTER:
      *val = LSM303AGR_CHECK_AFTER;
      break;
    default:
      *val = LSM303AGR_CHECK_BEFORE;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_mag_drdy_on_pin_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  if(ret == 0){
    cfg_reg_c_m.int_mag = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x62U,
                              (uint8_t*)&cfg_reg_c_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_drdy_on_pin_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  *val = cfg_reg_c_m.int_mag;

  return ret;
}








 
int32_t lsm303agr_mag_int_on_pin_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  if(ret == 0){
    cfg_reg_c_m.int_mag_pin = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x62U,
                              (uint8_t*)&cfg_reg_c_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_int_on_pin_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  *val = cfg_reg_c_m.int_mag_pin;

  return ret;
}








 
int32_t lsm303agr_mag_int_gen_conf_set(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int_crtl_reg_m_t *val)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x63U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_mag_int_gen_conf_get(lsm303agr_ctx_t *ctx,
                                       lsm303agr_int_crtl_reg_m_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x63U,
                           (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_mag_int_gen_source_get(lsm303agr_ctx_t *ctx,
                                         lsm303agr_int_source_reg_m_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x64U,
                           (uint8_t*) val, 1);
  return ret;
}











 
int32_t lsm303agr_mag_int_gen_treshold_set(lsm303agr_ctx_t *ctx,
                                           uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x65U, buff, 2);
  return ret;
}











 
int32_t lsm303agr_mag_int_gen_treshold_get(lsm303agr_ctx_t *ctx,
                                           uint8_t *buff)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x65U, buff, 2);
  return ret;
}




 







 








 
int32_t lsm303agr_xl_fifo_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  if(ret == 0){
    ctrl_reg5_a.fifo_en = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x24U,
                              (uint8_t*)&ctrl_reg5_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_fifo_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_ctrl_reg5_a_t ctrl_reg5_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x24U,
                           (uint8_t*)&ctrl_reg5_a, 1);
  *val = ctrl_reg5_a.fifo_en;

  return ret;
}








 
int32_t lsm303agr_xl_fifo_watermark_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_fifo_ctrl_reg_a_t fifo_ctrl_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2EU,
                           (uint8_t*)&fifo_ctrl_reg_a, 1);
  if(ret == 0){
    fifo_ctrl_reg_a.fth = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x2EU,
                              (uint8_t*)&fifo_ctrl_reg_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_fifo_watermark_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_fifo_ctrl_reg_a_t fifo_ctrl_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2EU,
                           (uint8_t*)&fifo_ctrl_reg_a, 1);
  *val = fifo_ctrl_reg_a.fth;

  return ret;
}








 
int32_t lsm303agr_xl_fifo_trigger_event_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_tr_a_t val)
{
  lsm303agr_fifo_ctrl_reg_a_t fifo_ctrl_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2EU,
                           (uint8_t*)&fifo_ctrl_reg_a, 1);
  if(ret == 0){
    fifo_ctrl_reg_a.tr = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x2EU,
                              (uint8_t*)&fifo_ctrl_reg_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_fifo_trigger_event_get(lsm303agr_ctx_t *ctx,
                                            lsm303agr_tr_a_t *val)
{
  lsm303agr_fifo_ctrl_reg_a_t fifo_ctrl_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2EU,
                           (uint8_t*)&fifo_ctrl_reg_a, 1);

    switch (fifo_ctrl_reg_a.tr){
    case LSM303AGR_INT1_GEN:
      *val = LSM303AGR_INT1_GEN;
      break;
    case LSM303AGR_INT2_GEN:
      *val = LSM303AGR_INT2_GEN;
      break;
    default:
      *val = LSM303AGR_INT1_GEN;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_fifo_mode_set(lsm303agr_ctx_t *ctx,
                                   lsm303agr_fm_a_t val)
{
  lsm303agr_fifo_ctrl_reg_a_t fifo_ctrl_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2EU,
                           (uint8_t*)&fifo_ctrl_reg_a, 1);
  if(ret == 0){
    fifo_ctrl_reg_a.fm = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x2EU,
                              (uint8_t*)&fifo_ctrl_reg_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_fifo_mode_get(lsm303agr_ctx_t *ctx,
                                   lsm303agr_fm_a_t *val)
{
  lsm303agr_fifo_ctrl_reg_a_t fifo_ctrl_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2EU,
                           (uint8_t*)&fifo_ctrl_reg_a, 1);

    switch (fifo_ctrl_reg_a.fm){
    case LSM303AGR_BYPASS_MODE:
      *val = LSM303AGR_BYPASS_MODE;
      break;
    case LSM303AGR_FIFO_MODE:
      *val = LSM303AGR_FIFO_MODE;
      break;
    case LSM303AGR_DYNAMIC_STREAM_MODE:
      *val = LSM303AGR_DYNAMIC_STREAM_MODE;
      break;
    case LSM303AGR_STREAM_TO_FIFO_MODE:
      *val = LSM303AGR_STREAM_TO_FIFO_MODE;
      break;
    default:
      *val = LSM303AGR_BYPASS_MODE;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_xl_fifo_status_get(lsm303agr_ctx_t *ctx,
                                     lsm303agr_fifo_src_reg_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x2FU, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_xl_fifo_data_level_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_fifo_src_reg_a_t fifo_src_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2FU,
                           (uint8_t*)&fifo_src_reg_a, 1);
  *val = fifo_src_reg_a.fss;

  return ret;
}








 
int32_t lsm303agr_xl_fifo_empty_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_fifo_src_reg_a_t fifo_src_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2FU,
                           (uint8_t*)&fifo_src_reg_a, 1);
  *val = fifo_src_reg_a.empty;

  return ret;
}








 
int32_t lsm303agr_xl_fifo_ovr_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_fifo_src_reg_a_t fifo_src_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2FU,
                           (uint8_t*)&fifo_src_reg_a, 1);
  *val = fifo_src_reg_a.ovrn_fifo;

  return ret;
}








 
int32_t lsm303agr_xl_fifo_fth_flag_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_fifo_src_reg_a_t fifo_src_reg_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x2FU,
                           (uint8_t*)&fifo_src_reg_a, 1);
  *val = fifo_src_reg_a.wtm;

  return ret;
}




 







 








 
int32_t lsm303agr_tap_conf_set(lsm303agr_ctx_t *ctx,
                               lsm303agr_click_cfg_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_write_reg(ctx, 0x38U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_tap_conf_get(lsm303agr_ctx_t *ctx,
                               lsm303agr_click_cfg_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x38U, (uint8_t*) val, 1);
  return ret;
}








 
int32_t lsm303agr_tap_source_get(lsm303agr_ctx_t *ctx,
                                 lsm303agr_click_src_a_t *val)
{
  int32_t ret;
  ret = lsm303agr_read_reg(ctx, 0x39U, (uint8_t*) val, 1);
  return ret;
}









 
int32_t lsm303agr_tap_threshold_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_click_ths_a_t click_ths_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3AU,
                           (uint8_t*)&click_ths_a, 1);
  if(ret == 0){
    click_ths_a.ths = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x3AU,
                              (uint8_t*)&click_ths_a, 1);
  }

  return ret;
}









 
int32_t lsm303agr_tap_threshold_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_click_ths_a_t click_ths_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3AU,
                           (uint8_t*)&click_ths_a, 1);
  *val = click_ths_a.ths;

  return ret;
}










 
int32_t lsm303agr_shock_dur_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_time_limit_a_t time_limit_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3BU,
                           (uint8_t*)&time_limit_a, 1);
  if(ret == 0){
    time_limit_a.tli = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x3BU,
                              (uint8_t*)&time_limit_a, 1);
  }

  return ret;
}










 
int32_t lsm303agr_shock_dur_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_time_limit_a_t time_limit_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3BU,
                           (uint8_t*)&time_limit_a, 1);
  *val = time_limit_a.tli;

  return ret;
}











 
int32_t lsm303agr_quiet_dur_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_time_latency_a_t time_latency_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3CU,
                           (uint8_t*)&time_latency_a, 1);
  if(ret == 0){
    time_latency_a.tla = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x3CU,
                              (uint8_t*)&time_latency_a, 1);
  }

  return ret;
}










 
int32_t lsm303agr_quiet_dur_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_time_latency_a_t time_latency_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3CU,
                           (uint8_t*)&time_latency_a, 1);
  *val = time_latency_a.tla;

  return ret;
}











 
int32_t lsm303agr_double_tap_timeout_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_time_window_a_t time_window_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3DU,
                           (uint8_t*)&time_window_a, 1);
  if(ret == 0){
    time_window_a.tw = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x3DU,
                              (uint8_t*)&time_window_a, 1);
  }

  return ret;
}











 
int32_t lsm303agr_double_tap_timeout_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_time_window_a_t time_window_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3DU,
                           (uint8_t*)&time_window_a, 1);
  *val = time_window_a.tw;

  return ret;
}




 







 










 
int32_t lsm303agr_act_threshold_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_act_ths_a_t act_ths_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3EU,
                           (uint8_t*)&act_ths_a, 1);
  if(ret == 0){
    act_ths_a.acth = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x3EU,
                              (uint8_t*)&act_ths_a, 1);
  }

  return ret;
}










 
int32_t lsm303agr_act_threshold_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_act_ths_a_t act_ths_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3EU,
                           (uint8_t*)&act_ths_a, 1);
  *val = act_ths_a.acth;

  return ret;
}








 
int32_t lsm303agr_act_timeout_set(lsm303agr_ctx_t *ctx, uint8_t val)
{
  lsm303agr_act_dur_a_t act_dur_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3FU,
                           (uint8_t*)&act_dur_a, 1);
  if(ret == 0){
    act_dur_a.actd = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x3FU,
                              (uint8_t*)&act_dur_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_act_timeout_get(lsm303agr_ctx_t *ctx, uint8_t *val)
{
  lsm303agr_act_dur_a_t act_dur_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x3FU,
                           (uint8_t*)&act_dur_a, 1);
  *val = act_dur_a.actd;

  return ret;
}




 







 








 
int32_t lsm303agr_xl_spi_mode_set(lsm303agr_ctx_t *ctx,
                                  lsm303agr_sim_a_t val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);
  if(ret == 0){
    ctrl_reg4_a.spi_enable = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x23U,
                              (uint8_t*)&ctrl_reg4_a, 1);
  }

  return ret;
}








 
int32_t lsm303agr_xl_spi_mode_get(lsm303agr_ctx_t *ctx,
                                  lsm303agr_sim_a_t *val)
{
  lsm303agr_ctrl_reg4_a_t ctrl_reg4_a;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x23U,
                           (uint8_t*)&ctrl_reg4_a, 1);

    switch (ctrl_reg4_a.spi_enable){
    case LSM303AGR_SPI_4_WIRE:
      *val = LSM303AGR_SPI_4_WIRE;
      break;
    case LSM303AGR_SPI_3_WIRE:
      *val = LSM303AGR_SPI_3_WIRE;
      break;
    default:
      *val = LSM303AGR_SPI_4_WIRE;
      break;
  }
  return ret;
}








 
int32_t lsm303agr_mag_i2c_interface_set(lsm303agr_ctx_t *ctx,
                                        lsm303agr_i2c_dis_m_t val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);
  if(ret == 0){
    cfg_reg_c_m.i2c_dis = (uint8_t)val;
    ret = lsm303agr_write_reg(ctx, 0x62U,
                              (uint8_t*)&cfg_reg_c_m, 1);
  }

  return ret;
}








 
int32_t lsm303agr_mag_i2c_interface_get(lsm303agr_ctx_t *ctx,
                                        lsm303agr_i2c_dis_m_t *val)
{
  lsm303agr_cfg_reg_c_m_t cfg_reg_c_m;
  int32_t ret;

  ret = lsm303agr_read_reg(ctx, 0x62U,
                           (uint8_t*)&cfg_reg_c_m, 1);

    switch (cfg_reg_c_m.i2c_dis){
    case LSM303AGR_I2C_ENABLE:
      *val = LSM303AGR_I2C_ENABLE;
      break;
    case LSM303AGR_I2C_DISABLE:
      *val = LSM303AGR_I2C_DISABLE;
      break;
    default:
      *val = LSM303AGR_I2C_ENABLE;
      break;
  }
  return ret;
}




 

 

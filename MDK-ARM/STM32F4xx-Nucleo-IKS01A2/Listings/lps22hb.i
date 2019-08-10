#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb.c"

































 

 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb.h"

































 

 








 
#line 1 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb_reg.h"



































 
 







 
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






 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb_reg.h"
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





 
#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb_reg.h"




 




 














 

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









 







 

typedef int32_t (*lps22hb_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*lps22hb_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
   
  lps22hb_write_ptr  write_reg;
  lps22hb_read_ptr   read_reg;
   
  void *handle;
} lps22hb_ctx_t;




 





 

   



 





 


typedef struct {
  uint8_t pe               : 2;  
  uint8_t lir              : 1;
  uint8_t diff_en          : 1;
  uint8_t reset_az         : 1;
  uint8_t autozero         : 1;
  uint8_t reset_arp        : 1;
  uint8_t autorifp         : 1;
} lps22hb_interrupt_cfg_t;





typedef struct {
  uint8_t sim              : 1;
  uint8_t bdu              : 1;
  uint8_t lpfp             : 2;  
  uint8_t odr              : 3;
  uint8_t not_used_01      : 1;
} lps22hb_ctrl_reg1_t;


typedef struct {
  uint8_t one_shot         : 1;
  uint8_t not_used_01      : 1;
  uint8_t swreset          : 1;
  uint8_t i2c_dis          : 1;
  uint8_t if_add_inc       : 1;
  uint8_t stop_on_fth      : 1;
  uint8_t fifo_en          : 1;
  uint8_t boot             : 1;
} lps22hb_ctrl_reg2_t;


typedef struct {
  uint8_t int_s            : 2;
  uint8_t drdy             : 1;
  uint8_t f_ovr            : 1;
  uint8_t f_fth            : 1;
  uint8_t f_fss5           : 1;
  uint8_t pp_od            : 1;
  uint8_t int_h_l          : 1;
} lps22hb_ctrl_reg3_t;



typedef struct {
  uint8_t wtm              : 5;
  uint8_t f_mode           : 3;
} lps22hb_fifo_ctrl_t;








typedef struct {
  uint8_t lc_en            : 1;
  uint8_t not_used_01      : 7;
} lps22hb_res_conf_t;


typedef struct {
  uint8_t ph               : 1;
  uint8_t pl               : 1;
  uint8_t ia               : 1;
  uint8_t not_used_01      : 4;
  uint8_t boot_status      : 1;
} lps22hb_int_source_t;


typedef struct {
  uint8_t fss              : 6;
  uint8_t ovr              : 1;
  uint8_t fth_fifo         : 1;
} lps22hb_fifo_status_t;


typedef struct {
  uint8_t p_da             : 1;
  uint8_t t_da             : 1;
  uint8_t not_used_02      : 2;
  uint8_t p_or             : 1;
  uint8_t t_or             : 1;
  uint8_t not_used_01      : 2;
} lps22hb_status_t;

#line 259 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb_reg.h"












 

typedef union{
  lps22hb_interrupt_cfg_t      interrupt_cfg;
  lps22hb_ctrl_reg1_t          ctrl_reg1;
  lps22hb_ctrl_reg2_t          ctrl_reg2;
  lps22hb_ctrl_reg3_t          ctrl_reg3;
  lps22hb_fifo_ctrl_t          fifo_ctrl;
  lps22hb_res_conf_t           res_conf;
  lps22hb_int_source_t         int_source;
  lps22hb_fifo_status_t        fifo_status;
  lps22hb_status_t             status;
  bitwise_t                    bitwise;
  uint8_t                      byte;
} lps22hb_reg_t;




 

int32_t lps22hb_read_reg(lps22hb_ctx_t *ctx, uint8_t reg, uint8_t* data,
                         uint16_t len);
int32_t lps22hb_write_reg(lps22hb_ctx_t *ctx, uint8_t reg, uint8_t* data,
                          uint16_t len);

extern float_t lps22hb_from_lsb_to_hpa(int32_t lsb);
extern float_t lps22hb_from_lsb_to_degc(int16_t lsb);

int32_t lps22hb_autozero_rst_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_autozero_rst_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_autozero_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_autozero_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_snap_rst_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_pressure_snap_rst_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_snap_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_pressure_snap_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_block_data_update_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_block_data_update_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_LPF_ODR_DIV_2  = 0,
  LPS22HB_LPF_ODR_DIV_9  = 2,
  LPS22HB_LPF_ODR_DIV_20 = 3,
} lps22hb_lpfp_t;
int32_t lps22hb_low_pass_filter_mode_set(lps22hb_ctx_t *ctx,
                                         lps22hb_lpfp_t val);
int32_t lps22hb_low_pass_filter_mode_get(lps22hb_ctx_t *ctx,
                                         lps22hb_lpfp_t *val);

typedef enum {
  LPS22HB_POWER_DOWN  = 0,
  LPS22HB_ODR_1_Hz    = 1,
  LPS22HB_ODR_10_Hz   = 2,
  LPS22HB_ODR_25_Hz   = 3,
  LPS22HB_ODR_50_Hz   = 4,
  LPS22HB_ODR_75_Hz   = 5,
} lps22hb_odr_t;
int32_t lps22hb_data_rate_set(lps22hb_ctx_t *ctx, lps22hb_odr_t val);
int32_t lps22hb_data_rate_get(lps22hb_ctx_t *ctx, lps22hb_odr_t *val);

int32_t lps22hb_one_shoot_trigger_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_one_shoot_trigger_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_ref_set(lps22hb_ctx_t *ctx, uint8_t *buff);
int32_t lps22hb_pressure_ref_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_pressure_offset_set(lps22hb_ctx_t *ctx, uint8_t *buff);
int32_t lps22hb_pressure_offset_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_press_data_ready_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_temp_data_ready_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_press_data_ovr_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_temp_data_ovr_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_pressure_raw_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_temperature_raw_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_low_pass_rst_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_device_id_get(lps22hb_ctx_t *ctx, uint8_t *buff);

int32_t lps22hb_reset_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_reset_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_boot_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_boot_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_low_power_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_low_power_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_boot_status_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef struct{
  lps22hb_fifo_status_t  fifo_status;
  lps22hb_status_t       status;
} lps22hb_dev_stat_t;
int32_t lps22hb_dev_status_get(lps22hb_ctx_t *ctx, lps22hb_dev_stat_t *val);

typedef enum {
  LPS22HB_NO_THRESHOLD = 0,
  LPS22HB_POSITIVE     = 1,
  LPS22HB_NEGATIVE     = 2,
  LPS22HB_BOTH         = 3,
} lps22hb_pe_t;
int32_t lps22hb_sign_of_int_threshold_set(lps22hb_ctx_t *ctx,
                                          lps22hb_pe_t val);
int32_t lps22hb_sign_of_int_threshold_get(lps22hb_ctx_t *ctx,
                                          lps22hb_pe_t *val);

typedef enum {
  LPS22HB_INT_PULSED  = 0,
  LPS22HB_INT_LATCHED = 1,
} lps22hb_lir_t;
int32_t lps22hb_int_notification_mode_set(lps22hb_ctx_t *ctx,
                                          lps22hb_lir_t val);
int32_t lps22hb_int_notification_mode_get(lps22hb_ctx_t *ctx,
                                          lps22hb_lir_t *val);

int32_t lps22hb_int_generation_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_int_generation_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_int_threshold_set(lps22hb_ctx_t *ctx, uint8_t *buff);
int32_t lps22hb_int_threshold_get(lps22hb_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LPS22HB_DRDY_OR_FIFO_FLAGS = 0,
  LPS22HB_HIGH_PRES_INT      = 1,
  LPS22HB_LOW_PRES_INT       = 2,
  LPS22HB_EVERY_PRES_INT     = 3,
} lps22hb_int_s_t;
int32_t lps22hb_int_pin_mode_set(lps22hb_ctx_t *ctx, lps22hb_int_s_t val);
int32_t lps22hb_int_pin_mode_get(lps22hb_ctx_t *ctx, lps22hb_int_s_t *val);

int32_t lps22hb_drdy_on_int_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_drdy_on_int_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_ovr_on_int_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_ovr_on_int_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_threshold_on_int_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_threshold_on_int_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_full_on_int_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_full_on_int_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_PUSH_PULL  = 0,
  LPS22HB_OPEN_DRAIN = 1,
} lps22hb_pp_od_t;
int32_t lps22hb_pin_mode_set(lps22hb_ctx_t *ctx, lps22hb_pp_od_t val);
int32_t lps22hb_pin_mode_get(lps22hb_ctx_t *ctx, lps22hb_pp_od_t *val);

typedef enum {
  LPS22HB_ACTIVE_HIGH = 0,
  LPS22HB_ACTIVE_LOW = 1,
} lps22hb_int_h_l_t;
int32_t lps22hb_int_polarity_set(lps22hb_ctx_t *ctx, lps22hb_int_h_l_t val);
int32_t lps22hb_int_polarity_get(lps22hb_ctx_t *ctx, lps22hb_int_h_l_t *val);

int32_t lps22hb_int_source_get(lps22hb_ctx_t *ctx, lps22hb_int_source_t *val);

int32_t lps22hb_int_on_press_high_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_int_on_press_low_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_interrupt_event_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_stop_on_fifo_threshold_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_stop_on_fifo_threshold_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_watermark_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_fifo_watermark_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_BYPASS_MODE           = 0,
  LPS22HB_FIFO_MODE             = 1,
  LPS22HB_STREAM_MODE           = 2,
  LPS22HB_STREAM_TO_FIFO_MODE   = 3,
  LPS22HB_BYPASS_TO_STREAM_MODE = 4,
  LPS22HB_DYNAMIC_STREAM_MODE   = 6,
  LPS22HB_BYPASS_TO_FIFO_MODE   = 7,
} lps22hb_f_mode_t;
int32_t lps22hb_fifo_mode_set(lps22hb_ctx_t *ctx, lps22hb_f_mode_t val);
int32_t lps22hb_fifo_mode_get(lps22hb_ctx_t *ctx, lps22hb_f_mode_t *val);

int32_t lps22hb_fifo_data_level_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_ovr_flag_get(lps22hb_ctx_t *ctx, uint8_t *val);

int32_t lps22hb_fifo_fth_flag_get(lps22hb_ctx_t *ctx, uint8_t *val);

typedef enum {
  LPS22HB_SPI_4_WIRE = 0,
  LPS22HB_SPI_3_WIRE = 1,
} lps22hb_sim_t;
int32_t lps22hb_spi_mode_set(lps22hb_ctx_t *ctx, lps22hb_sim_t val);
int32_t lps22hb_spi_mode_get(lps22hb_ctx_t *ctx, lps22hb_sim_t *val);

typedef enum {
  LPS22HB_I2C_ENABLE = 0,
  LPS22HB_I2C_DISABLE = 1,
} lps22hb_i2c_dis_t;
int32_t lps22hb_i2c_interface_set(lps22hb_ctx_t *ctx, lps22hb_i2c_dis_t val);
int32_t lps22hb_i2c_interface_get(lps22hb_ctx_t *ctx, lps22hb_i2c_dis_t *val);

int32_t lps22hb_auto_add_inc_set(lps22hb_ctx_t *ctx, uint8_t val);
int32_t lps22hb_auto_add_inc_get(lps22hb_ctx_t *ctx, uint8_t *val);




 







 
#line 47 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb.h"
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



 

#line 48 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb.h"



 



 



 



 

typedef int32_t (*LPS22HB_Init_Func)(void);
typedef int32_t (*LPS22HB_DeInit_Func)(void);
typedef int32_t (*LPS22HB_GetTick_Func)(void);
typedef int32_t (*LPS22HB_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*LPS22HB_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);

typedef struct
{
  LPS22HB_Init_Func          Init;
  LPS22HB_DeInit_Func        DeInit;
  uint32_t                   BusType;  
  uint8_t                    Address;
  LPS22HB_WriteReg_Func      WriteReg;
  LPS22HB_ReadReg_Func       ReadReg;
  LPS22HB_GetTick_Func       GetTick;
} LPS22HB_IO_t;

typedef struct
{
  LPS22HB_IO_t        IO;
  lps22hb_ctx_t       Ctx;
  uint8_t             is_initialized;
  uint8_t             press_is_enabled;
  uint8_t             temp_is_enabled;
  lps22hb_odr_t       last_odr;
} LPS22HB_Object_t;

typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} LPS22HB_Capabilities_t;

typedef struct
{
  int32_t (*Init)(LPS22HB_Object_t *);
  int32_t (*DeInit)(LPS22HB_Object_t *);
  int32_t (*ReadID)(LPS22HB_Object_t *, uint8_t *);
  int32_t (*GetCapabilities)(LPS22HB_Object_t *, LPS22HB_Capabilities_t *);
} LPS22HB_CommonDrv_t;

typedef struct
{
  int32_t (*Enable)(LPS22HB_Object_t *);
  int32_t (*Disable)(LPS22HB_Object_t *);
  int32_t (*GetOutputDataRate)(LPS22HB_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS22HB_Object_t *, float);
  int32_t (*GetTemperature)(LPS22HB_Object_t *, float *);
} LPS22HB_TEMP_Drv_t;

typedef struct
{
  int32_t (*Enable)(LPS22HB_Object_t *);
  int32_t (*Disable)(LPS22HB_Object_t *);
  int32_t (*GetOutputDataRate)(LPS22HB_Object_t *, float *);
  int32_t (*SetOutputDataRate)(LPS22HB_Object_t *, float);
  int32_t (*GetPressure)(LPS22HB_Object_t *, float *);
} LPS22HB_PRESS_Drv_t;

typedef enum
{
  LPS22HB_FIFO_BYPASS_MODE                    = (uint8_t)0x00,     
  LPS22HB_FIFO_FIFO_MODE                      = (uint8_t)0x20,     
  LPS22HB_FIFO_STREAM_MODE                    = (uint8_t)0x40,     
  LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,     
  LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,     
  LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0      
} LPS22HB_FifoMode;



 



 












 



 

int32_t LPS22HB_RegisterBusIO(LPS22HB_Object_t *pObj, LPS22HB_IO_t *pIO);
int32_t LPS22HB_Init(LPS22HB_Object_t *pObj);
int32_t LPS22HB_DeInit(LPS22HB_Object_t *pObj);
int32_t LPS22HB_ReadID(LPS22HB_Object_t *pObj, uint8_t *Id);
int32_t LPS22HB_GetCapabilities(LPS22HB_Object_t *pObj, LPS22HB_Capabilities_t *Capabilities);
int32_t LPS22HB_Get_Init_Status(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_PRESS_Enable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_PRESS_Disable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_PRESS_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr);
int32_t LPS22HB_PRESS_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr);
int32_t LPS22HB_PRESS_GetPressure(LPS22HB_Object_t *pObj, float *Value);
int32_t LPS22HB_PRESS_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_PRESS_Get_FThStatus(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_TEMP_Enable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_TEMP_Disable(LPS22HB_Object_t *pObj);
int32_t LPS22HB_TEMP_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr);
int32_t LPS22HB_TEMP_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr);
int32_t LPS22HB_TEMP_GetTemperature(LPS22HB_Object_t *pObj, float *Value);
int32_t LPS22HB_TEMP_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status);

int32_t LPS22HB_FIFO_Get_Data(LPS22HB_Object_t *pObj, float *Press, float *Temp);
int32_t LPS22HB_FIFO_Get_FTh_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Full_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Level(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Get_Ovr_Status(LPS22HB_Object_t *pObj, uint8_t *Status);
int32_t LPS22HB_FIFO_Reset_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt);
int32_t LPS22HB_FIFO_Set_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt);
int32_t LPS22HB_FIFO_Set_Mode(LPS22HB_Object_t *pObj, uint8_t Mode);
int32_t LPS22HB_FIFO_Set_Watermark_Level(LPS22HB_Object_t *pObj, uint8_t Watermark);
int32_t LPS22HB_FIFO_Usage(LPS22HB_Object_t *pObj, uint8_t Status);


int32_t LPS22HB_Read_Reg(LPS22HB_Object_t *pObj, uint8_t reg, uint8_t *Data);
int32_t LPS22HB_Write_Reg(LPS22HB_Object_t *pObj, uint8_t reg, uint8_t Data);

int32_t LPS22HB_Get_Press(LPS22HB_Object_t *pObj, float *Data);
int32_t LPS22HB_Get_Temp(LPS22HB_Object_t *pObj, float *Data);

int32_t LPS22HB_Set_One_Shot(LPS22HB_Object_t *pObj);
int32_t LPS22HB_Get_One_Shot_Status(LPS22HB_Object_t *pObj, uint8_t *Status);



 



 
extern LPS22HB_CommonDrv_t LPS22HB_COMMON_Driver;
extern LPS22HB_PRESS_Drv_t LPS22HB_PRESS_Driver;
extern LPS22HB_TEMP_Drv_t LPS22HB_TEMP_Driver;



 









 



 



 

 
#line 38 "..\\..\\..\\..\\..\\..\\Drivers\\BSP\\Components\\lps22hb\\lps22hb.c"



 



 



 



 

LPS22HB_CommonDrv_t LPS22HB_COMMON_Driver =
{
  LPS22HB_Init,
  LPS22HB_DeInit,
  LPS22HB_ReadID,
  LPS22HB_GetCapabilities,
};

LPS22HB_PRESS_Drv_t LPS22HB_PRESS_Driver =
{
  LPS22HB_PRESS_Enable,
  LPS22HB_PRESS_Disable,
  LPS22HB_PRESS_GetOutputDataRate,
  LPS22HB_PRESS_SetOutputDataRate,
  LPS22HB_PRESS_GetPressure,
};

LPS22HB_TEMP_Drv_t LPS22HB_TEMP_Driver =
{
  LPS22HB_TEMP_Enable,
  LPS22HB_TEMP_Disable,
  LPS22HB_TEMP_GetOutputDataRate,
  LPS22HB_TEMP_SetOutputDataRate,
  LPS22HB_TEMP_GetTemperature,
};



 



 

static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length);
static int32_t LPS22HB_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr);
static int32_t LPS22HB_SetOutputDataRate_When_Enabled(LPS22HB_Object_t *pObj, float Odr);
static int32_t LPS22HB_SetOutputDataRate_When_Disabled(LPS22HB_Object_t *pObj, float Odr);
static int32_t LPS22HB_Initialize(LPS22HB_Object_t *pObj);



 



 





 
int32_t LPS22HB_RegisterBusIO(LPS22HB_Object_t *pObj, LPS22HB_IO_t *pIO)
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

    pObj->Ctx.read_reg  = ReadRegWrap;
    pObj->Ctx.write_reg = WriteRegWrap;
    pObj->Ctx.handle   = pObj;

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
      if (pObj->IO.BusType == 2U)  
      {
         
        if (pObj->is_initialized == 0U)
        {
           
          uint8_t data = 0x01;

          if (LPS22HB_Write_Reg(pObj, 0x10U, data) != 0)
          {
            ret = -1;
          }
        }
      }
    }
  }

  return ret;
}





 
int32_t LPS22HB_Init(LPS22HB_Object_t *pObj)
{
  if (pObj->is_initialized == 0U)
  {
    if (LPS22HB_Initialize(pObj) != 0)
    {
      return -1;
    }
  }

  pObj->is_initialized = 1U;

  return 0;
}





 
int32_t LPS22HB_DeInit(LPS22HB_Object_t *pObj)
{
  if (pObj->is_initialized == 1U)
  {
    if (LPS22HB_PRESS_Disable(pObj) != 0)
    {
      return -1;
    }

    if (LPS22HB_TEMP_Disable(pObj) != 0)
    {
      return -1;
    }
  }

  pObj->is_initialized = 0;

  return 0;
}






 
int32_t LPS22HB_ReadID(LPS22HB_Object_t *pObj, uint8_t *Id)
{
  if (lps22hb_device_id_get(&(pObj->Ctx), Id) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_GetCapabilities(LPS22HB_Object_t *pObj, LPS22HB_Capabilities_t *Capabilities)
{
   
  (void)(pObj);

  Capabilities->Humidity    = 0;
  Capabilities->Pressure    = 1;
  Capabilities->Temperature = 1;
  Capabilities->LowPower    = 0;
  Capabilities->HumMaxOdr   = 0.0f;
  Capabilities->TempMaxOdr  = 75.0f;
  Capabilities->PressMaxOdr = 75.0f;
  return 0;
}






 
int32_t LPS22HB_Get_Init_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (pObj == 0)
  {
    return -1;
  }

  *Status = pObj->is_initialized;

  return 0;
}





 
int32_t LPS22HB_PRESS_Enable(LPS22HB_Object_t *pObj)
{
   
  if (pObj->press_is_enabled == 1U)
  {
    return 0;
  }

   
  if (lps22hb_data_rate_set(&(pObj->Ctx), pObj->last_odr) != 0)
  {
    return -1;
  }

  pObj->press_is_enabled = 1;

  return 0;
}





 
int32_t LPS22HB_PRESS_Disable(LPS22HB_Object_t *pObj)
{
   
  if (pObj->press_is_enabled == 0U)
  {
    return 0;
  }

   
   
  if (pObj->temp_is_enabled == 0U)
  {
     
    if (lps22hb_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != 0)
    {
      return -1;
    }

     
    if (lps22hb_data_rate_set(&(pObj->Ctx), LPS22HB_POWER_DOWN) != 0)
    {
      return -1;
    }
  }

  pObj->press_is_enabled = 0;

  return 0;
}






 
int32_t LPS22HB_PRESS_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr)
{
  return LPS22HB_GetOutputDataRate(pObj, Odr);
}






 
int32_t LPS22HB_PRESS_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr)
{
   
  if (pObj->press_is_enabled == 1U)
  {
    return LPS22HB_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LPS22HB_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}






 
int32_t LPS22HB_PRESS_GetPressure(LPS22HB_Object_t *pObj, float *Value)
{
  axis1bit32_t data_raw_pressure;

  (void)memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
  if (lps22hb_pressure_raw_get(&(pObj->Ctx), data_raw_pressure.u8bit) != 0)
  {
    return -1;
  }

  *Value = lps22hb_from_lsb_to_hpa(data_raw_pressure.i32bit);

  return 0;
}






 
int32_t LPS22HB_PRESS_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_press_data_ready_get(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  return 0;
}





 
int32_t LPS22HB_TEMP_Enable(LPS22HB_Object_t *pObj)
{
   
  if (pObj->temp_is_enabled == 1U)
  {
    return 0;
  }

   
  if (lps22hb_data_rate_set(&(pObj->Ctx), pObj->last_odr) != 0)
  {
    return -1;
  }

  pObj->temp_is_enabled = 1;

  return 0;
}





 
int32_t LPS22HB_TEMP_Disable(LPS22HB_Object_t *pObj)
{
   
  if (pObj->temp_is_enabled == 0U)
  {
    return 0;
  }

   
   
  if (pObj->press_is_enabled == 0U)
  {
     
    if (lps22hb_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != 0)
    {
      return -1;
    }

     
    if (lps22hb_data_rate_set(&(pObj->Ctx), LPS22HB_POWER_DOWN) != 0)
    {
      return -1;
    }
  }

  pObj->temp_is_enabled = 0;

  return 0;
}






 
int32_t LPS22HB_TEMP_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr)
{
  return LPS22HB_GetOutputDataRate(pObj, Odr);
}






 
int32_t LPS22HB_TEMP_SetOutputDataRate(LPS22HB_Object_t *pObj, float Odr)
{
   
  if (pObj->temp_is_enabled == 1U)
  {
    return LPS22HB_SetOutputDataRate_When_Enabled(pObj, Odr);
  }
  else
  {
    return LPS22HB_SetOutputDataRate_When_Disabled(pObj, Odr);
  }
}






 
int32_t LPS22HB_TEMP_GetTemperature(LPS22HB_Object_t *pObj, float *Value)
{
  axis1bit16_t data_raw_temperature;

  (void)memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
  if (lps22hb_temperature_raw_get(&(pObj->Ctx), data_raw_temperature.u8bit) != 0)
  {
    return -1;
  }

  *Value = lps22hb_from_lsb_to_degc(data_raw_temperature.i16bit);

  return 0;
}






 
int32_t LPS22HB_TEMP_Get_DRDY_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_temp_data_ready_get(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Get_Data(LPS22HB_Object_t *pObj, float *Press, float *Temp)
{
  if (LPS22HB_PRESS_GetPressure(pObj, Press) != 0)
  {
    return -1;
  }

  if (LPS22HB_Get_Temp(pObj, Temp) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Get_FTh_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_fifo_fth_flag_get(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Get_Full_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_fifo_data_level_get(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  if (*Status == (uint8_t)0x20)
  {
    *Status = (uint8_t) 1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Get_Level(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_fifo_data_level_get(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Get_Ovr_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  if (lps22hb_fifo_ovr_flag_get(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Reset_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt)
{
  switch (interrupt)
  {
    case 0:
      if (lps22hb_fifo_threshold_on_int_set(&(pObj->Ctx), (0U)) != 0)
      {
        return -1;
      }
      break;
    case 1:
      if (lps22hb_fifo_full_on_int_set(&(pObj->Ctx), (0U)) != 0)
      {
        return -1;
      }
      break;
    case 2:
      if (lps22hb_fifo_ovr_on_int_set(&(pObj->Ctx), (0U)) != 0)
      {
        return -1;
      }
      break;
    default:
      return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Set_Interrupt(LPS22HB_Object_t *pObj, uint8_t interrupt)
{
  switch (interrupt)
  {
    case 0:
      if (lps22hb_fifo_threshold_on_int_set(&(pObj->Ctx), (1U)) != 0)
      {
        return -1;
      }
      break;
    case 1:
      if (lps22hb_fifo_full_on_int_set(&(pObj->Ctx), (1U)) != 0)
      {
        return -1;
      }
      break;
    case 2:
      if (lps22hb_fifo_ovr_on_int_set(&(pObj->Ctx), (1U)) != 0)
      {
        return -1;
      }
      break;
    default:
      return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Set_Mode(LPS22HB_Object_t *pObj, uint8_t Mode)
{

   
  switch ((lps22hb_f_mode_t)Mode)
  {
    case LPS22HB_BYPASS_MODE:
    case LPS22HB_FIFO_MODE:
    case LPS22HB_STREAM_MODE:
    case LPS22HB_STREAM_TO_FIFO_MODE:
    case LPS22HB_BYPASS_TO_STREAM_MODE:
    case LPS22HB_BYPASS_TO_FIFO_MODE:
      break;
    default:
      return -1;
  }

  if (lps22hb_fifo_mode_set(&(pObj->Ctx), (lps22hb_f_mode_t)Mode) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Set_Watermark_Level(LPS22HB_Object_t *pObj, uint8_t Watermark)
{
  if (lps22hb_fifo_watermark_set(&(pObj->Ctx), Watermark) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_FIFO_Usage(LPS22HB_Object_t *pObj, uint8_t Status)
{

  if (lps22hb_fifo_set(&(pObj->Ctx), Status) != 0)
  {
    return -1;
  }

  return 0;
}







 
int32_t LPS22HB_Read_Reg(LPS22HB_Object_t *pObj, uint8_t Reg, uint8_t *Data)
{
  if (lps22hb_read_reg(&(pObj->Ctx), Reg, Data, 1) != 0)
  {
    return -1;
  }

  return 0;
}







 
int32_t LPS22HB_Write_Reg(LPS22HB_Object_t *pObj, uint8_t Reg, uint8_t Data)
{
  if (lps22hb_write_reg(&(pObj->Ctx), Reg, &Data, 1) != 0)
  {
    return -1;
  }

  return 0;
}



 



 






 
int32_t LPS22HB_Get_Temp(LPS22HB_Object_t *pObj, float *Data)
{
  uint8_t buffer[2];
  uint32_t tmp = 0;

   
  if (lps22hb_temperature_raw_get(&(pObj->Ctx), buffer) != 0)
  {
    return -1;
  }

   
  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];

  *Data = ((float)tmp) / 100.0f;

  return 0;
}






 
static int32_t LPS22HB_GetOutputDataRate(LPS22HB_Object_t *pObj, float *Odr)
{
  int32_t ret = 0;
  lps22hb_odr_t odr_low_level;

  if (lps22hb_data_rate_get(&(pObj->Ctx), &odr_low_level) != 0)
  {
    return -1;
  }

  switch (odr_low_level)
  {
    case LPS22HB_POWER_DOWN:
      *Odr = 0.0f;
      break;

    case LPS22HB_ODR_1_Hz:
      *Odr = 1.0f;
      break;

    case LPS22HB_ODR_10_Hz:
      *Odr = 10.0f;
      break;

    case LPS22HB_ODR_25_Hz:
      *Odr = 25.0f;
      break;

    case LPS22HB_ODR_50_Hz:
      *Odr = 50.0f;
      break;

    case LPS22HB_ODR_75_Hz:
      *Odr = 75.0f;
      break;

    default:
      ret = -1;
      break;
  }

  return ret;
}






 
static int32_t LPS22HB_SetOutputDataRate_When_Enabled(LPS22HB_Object_t *pObj, float Odr)
{
  lps22hb_odr_t new_odr;

  new_odr = (Odr <=  1.0f) ? LPS22HB_ODR_1_Hz
            : (Odr <= 10.0f) ? LPS22HB_ODR_10_Hz
            : (Odr <= 25.0f) ? LPS22HB_ODR_25_Hz
            : (Odr <= 50.0f) ? LPS22HB_ODR_50_Hz
            :                  LPS22HB_ODR_75_Hz;

  if (lps22hb_data_rate_set(&(pObj->Ctx), new_odr) != 0)
  {
    return -1;
  }

  if (lps22hb_data_rate_get(&(pObj->Ctx), &pObj->last_odr) != 0)
  {
    return -1;
  }

  return 0;
}






 
static int32_t LPS22HB_SetOutputDataRate_When_Disabled(LPS22HB_Object_t *pObj, float Odr)
{
  pObj->last_odr = (Odr <=  1.0f) ? LPS22HB_ODR_1_Hz
                   : (Odr <= 10.0f) ? LPS22HB_ODR_10_Hz
                   : (Odr <= 25.0f) ? LPS22HB_ODR_25_Hz
                   : (Odr <= 50.0f) ? LPS22HB_ODR_50_Hz
                   :                  LPS22HB_ODR_75_Hz;

  return 0;
}





 
static int32_t LPS22HB_Initialize(LPS22HB_Object_t *pObj)
{
   
  if (lps22hb_low_power_set(&(pObj->Ctx), (1U)) != 0)
  {
    return -1;
  }

   
  if (lps22hb_data_rate_set(&(pObj->Ctx), LPS22HB_POWER_DOWN) != 0)
  {
    return -1;
  }

   
  if (lps22hb_low_pass_filter_mode_set(&(pObj->Ctx), LPS22HB_LPF_ODR_DIV_9) != 0)
  {
    return -1;
  }

  if (lps22hb_block_data_update_set(&(pObj->Ctx), (1U)) != 0)
  {
    return -1;
  }

  if (pObj->IO.BusType == 0U)  
  {
    if (lps22hb_auto_add_inc_set(&(pObj->Ctx), (0U)) != 0)
    {
      return -1;
    }
  }
  else  
  {
    if (lps22hb_auto_add_inc_set(&(pObj->Ctx), (1U)) != 0)
    {
      return -1;
    }
  }

  pObj->last_odr = LPS22HB_ODR_25_Hz;

  return 0;
}





 
int32_t LPS22HB_Set_One_Shot(LPS22HB_Object_t *pObj)
{
   
  if(lps22hb_data_rate_set(&(pObj->Ctx), LPS22HB_POWER_DOWN) != 0)
  {
    return -1;
  }

   
  if(lps22hb_one_shoot_trigger_set(&(pObj->Ctx), 1) != 0)
  {
    return -1;
  }

  return 0;
}






 
int32_t LPS22HB_Get_One_Shot_Status(LPS22HB_Object_t *pObj, uint8_t *Status)
{
  uint8_t p_da;
  uint8_t t_da;

   
  if(lps22hb_press_data_ready_get(&(pObj->Ctx), &p_da) != 0)
  {
    return -1;
  }

   
  if(lps22hb_temp_data_ready_get(&(pObj->Ctx), &t_da) != 0)
  {
    return -1;
  }

  if(p_da && t_da)
  {
    *Status = 1;
  }
  else
  {
    *Status = 0;
  }

  return 0;
}








 
static int32_t ReadRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  uint16_t i;
  int32_t ret = 0;
  LPS22HB_Object_t *pObj = (LPS22HB_Object_t *)Handle;

  if (pObj->IO.BusType == 0U)  
  {
    for (i = 0; i < Length; i++)
    {
      ret = pObj->IO.ReadReg(pObj->IO.Address, (Reg + i), &pData[i], 1);
      if (ret != 0)
      {
        return -1;
      }
    }

    return ret;
  }
  else  
  {
    return pObj->IO.ReadReg(pObj->IO.Address, Reg, pData, Length);
  }
}








 
static int32_t WriteRegWrap(void *Handle, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
  uint16_t i;
  int32_t ret = 0;
  LPS22HB_Object_t *pObj = (LPS22HB_Object_t *)Handle;

  if (pObj->IO.BusType == 0U)  
  {
    for (i = 0; i < Length; i++)
    {
      ret = pObj->IO.WriteReg(pObj->IO.Address, (Reg + i), &pData[i], 1);
      if (ret != 0)
      {
        return -1;
      }
    }

    return ret;
  }
  else  
  {
    return pObj->IO.WriteReg(pObj->IO.Address, Reg, pData, Length);
  }
}



 



 



 



 

 

#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>


/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
//+++rgk change for z reverse Z direction by anxiang.xiao 20140317
#if 0
    .direction = 3,
#else
    .direction = 7,
#endif
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw(void) 
{
    return &cust_acc_hw;
}

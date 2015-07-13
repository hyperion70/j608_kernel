#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/seq_file.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_power_gs.h>

extern unsigned int *AP_CG_gs_dpidle;
extern unsigned int AP_CG_gs_dpidle_len;

extern unsigned int *AP_DCM_gs_dpidle;
extern unsigned int AP_DCM_gs_dpidle_len;

extern unsigned int *PMIC23_LDO_BUCK_gs_dpidle;
extern unsigned int PMIC23_LDO_BUCK_gs_dpidle_len;

extern unsigned int *CHG33_CHG_BUCK_gs_dpidle;
extern unsigned int CHG33_CHG_BUCK_gs_dpidle_len;

void mt_power_gs_dump_dpidle(void)
{
    mt_power_gs_compare("DPIdle",                               \
                        AP_CG_gs_dpidle, AP_CG_gs_dpidle_len,   \
                        AP_DCM_gs_dpidle, AP_DCM_gs_dpidle_len, \
                        PMIC23_LDO_BUCK_gs_dpidle, PMIC23_LDO_BUCK_gs_dpidle_len, \
                        CHG33_CHG_BUCK_gs_dpidle, CHG33_CHG_BUCK_gs_dpidle_len);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
static int dump_dpidle_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "mt_power_gs : dpidle\n");

    mt_power_gs_dump_dpidle();

    len = p - buf;
    return len;
}
#else
static int dump_dpidle_read(struct seq_file *m, void *v)
{
    seq_printf(m, "mt_power_gs : dpidle\n");
    mt_power_gs_dump_dpidle();
    return 0;
}

static int proc_mt_power_gs_dump_dpidle_open(struct inode *inode, struct file *file)
{
    return single_open(file, dump_dpidle_read, NULL);
}
static const struct file_operations mt_power_gs_dump_dpidle_proc_fops = {
    .owner = THIS_MODULE,
    .open  = proc_mt_power_gs_dump_dpidle_open, 
    .read  = seq_read,
};
#endif

static void __exit mt_power_gs_dpidle_exit(void)
{
    //return 0;
}

static int __init mt_power_gs_dpidle_init(void)
{
    struct proc_dir_entry *mt_entry = NULL;

    if (!mt_power_gs_dir)
    {
        printk("[%s]: mkdir /proc/mt_power_gs failed\n", __FUNCTION__);
    }
    else
    {
        #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
        mt_entry = create_proc_entry("dump_dpidle", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir);
        if (mt_entry)
        {
            mt_entry->read_proc = dump_dpidle_read;
        }
        #else
        if (proc_create("dump_dpidle", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir, &mt_power_gs_dump_dpidle_proc_fops) == NULL) {
            pr_err("%s: create_proc_entry dump_dpidle failed\n", __FUNCTION__);
        }
        #endif
    }

    return 0;
}

module_init(mt_power_gs_dpidle_init);
module_exit(mt_power_gs_dpidle_exit);

MODULE_DESCRIPTION("MT Power Golden Setting - dpidle");
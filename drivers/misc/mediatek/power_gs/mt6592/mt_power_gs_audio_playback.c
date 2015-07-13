#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/seq_file.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_power_gs.h>

extern unsigned int *AP_CG_gs_mp3_play;
extern unsigned int AP_CG_gs_mp3_play_len;

extern unsigned int *AP_DCM_gs_mp3_play;
extern unsigned int AP_DCM_gs_mp3_play_len;

extern unsigned int *PMIC23_LDO_BUCK_gs_mp3_play;
extern unsigned int PMIC23_LDO_BUCK_gs_mp3_play_len;

extern unsigned int *CHG33_CHG_BUCK_gs_mp3_play;
extern unsigned int CHG33_CHG_BUCK_gs_mp3_play_len;

void mt_power_gs_dump_audio_playback(void)
{
    mt_power_gs_compare("Audio Playback",                           \
                        AP_CG_gs_mp3_play, AP_CG_gs_mp3_play_len,   \
                        AP_DCM_gs_mp3_play, AP_DCM_gs_mp3_play_len, \
                        PMIC23_LDO_BUCK_gs_mp3_play, PMIC23_LDO_BUCK_gs_mp3_play_len, \
                        CHG33_CHG_BUCK_gs_mp3_play, CHG33_CHG_BUCK_gs_mp3_play_len);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
static int dump_audio_playback_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "mt_power_gs : audio_playback\n");

    mt_power_gs_dump_audio_playback();

    len = p - buf;
    return len;
}
#else
static int dump_audio_playback_read(struct seq_file *m, void *v)
{
    seq_printf(m, "mt_power_gs : audio_playback\n");
    mt_power_gs_dump_audio_playback();
    return 0;
}

static int proc_mt_power_gs_dump_audio_playback_open(struct inode *inode, struct file *file)
{
    return single_open(file, dump_audio_playback_read, NULL);
}
static const struct file_operations mt_power_gs_dump_audio_playback_proc_fops = {
    .owner = THIS_MODULE,
    .open  = proc_mt_power_gs_dump_audio_playback_open, 
    .read  = seq_read,
};
#endif

static void __exit mt_power_gs_audio_playback_exit(void)
{
    //return 0;
}

static int __init mt_power_gs_audio_playback_init(void)
{
    struct proc_dir_entry *mt_entry = NULL;

    if (!mt_power_gs_dir)
    {
        printk("[%s]: mkdir /proc/mt_power_gs failed\n", __FUNCTION__);
    }
    else
    {
        #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
        mt_entry = create_proc_entry("dump_audio_playback", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir);
        if (mt_entry)
        {
            mt_entry->read_proc = dump_audio_playback_read;
        }
        #else
        if (proc_create("dump_audio_playback", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir, &mt_power_gs_dump_audio_playback_proc_fops) == NULL) {
            pr_err("%s: create_proc_entry dump_audio_playback failed\n", __FUNCTION__);
        }
        #endif
    }

    return 0;
}

module_init(mt_power_gs_audio_playback_init);
module_exit(mt_power_gs_audio_playback_exit);

MODULE_DESCRIPTION("MT Power Golden Setting - Audio Playback");
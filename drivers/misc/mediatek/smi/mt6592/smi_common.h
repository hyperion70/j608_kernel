#ifndef __SMI_COMMON_H__
#define __SMI_COMMON_H__


#include <linux/aee.h>


#define SMIMSG(string, args...) if(1){\
 pr_warn("[pid=%d]"string, current->tgid, ##args); \
 }
#define SMIMSG2(string, args...) pr_warn(string, ##args)
#define SMITMP(string, args...) pr_warn("[pid=%d]"string, current->tgid, ##args)
#define SMIERR(string, args...) do{\
	pr_err("error: "string, ##args); \
	aee_kernel_warning(SMI_LOG_TAG, "error: "string, ##args);  \
}while(0)

#define smi_aee_print(string, args...) do{\
    char smi_name[100];\
    snprintf(smi_name,100, "["SMI_LOG_TAG"]"string, ##args); \
  aee_kernel_warning(smi_name, "["SMI_LOG_TAG"]error:"string,##args);  \
}while(0)


#define MAU_ENTRY_NR    3
#define SMI_LARB_NR     6 
#define SMI_ERROR_ADDR  0

// Please use the function to instead gLarbBaseAddr to prevent the NULL pointer access error 
// when the corrosponding larb is not exist
// extern unsigned int gLarbBaseAddr[SMI_LARB_NR];
extern int get_larb_base_addr(int larb_id);
extern char *smi_port_name[][19];

//output_gce_buffer = 1, pass log to CMDQ error dumping messages
int smi_debug_bus_hanging_detect_ext( unsigned int larbs, int show_dump, int output_gce_buffer);
int larb_clock_on(int larb_id);
int larb_clock_off(int larb_id);

void smi_dumpDebugMsg(void);

int mau_init(void);
void SMI_DBG_Init(void);


#endif


#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>

#define SECTION(_name)      __attribute__ ((__section__(_name)))

#define IMAGE2_ENTRY_SECTION                     \
        SECTION(".image2.entry.data")

typedef struct {
	void (*RamStartFun)(void);
	void (*RamWakeupFun)(void);
	uint32_t VectorNS;
} RAM_START_FUNCTION, *PRAM_START_FUNCTION;

uint32_t NewVectorTable[2];

void z_arm_reset(void);

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {
	z_arm_reset,
	NULL,//BOOT_RAM_WakeFromPG,
	(uint32_t)NewVectorTable
};

static int amebadplus_init(void)
{
	/* Update CMSIS SystemCoreClock variable (HCLK) */
	/* At reset, system core clock is set to 4 MHz from MSI */
	// SystemCoreClock = 4000000;

	return 0;
}

SYS_INIT(amebadplus_init, PRE_KERNEL_1, 0);

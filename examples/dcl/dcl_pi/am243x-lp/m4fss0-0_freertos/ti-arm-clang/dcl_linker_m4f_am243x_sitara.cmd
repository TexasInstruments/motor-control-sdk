/*
* This linker file is optional, it's complementary to the device/core linker file
* Here it specifies the dcl function & data mapping allocated by __attribute__((section(...)))
* and link to a specific memory region for the best performance
*/

/* Define core & platform specific variables */
#define FAST_RAM    M4F_IRAM
#define REG_RAM     M4F_IRAM /* M4F_DRAM also works */

SECTIONS
{
   /* dcl functions mapped to on-chip fast ram */
   dclfuncs       : {} > FAST_RAM

    /* optional -- map datas & variables to regular ram */
    /* Sitara default linker should already map variables to ram */
   dclDataSection : {} > REG_RAM
}
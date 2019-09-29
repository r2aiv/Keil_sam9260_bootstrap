/******************************************************************************/
/*  This file is part of the ARM Toolchain package                            */
/*  Copyright KEIL ELEKTRONIK GmbH 2005 - 2010                                */
/******************************************************************************/
/*                                                                            */
/*  FlashHW.H:   Include file for Hardware Layer Flash Programming functions  */
/*                                                                            */
/******************************************************************************/

/* Functions defined in module DFlashHW.c                                     */
int InitFlashController_HW (unsigned char bus_width, unsigned char adr_cycles, unsigned char page_type, unsigned long clk);
int UnInit_HW              (void);
int GetPageSize_HW         (void);
int ErasePage_HW           (unsigned long adr);
int EraseBlock_HW          (unsigned long adr);
int EraseChip_HW           (void);
int CheckBlock_HW          (unsigned long adr);
int MarkBlockBad_HW        (unsigned long adr);
int ReadPage_HW            (unsigned long adr, unsigned long sz, unsigned char *buf);
int ProgramPage_HW         (unsigned long adr, unsigned long sz, unsigned char *buf);
int WaitOperation_HW       (void);


/******************************************************************************/

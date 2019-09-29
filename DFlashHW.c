/******************************************************************************/
/*  This file is part of the ARM Toolchain package                            */
/*  Copyright KEIL ELEKTRONIK GmbH 2005 - 2010                                */
/******************************************************************************/
/*                                                                            */
/*  FlashHW.C:   Hardware Layer of External DataFlash Programming Functions   */
/*               adapted for the Atmel AT91SAM9260 Devices                    */
/*                                                                            */
/******************************************************************************/

#include <AT91SAM9260.H>                // AT91SAM9260 definitions
#include "Error.h"                      // Error definitions

#define SYS_CLOCK    96109714           // System Clock (in Hz)
#define SPI_CLOCK    8000000            // Expected SPI clock speed (in Hz)
#define DLYBS        300                // Delay Before SPCK (in ns)
#define DLYBCT       2000               // Delay Between Consecutive Transfers (in ns)

#define SPI_PCS0_DF  0x0E               // Chip Select 0 : NPCS0 %1110
#define SPI_PCS1_DF  0x0D               // Chip Select 1 : NPCS1 %1101
#define SPI_PCS2_DF  0x0B               // Chip Select 2 : NPCS2 %1011
#define SPI_PCS3_DF  0x07               // Chip Select 3 : NPCS3 %0111

// Constants depending on Chip used

#define ptrSPI       AT91C_BASE_SPI0
#define ID_SPI       AT91C_ID_SPI0

// ----------------- Page Size  528, CS0 used ----------------------------------
#ifdef P528_CS0
#define PAGE_SIZE    528                // Size of page
#define ADDR_BITS    10                 // Bits for addressing inside page
#define CHIP_SIZE    0x420000           // Chip size (in bytes)
#define SPI_PCS_DF   SPI_PCS0_DF
#define CS_INDEX     0
#endif

// ----------------- Page Size 1056, CS0 used ----------------------------------
#ifdef P1056_CS0
#define PAGE_SIZE    1056               // Size of page
#define ADDR_BITS    11                 // Bits for addressing inside page
#define CHIP_SIZE    0x840000           // Chip size (in bytes) = 8MB
#define SPI_PCS_DF   SPI_PCS0_DF
#define CS_INDEX     0
#endif

// ----------------- Page Size 1056, CS1 used ----------------------------------
#ifdef P1056_CS1
#define PAGE_SIZE    1056               // Size of page
#define ADDR_BITS    11                 // Bits for addressing inside page
#define CHIP_SIZE    0x840000           // Chip size (in bytes) = 8MB
#define SPI_PCS_DF   SPI_PCS1_DF
#define CS_INDEX     1
#endif

#define BLOCK_SIZE   8*PAGE_SIZE        // Block size (used for erase)

// READ COMMANDS
#define DB_CONTINUOUS_ARRAY_READ  0xE8  // Continuous array read
#define DB_BURST_ARRAY_READ       0xE8  // Burst array read
#define DB_PAGE_READ              0xD2  // Main memory page read
#define DB_BUF1_READ              0xD4  // Buffer 1 read
#define DB_BUF2_READ              0xD6  // Buffer 2 read
#define DB_STATUS                 0xD7  // Status Register

// PROGRAM and ERASE COMMANDS
#define DB_BUF1_WRITE             0x84  // Buffer 1 write
#define DB_BUF2_WRITE             0x87  // Buffer 2 write
#define DB_BUF1_PAGE_ERS_PGM      0x83  // Buf 1 to main mem page prg w built-In erase               
#define DB_BUF1_PAGE_ERS_FASTPGM  0x93  // Buf 1 to main mem page prg w built-In erase, Fast program 
#define DB_BUF2_PAGE_ERS_PGM      0x86  // Buf 2 to main mem page prg w built-In erase               
#define DB_BUF2_PAGE_ERS_FASTPGM  0x96  // Buf 2 to main mem page prg w built-In erase, Fast program 
#define DB_BUF1_PAGE_PGM          0x88  // Buf 1 to main mem page prg wo built-In erase              
#define DB_BUF1_PAGE_FASTPGM      0x98  // Buf 1 to main mem page prg wo built-In erase, Fast program
#define DB_BUF2_PAGE_PGM          0x89  // Buf 2 to main mem page prg wo built-In erase              
#define DB_BUF2_PAGE_FASTPGM      0x99  // Buf 2 to main mem page prg wo built-In erase, Fast program
#define DB_PAGE_ERASE             0x81  // Page Erase                    
#define DB_BLOCK_ERASE            0x50  // Block Erase                   
#define DB_PAGE_PGM_BUF1          0x82  // Main mem page thru buf 1      
#define DB_PAGE_FASTPGM_BUF1      0x92  // Main mem page thru buf 1, Fast
#define DB_PAGE_PGM_BUF2          0x85  // Main mem page thru buf 2      
#define DB_PAGE_FASTPGM_BUF2      0x95  // Main mem page thru buf 2, Fast

// ADDITIONAL COMMANDS
#define DB_PAGE_2_BUF1_TRF        0x53  // Main mem page to buf 1 transfer
#define DB_PAGE_2_BUF2_TRF        0x55  // Main mem page to buf 2 transfer
#define DB_PAGE_2_BUF1_CMP        0x60  // Main mem page to buf 1 compare
#define DB_PAGE_2_BUF2_CMP        0x61  // Main mem page to buf 2 compare
#define DB_AUTO_PAGE_PGM_BUF1     0x58  // Auto page rewrite thru buf 1
#define DB_AUTO_PAGE_PGM_BUF2     0x59  // Auto page rewrite thru buf 2


static unsigned int    command[2];      // Command for DataFlash

// Module's local functions prototypes
static unsigned long   SwapU32            (unsigned long to_swap);
static          int    SendCommandDF      (unsigned char cmd, unsigned char cmd_sz, unsigned long adr, unsigned char *buf, unsigned long sz);


/************************* Module Exported Functions ***************************/

/*- InitFlashController_HW (...) -----------------------------------------------
 *
 *  Initialize Flash Controller
 *    Parameter:  bus_width:  Bus Width (8, 16 bit)
 *               adr_cycles:  Addressing Cycles (3, 4, 5)
 *                page_type:  Page Type (0 -Small Page, 1 - Large Page)
 *                      clk:  Clock Frequency (Hz)
 *    Return Value:           ERROR code
 */

int InitFlashController_HW (unsigned char bus_width, unsigned char adr_cycles, unsigned char page_type, unsigned long clk) {

  /* 1. Initialize pins for SPI functionality                                 */

  /* Enable clocks                                                            */
  *AT91C_PMC_PCER  = (1 << ID_SPI);
  *AT91C_PMC_PCER  = (1 << AT91C_ID_PIOA);
#if (CS_INDEX == 1)
  *AT91C_PMC_PCER  = (1 << AT91C_ID_PIOC);
#endif

  /* Setup pins                                                               */
  /* MISO  pin: int disable, not pull-up, alt func A select, enable alt func  */
  *AT91C_PIOA_IDR   = AT91C_PA0_SPI0_MISO; 
  *AT91C_PIOA_PPUDR = AT91C_PA0_SPI0_MISO; 
  *AT91C_PIOA_ASR   = AT91C_PA0_SPI0_MISO; 
  *AT91C_PIOA_PDR   = AT91C_PA0_SPI0_MISO; 
  /* MOSI  pin: int disable, not pull-up, alt func A select, enable alt func  */
  *AT91C_PIOA_IDR   = AT91C_PA1_SPI0_MOSI; 
  *AT91C_PIOA_PPUDR = AT91C_PA1_SPI0_MOSI; 
  *AT91C_PIOA_ASR   = AT91C_PA1_SPI0_MOSI; 
  *AT91C_PIOA_PDR   = AT91C_PA1_SPI0_MOSI; 
  /* SPCK  pin: int disable, not pull-up, alt func A select, enable alt func  */
  *AT91C_PIOA_IDR   = AT91C_PA2_SPI0_SPCK; 
  *AT91C_PIOA_PPUDR = AT91C_PA2_SPI0_SPCK; 
  *AT91C_PIOA_ASR   = AT91C_PA2_SPI0_SPCK; 
  *AT91C_PIOA_PDR   = AT91C_PA2_SPI0_SPCK; 
#if   (CS_INDEX == 0)
  /* NPCS0 pin: int disable, not pull-up, alt func A select, enable alt func  */
  *AT91C_PIOA_IDR   = AT91C_PA3_SPI0_NPCS0;
  *AT91C_PIOA_PPUDR = AT91C_PA3_SPI0_NPCS0;
  *AT91C_PIOA_ASR   = AT91C_PA3_SPI0_NPCS0;
  *AT91C_PIOA_PDR   = AT91C_PA3_SPI0_NPCS0;
#elif (CS_INDEX == 1)
  /* NPCS1 pin: int disable, not pull-up, alt func B select, enable alt func  */
  *AT91C_PIOC_IDR   = AT91C_PC11_SPI0_NPCS1;
  *AT91C_PIOC_PPUDR = AT91C_PC11_SPI0_NPCS1;
  *AT91C_PIOC_BSR   = AT91C_PC11_SPI0_NPCS1;
  *AT91C_PIOC_PDR   = AT91C_PC11_SPI0_NPCS1;
#endif

  /* 2. Initialize SPI0 controller                                            */

  ptrSPI->SPI_CR    = AT91C_SPI_SWRST;
  ptrSPI->SPI_MR    = AT91C_SPI_MSTR | AT91C_SPI_MODFDIS | AT91C_SPI_PCS;

  ptrSPI->SPI_CSR[CS_INDEX] =  AT91C_SPI_NCPHA                                                      |
                              (AT91C_SPI_DLYBS  &   ((SYS_CLOCK/1000000*DLYBS/1000)         << 16)) | 
                              (AT91C_SPI_DLYBCT & ((((SYS_CLOCK/1000000*DLYBCT/1000)/32)+1) << 24)) | 
                              ((SYS_CLOCK/SPI_CLOCK) << 8);
  ptrSPI->SPI_MR   &= ~AT91C_SPI_PCS;
  ptrSPI->SPI_MR   |= ((SPI_PCS_DF << 16) & AT91C_SPI_PCS);

  ptrSPI->SPI_CR    = AT91C_SPI_SPIEN;

  // Wait for DataFlash to become ready
  do { SendCommandDF(DB_STATUS, 2, 0, 0, 0); }  while (!(command[0] & 0x8000));

  return (OK);
}


/*- UnInit_HW (...) ------------------------------------------------------------
 *
 *  Uninitialize Hardware to leave it in reset state
 *    Parameter:
 *    Return Value:           ERROR code
 */

int UnInit_HW (void) {

  /* Disable SPI controller                                                   */
  ptrSPI->SPI_CR    = AT91C_SPI_SPIDIS;

  /* Disable SPI pins                                                         */
  *AT91C_PIOA_PER   = AT91C_PA0_SPI0_MISO;
  *AT91C_PIOA_PER   = AT91C_PA1_SPI0_MOSI;
  *AT91C_PIOA_PER   = AT91C_PA2_SPI0_SPCK;
#if   (CS_INDEX == 0)
  *AT91C_PIOA_PER   = AT91C_PA3_SPI0_NPCS0;
#elif (CS_INDEX == 1)
  *AT91C_PIOC_PER   = AT91C_PC11_SPI0_NPCS1;
#endif

  /* Disable clocks                                                           */
  *AT91C_PMC_PCDR   = (1 << AT91C_ID_PIOA);
  *AT91C_PMC_PCDR   = (1 << ID_SPI);

  /* Leave PIOC enabled as SDRAM is connected to it                           */

  return (OK);
}


/*- GetPageSize_HW (...) -------------------------------------------------------
 *
 *  Get Size of Page
 *    Parameter:
 *    Return Value:           page size in bytes
 */

int GetPageSize_HW (void) {

  return (PAGE_SIZE);
}


/*- ErasePage_HW (...) ---------------------------------------------------------
 *
 *  Erase a Page
 *    Parameter:        adr:  Page Start Address
 *    Return Value:           ERROR code
 */

int ErasePage_HW (unsigned long adr) {

  if (SendCommandDF(DB_PAGE_ERASE, 4, adr, 0, 0) != OK)
    return (ERROR_ERASE);

  // Wait for operation to finish
  do { SendCommandDF(DB_STATUS, 2, 0, 0, 0); }  while (!(command[0] & 0x8000));

  return (OK);
}


/*- EraseBlock_HW (...) ---------------------------------------------------------
 *
 *  Erase a Block
 *    Parameter:        adr:  Block Start Address
 *    Return Value:           ERROR code
 */

int EraseBlock_HW (unsigned long adr) {

  if (SendCommandDF(DB_BLOCK_ERASE, 4, adr, 0, 0) != OK)
    return (ERROR_ERASE);

  // Wait for operation to finish
  do { SendCommandDF(DB_STATUS, 2, 0, 0, 0); }  while (!(command[0] & 0x8000));

  return (OK);
}


/*- EraseChip_HW (...) ---------------------------------------------------------
 *
 *  Erase Full Flash Memory
 *    Parameter:
 *    Return Value:           ERROR code
 */

int EraseChip_HW (void) {

  unsigned long i;

  // Because Chip Erase Command (0xC7) in documentation is said not to 
  // be reliable on some chips, chip is erased with multiply Block Erase 
  // Commands (0x50)

  for (i = 0; i < CHIP_SIZE; i += BLOCK_SIZE) {
    if (SendCommandDF(DB_BLOCK_ERASE, 4, i, 0, 0) != OK)
      return (ERROR_ERASE);

    // Wait for operation to finish
    do { SendCommandDF(DB_STATUS, 2, 0, 0, 0); }  while (!(command[0] & 0x8000));
  }

  return (OK);
}


/*- ReadPage_HW (...) ----------------------------------------------------------
 *
 *  Initialize Flash Programming Functions
 *    Parameter:        adr:  Page Start Address
 *                       sz:  Page Size
 *                      buf:  Page Data
 *    Return Value:           ERROR code
 */

int ReadPage_HW (unsigned long adr, unsigned long sz, unsigned char *buf) {

  if (SendCommandDF(DB_CONTINUOUS_ARRAY_READ, 8, adr, buf, sz) != OK)
    return (ERROR_READ);

  // Wait for operation to finish
  do { SendCommandDF(DB_STATUS, 2, 0, 0, 0); }  while (!(command[0] & 0x8000));

  return (OK);
}


/*- ProgramPage_HW (...) -------------------------------------------------------
 *
 *  Initialize Flash Programming Functions
 *    Parameter:        adr:  Page Start Address
 *                       sz:  Page Size
 *                      buf:  Page Data
 *    Return Value:           ERROR code
 */

int ProgramPage_HW (unsigned long adr, unsigned long sz, unsigned char *buf) {

  unsigned int i;

  // Clear (0xFF) rest of a page if not full page is requested for program 
  // because always a whole page must be programmed
  if (sz < PAGE_SIZE) 
    for (i = sz; i < PAGE_SIZE; i++)
      buf[i] = 0xFF;

  // Write Data to Buf1
  if (SendCommandDF(DB_BUF1_WRITE, 4, adr, buf, PAGE_SIZE) != OK)
    return (ERROR_WRITE);

  // Wait for operation to finish
  do { SendCommandDF(DB_STATUS, 2, 0, 0, 0); }  while (!(command[0] & 0x8000));

  // Write Buf1 to DataFlash
  if (SendCommandDF(DB_BUF1_PAGE_PGM, 4, adr, buf, PAGE_SIZE) != OK)
    return (ERROR_WRITE);

  // Wait for operation to finish
  do { SendCommandDF(DB_STATUS, 2, 0, 0, 0); }  while (!(command[0] & 0x8000));

  return (OK);
}


/*- CheckBlock_HW (...) --------------------------------------------------------
 *
 *  Check if block at requested address is valid
 *    Parameter:        adr:  Block Start Address
 *    Return Value:           ERROR code
 */

int CheckBlock_HW (unsigned long adr) {

  return (ERROR_NOT_IMPLEMENTED);
}


/*- MarkBlockBad_HW (...) ------------------------------------------------------
 *
 *  Mark the block as being bad
 *    Parameter:        adr:  Block Start Address
 *    Return Value:           ERROR code
 */

int MarkBlockBad_HW (unsigned long adr) {

  return (ERROR_NOT_IMPLEMENTED);
}


/**************************** Auxiliary Functions ******************************/

/*- SwapU32 (...) --------------------------------------------------------------
 *
 *  Swap big <-> little-endian for 32 bit value
 *    Parameter:
 *    Return Value:           swapped value
 */

static unsigned long SwapU32(unsigned long to_swap)
{
  const unsigned long mask = 0x00FF00FF;
  unsigned long temp;

  __asm {
    AND temp,    mask,    to_swap
    AND to_swap, mask,    to_swap, ROR #24
    ORR to_swap, to_swap, temp,    ROR #8
  }

  return (to_swap);
} 


/*- SendCommandDF (...) --------------------------------------------------------
 *
 *  Send command to DataFlash
 *    Parameter:        cmd:  Command to be Sent
 *                   cmd_sz:  Command Size (in bytes)
 *                      adr:  Address in DataFlash
 *                      buf:  Data to be Sent (or Received)
 *                       sz:  Size of Data
 *    Return Value:           ERROR code
 */

static int SendCommandDF (unsigned char cmd, unsigned char cmd_sz, unsigned long adr, unsigned char *buf, unsigned long sz) {

  unsigned long mul, rem;

  // Divide adr by PAGE_SIZE
  // Instead of using multiplication and division 
  // because it needs library which takes too much 
  // memory
  mul = 0;
  rem = adr;
  while (rem >= PAGE_SIZE) {
    rem -= PAGE_SIZE;
    mul++;
  }

  // Calculate address
  adr = (mul << ADDR_BITS) + (rem); 

  command[0]        = (SwapU32(adr) & 0xFFFFFF00) | cmd;
  command[1]        = 0;
  ptrSPI->SPI_PTCR  = AT91C_PDC_RXTDIS;
  ptrSPI->SPI_RPR   = (unsigned int) &command[0];
  ptrSPI->SPI_RCR   = cmd_sz;
  ptrSPI->SPI_RNPR  = (unsigned int) &buf[0];
  ptrSPI->SPI_RNCR  = sz;

  ptrSPI->SPI_PTCR  = AT91C_PDC_TXTDIS;
  ptrSPI->SPI_TPR   = (unsigned int) &command[0];
  ptrSPI->SPI_TCR   = cmd_sz;
  ptrSPI->SPI_TNPR  = (unsigned int) &buf[0];
  ptrSPI->SPI_TNCR  = sz;

  ptrSPI->SPI_PTCR  = AT91C_PDC_RXTEN;
  ptrSPI->SPI_PTCR  = AT91C_PDC_TXTEN;

  while (!(ptrSPI->SPI_SR & AT91C_SPI_RXBUFF));
  ptrSPI->SPI_PTCR  = AT91C_PDC_RXTDIS;
  ptrSPI->SPI_PTCR  = AT91C_PDC_TXTDIS;

  return (OK);
}


/*******************************************************************************/

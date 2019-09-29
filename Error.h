/******************************************************************************/
/*  This file is part of the ARM Toolchain package                            */
/*  Copyright KEIL ELEKTRONIK GmbH 2005 - 2007                                */
/******************************************************************************/
/*                                                                            */
/*  Error.h        Error codes that functions can return                      */
/*                                                                            */
/******************************************************************************/

/* Error values that functions can return                                     */
typedef enum   
{ OK = 0,                            /* No error                              */
  ERROR_NOT_IMPLEMENTED,             /* Functionality has not been implemented*/
  ERROR_NOT_ALLOWED,                 /* Function does not allow that operation*/
  ERROR_INTERFACE,                   /* Interface with flash controller error */
  ERROR_ECC,                         /* ECC can not repair bit errors         */
  ERROR_BAD_BLOCK,                   /* Accessed block is bad                 */
  ERROR_READ,                        /* Error reading data                    */
  ERROR_WRITE,                       /* Error writing data                    */
  ERROR_ERASE,                       /* Error erasing                         */
} ERROR;

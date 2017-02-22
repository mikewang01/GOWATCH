/***************************************************************************//**
 * @file standards.h
 * @brief Common headers for SSI projects
 *
 * @details This file should be included in all C source code files for SSI
 *          projects that conform to the SSI coding standard.  It defines
 *          standard types, constants, and macros used by all SSI projects.
 *
 * Copyright &copy; 2011 RHT as unpublished work.  All rights reserved.
 *
 * @author  Greg Lindsey
 * @date 10/05/2009
 * @version 1.1
 *
 * @note This file should be included prior to any other includes in a source
 *       file.
 ******************************************************************************/
#ifndef __STANDARDS_H__
#define __STANDARDS_H__
/** @addtogroup STANDARDS */
/** @{ */

/* Platform-specific */
#include "standard_types.h"
#include "standard_macros.h"

/* Platform-agnostic */
#include "standard_status.h"
#include "standard_enums.h"
#include "standard_binary.h"

/** @} */
#endif

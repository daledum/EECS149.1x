/** \file irobotTypes.h
 *
 * Boolean and bit-level functions; common data types
 *
 * \author Jeff C. Jensen
 * \date 2013-12-09
 * \copyright Copyright (C) 2013, Jeff C. Jensen, Edward A. Lee, and Sanjit A. Seshia.
 * 			  This software accompanies An Introductory Lab in Embedded and Cyber-Physical Systems
 * 			  and is licensed under a Creative Commons Attribution-NonCommercial-NoDerivs 3.0
 * 			  Unported License. See http://leeseshia.org/lab.
 */

#ifndef _IROBOTTYPES_H
#define _IROBOTTYPES_H

#include <stdint.h>
#ifdef _MSC_VER
	#ifndef __BOOL_DEFINED
		typedef uint8_t bool;
		#define true ( (bool)1 )
		#define false ( (bool)0 )
		#define __BOOL_DEFINED
	#endif
#else
	#include <stdbool.h>
#endif
#include <stddef.h>

// Byte-level operations - extract bytes from 16 and 32-bit numbers
#define LO(x)					((x) & 0xFF)			///< Low order byte of a 16 bit number
#define HO(x)					((x) >> 0x08)			///< High order byte of a 16 bit number
#define LLO(x)					((x) & 0xFF)			///< Byte 0 (lowest order) byte of a 32-bit number
#define LHO(x)					(((x) >> 0x08) & 0xFF)	///< Byte 1 of a 32-bit number
#define HLO(x)					(((x) >> 0x10) & 0xFF)	///< Byte 2 of a 32-bit number
#define HHO(x)					((x) >> 0x18)			///< Byte 3 (highest order) byte of a 32-bit number

// Byte-level operations - reconstruct 16 and 32-bit numbers from constituent bytes
#define MLO(x)					((uint16_t)( ((uint8_t)(x)) & 0xFF ))	///< Convert a byte to Byte 0 of a 16-bit number
#define MHO(x)					((uint16_t)( MLO(x) << 0x08 )		)	///< Convert a byte to Byte 1 of a 16-bit number
#define MLLO(x)					((uint32_t)( MLO(x) )				)				///< Convert a byte to Byte 0 of a 32-bit number
#define MLHO(x)					((uint32_t)(  (uint32_t)MHO(x) ) 				)	///< Convert a byte to Byte 1 of a 32-bit number
#define MHLO(x)					((uint32_t)( ((uint32_t)MLO(x)) << 0x10 )		)	///< Convert a byte to Byte 2 of a 32-bit number
#define MHHO(x)					((uint32_t)( ((uint32_t)MLO(x)) << 0x18 )		)	///< Convert a byte to Byte 3 of a 32-bit number

// Gives a byte-level checksum of a number
#define CX8(x)					((uint8_t)x) ///< Byte-level checksum of an 8-bit number (identity function)
#define CX16(x)					((uint8_t)(HO(x) + LO(x))	) ///< Byte-level checksum of a 16-bit number; sum of two bytes
#define CX32(x)					((uint8_t)(HHO(x) + HLO(x) + LHO(x) + LLO(x)))	///< Byte-level checksum of a 32-bit number; sum of four bytes

#endif	// _IROBOTTYPES_H

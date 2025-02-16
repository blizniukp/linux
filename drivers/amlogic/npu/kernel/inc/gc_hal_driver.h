/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2019 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2019 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/


#ifndef __gc_hal_driver_h_
#define __gc_hal_driver_h_

#include "gc_hal_enum.h"
#include "gc_hal_types.h"


#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************\
******************************* I/O Control Codes ******************************
\******************************************************************************/

#define gcvHAL_CLASS                    "galcore"
#define IOCTL_GCHAL_INTERFACE           30000
#define IOCTL_GCHAL_KERNEL_INTERFACE    30001
#define IOCTL_GCHAL_TERMINATE           30002

/******************************************************************************\
********************************* Command Codes ********************************
\******************************************************************************/

typedef enum _gceHAL_COMMAND_CODES
{
    /*************** Common ***************/

    /* Chip info: count, type and so on. */
    gcvHAL_CHIP_INFO,

    /* HAL driver version. */
    gcvHAL_VERSION,

    /* Query chip id and options. */
    gcvHAL_QUERY_CHIP_IDENTITY,
    gcvHAL_QUERY_CHIP_OPTION,

    /* Query chip frequency, used by CL. */
    gcvHAL_QUERY_CHIP_FREQUENCY,

    /* Query system pool video memory, used by CL. */
    gcvHAL_QUERY_VIDEO_MEMORY,

    /* Memory management. */
    gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY,
    gcvHAL_WRAP_USER_MEMORY,
    gcvHAL_RELEASE_VIDEO_MEMORY,
    gcvHAL_LOCK_VIDEO_MEMORY,
    gcvHAL_UNLOCK_VIDEO_MEMORY,
    gcvHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY,
    gcvHAL_MAP_MEMORY,
    gcvHAL_UNMAP_MEMORY,

    /* Cache operations. */
    gcvHAL_CACHE,

    /* HAL user attach and detach. */
    gcvHAL_ATTACH,
    gcvHAL_DETACH,

    /* Event commit. */
    gcvHAL_EVENT_COMMIT,

    /* User command commit. */
    gcvHAL_COMMIT,

    /* Set hardware timeout, used by CL. */
    gcvHAL_SET_TIMEOUT,

    /* User signal operations. */
    gcvHAL_USER_SIGNAL,

    /* Event signal, commit stall. */
    gcvHAL_SIGNAL,

    /* Profile related. */
    gcvHAL_SET_PROFILE_SETTING,
    gcvHAL_READ_PROFILER_REGISTER_SETTING,
    gcvHAL_READ_ALL_PROFILE_REGISTERS_PART1,
    gcvHAL_READ_ALL_PROFILE_REGISTERS_PART2,
    /* Query process database info when debug trace and proflie. */
    gcvHAL_DATABASE,

    /* Power managment enable/disable. */
    gcvHAL_CONFIG_POWER_MANAGEMENT,

    /* Debug/dump feature. */
    gcvHAL_DEBUG_DUMP,

    /*************** Common end ***************/

    /*************** GPU only ***************/

    /* Register operations, 2D only. */
    gcvHAL_READ_REGISTER,
    gcvHAL_WRITE_REGISTER,
    gcvHAL_PROFILE_REGISTERS_2D,

    /* Get base address for old mmu. */
    gcvHAL_GET_BASE_ADDRESS,

    /* Read frame database, 3D only. */
    gcvHAL_GET_FRAME_INFO,

    /* Query command buffer, VG only. */
    gcvHAL_QUERY_COMMAND_BUFFER,

    /* Reset time stamp. */
    gcvHAL_QUERY_RESET_TIME_STAMP,

    /* Create native fence. */
    gcvHAL_CREATE_NATIVE_FENCE,

    /* Wait native fence. */
    gcvHAL_WAIT_NATIVE_FENCE,

    /* Wait until GPU finishes access to a resource. */
    gcvHAL_WAIT_FENCE,

    /* Video memory node operations. */
    gcvHAL_EXPORT_VIDEO_MEMORY,
    gcvHAL_NAME_VIDEO_MEMORY,
    gcvHAL_IMPORT_VIDEO_MEMORY,

    /*************** GPU only end ***************/

    /*************** DEC only ***************/
    /* DEC200 test. */
    gcvHAL_DEC200_TEST,

    /* DEC300 related operations. */
    gcvHAL_DEC300_READ,
    gcvHAL_DEC300_WRITE,
    gcvHAL_DEC300_FLUSH,
    gcvHAL_DEC300_FLUSH_WAIT,
    /*************** DEC only end ***************/

    /*************** OS specific ***************/

    /* Android gralloc: shared buffer operations. */
    gcvHAL_SHBUF,

    /* Android gralloc: get graphic buffer fd. */
    gcvHAL_GET_GRAPHIC_BUFFER_FD,

    /* Vsimulator only. */
    gcvHAL_UPDATE_DEBUG_CALLBACK,

    /* Non paged memory management backup compatibility, windows, qnx. */
    gcvHAL_ALLOCATE_NON_PAGED_MEMORY,
    gcvHAL_FREE_NON_PAGED_MEMORY,

    /* Write user data, windows only. */
    gcvHAL_WRITE_DATA,

    /*************** OS specific end ***************/

    /*************** Reserved ***************/
    gcvHAL_SET_IDLE,
    gcvHAL_RESET,

    /* Command commit done. */
    gcvHAL_COMMIT_DONE,

    /* Get video memory file description. */
    gcvHAL_GET_VIDEO_MEMORY_FD,

    /* Get profile setting. */
    gcvHAL_GET_PROFILE_SETTING,

    /* Read/Write register ex. */
    gcvHAL_READ_REGISTER_EX,
    gcvHAL_WRITE_REGISTER_EX,

    /* Power managment state. */
    gcvHAL_SET_POWER_MANAGEMENT_STATE,
    gcvHAL_QUERY_POWER_MANAGEMENT_STATE,

    /* Set debug level. */
    gcvHAL_SET_DEBUG_LEVEL_ZONE,

    /* Dump info. */
    gcvHAL_DUMP_GPU_STATE,
    gcvHAL_DUMP_EVENT,
    gcvHAL_DUMP_GPU_PROFILE,

    /* Timer. */
    gcvHAL_TIMESTAMP,

    /* FSCALE_VAL. */
    gcvHAL_SET_FSCALE_VALUE,
    gcvHAL_GET_FSCALE_VALUE,

    /* Destory MMU. */
    gcvHAL_DESTROY_MMU,
    /*************** Reserved end ***************/
}
gceHAL_COMMAND_CODES;

/******************************************************************************\
****************************** Interface Structure *****************************
\******************************************************************************/

#define gcdMAX_PROFILE_FILE_NAME        128
#define gcdMAX_FLAT_MAPPING_COUNT       16

/* gcvHAL_CHIP_INFO */
typedef struct _gcsHAL_CHIP_INFO
{
    /* Chip count. */
    OUT gctINT32                count;

    /* Chip types. */
    OUT gceHARDWARE_TYPE        types[gcdCHIP_COUNT];

    /* Chip IDs. */
    OUT gctUINT32               ids[gcvCORE_COUNT];
}
gcsHAL_CHIP_INFO;

/* gcvHAL_VERSION */
typedef struct _gcsHAL_VERSION
{
    /* version: <major>.<minor>.<patch>. */
    OUT gctINT32                major;
    OUT gctINT32                minor;
    OUT gctINT32                patch;

    /* Build version. */
    OUT gctUINT32               build;
}
gcsHAL_VERSION;

/* gcvHAL_SET_TIMEOUT. */
typedef struct _gcsHAL_SET_TIMEOUT
{
    gctUINT32                   timeOut;
}
gcsHAL_SET_TIMEOUT;

/* gcvHAL_QUERY_VIDEO_MEMORY */
typedef struct _gcsHAL_QUERY_VIDEO_MEMORY
{
    /* Physical memory address of internal memory. Just a name. */
    OUT gctUINT32               internalPhysName;
    /* Size in bytes of internal memory. */
    OUT gctUINT64               internalSize;

    /* Physical memory address of external memory. Just a name. */
    OUT gctUINT32               externalPhysName;
    /* Size in bytes of external memory.*/
    OUT gctUINT64               externalSize;

    /* Physical memory address of contiguous memory. Just a name. */
    OUT gctUINT32               contiguousPhysName;
    /* Size in bytes of contiguous memory.*/
    OUT gctUINT64               contiguousSize;
}
gcsHAL_QUERY_VIDEO_MEMORY;

/* gcvHAL_QUERY_CHIP_IDENTITY */
typedef struct _gcsHAL_QUERY_CHIP_IDENTITY * gcsHAL_QUERY_CHIP_IDENTITY_PTR;
typedef struct _gcsHAL_QUERY_CHIP_IDENTITY
{

    /* Chip model. */
    gceCHIPMODEL                chipModel;

    /* Revision value.*/
    gctUINT32                   chipRevision;

    /* Chip date. */
    gctUINT32                   chipDate;

    /* Supported feature fields. */
    gctUINT32                   chipFeatures;

    /* Supported minor feature fields. */
    gctUINT32                   chipMinorFeatures;

    /* Supported minor feature 1 fields. */
    gctUINT32                   chipMinorFeatures1;

    /* Supported minor feature 2 fields. */
    gctUINT32                   chipMinorFeatures2;

    /* Supported minor feature 3 fields. */
    gctUINT32                   chipMinorFeatures3;

    /* Supported minor feature 4 fields. */
    gctUINT32                   chipMinorFeatures4;

    /* Supported minor feature 5 fields. */
    gctUINT32                   chipMinorFeatures5;

    /* Supported minor feature 6 fields. */
    gctUINT32                   chipMinorFeatures6;

    /* Number of streams supported. */
    gctUINT32                   streamCount;

    /* Number of pixel pipes. */
    gctUINT32                   pixelPipes;

    /* Number of resolve pipes. */
    gctUINT32                   resolvePipes;

    /* Number of instructions. */
    gctUINT32                   instructionCount;

    /* Number of constants. */
    gctUINT32                   numConstants;

    /* Number of varyings */
    gctUINT32                   varyingsCount;

    /* Number of 3D GPUs */
    gctUINT32                   gpuCoreCount;

    /* Physical mask of all AVAILABLE clusters in core.*/
    gctUINT32                   clusterAvailMask;

    /* Product ID */
    gctUINT32                   productID;

    /* Special chip flag bits */
    gceCHIP_FLAG                chipFlags;

    /* ECO ID. */
    gctUINT32                   ecoID;

    /* Customer ID. */
    gctUINT32                   customerID;

    /* CPU view physical address and size of SRAMs. */
    gctUINT64                   sRAMBases[gcvSRAM_COUNT];
    gctUINT32                   sRAMSizes[gcvSRAM_COUNT];
}
gcsHAL_QUERY_CHIP_IDENTITY;

/* gcvHAL_QUERY_CHIP_OPTION. */
typedef struct _gcsHAL_QUERY_CHIP_OPTIONS * gcsHAL_QUERY_CHIP_OPTIONS_PTR;
typedef struct _gcsHAL_QUERY_CHIP_OPTIONS
{
    gctBOOL                     gpuProfiler;
    gctBOOL                     allowFastClear;
    gctBOOL                     powerManagement;
    /* Whether use new MMU. It is meaningless
    ** for old MMU since old MMU is always enabled.
    */
    gctBOOL                     enableMMU;
    gceCOMPRESSION_OPTION       allowCompression;
    gctBOOL                     smallBatch;
    gctUINT32                   uscL1CacheRatio;
    gctUINT32                   uscAttribCacheRatio;
    gctUINT32                   userClusterMask;

    /* GPU/VIP virtual address of SRAMs. */
    gctUINT32                   sRAMBaseAddresses[gcvSRAM_COUNT];
    /* SRAMs size. */
    gctUINT32                   sRAMSizes[gcvSRAM_COUNT];
    /* GPU/VIP view physical address of SRAMs. */
    gctPHYS_ADDR_T              sRAMPhysicalBases[gcvSRAM_COUNT];

    gceSECURE_MODE              secureMode;

}
gcsHAL_QUERY_CHIP_OPTIONS;

/* gcvHAL_QUERY_CHIP_FREQUENCY. */
typedef struct _gcsHAL_QUERY_CHIP_FREQUENCY * gcsHAL_QUERY_CHIP_FREQUENCY_PTR;
typedef struct _gcsHAL_QUERY_CHIP_FREQUENCY
{
    OUT gctUINT32               mcClk;
    OUT gctUINT32               shClk;
}
gcsHAL_QUERY_CHIP_FREQUENCY;

/* Obsolete for userpace. */
/* gcvHAL_ALLOCATE_NON_PAGED_MEMORY */
typedef struct _gcsHAL_ALLOCATE_NON_PAGED_MEMORY
{
    /* Allocation flags. */
    IN gctUINT32                flags;

    /* Number of bytes to allocate. */
    IN OUT gctUINT64            bytes;

    /* Physical address of allocation. Just a name. */
    OUT gctUINT32               physName;

    /* Logical address of allocation. */
    OUT gctUINT64               logical;
}
gcsHAL_ALLOCATE_NON_PAGED_MEMORY;

/* Obsolete for userpace. */
/* gcvHAL_FREE_NON_PAGED_MEMORY */
typedef struct _gcsHAL_FREE_NON_PAGED_MEMORY
{
    /* Number of bytes allocated. */
    IN gctUINT64                bytes;

    /* Physical address of allocation. Just a name. */
    IN gctUINT32                physName;

    /* Logical address of allocation. */
    IN gctUINT64                logical;
}
gcsHAL_FREE_NON_PAGED_MEMORY;

/* Video memory allocation. */
/* gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY */
typedef struct _gcsHAL_ALLOCATE_LINEAR_VIDEO_MEMORY
{
    /* Number of bytes to allocate. */
    IN OUT gctUINT64            bytes;

    /* Buffer alignment. */
    IN gctUINT32                alignment;

    /* Type of allocation, see gceVIDMEM_TYPE. */
    IN gctUINT32                type;

    /* Flag of allocation. */
    IN gctUINT32                flag;

    /* Memory pool to allocate from. */
    IN OUT gctUINT32            pool;

    /* Allocated video memory. */
    OUT gctUINT32               node;
}
gcsHAL_ALLOCATE_LINEAR_VIDEO_MEMORY;

typedef struct _gcsUSER_MEMORY_DESC
{
    /* Import flag. */
    gctUINT32                  flag;

    /* gcvALLOC_FLAG_DMABUF */
    gctUINT32                  handle;
    gctUINT64                  dmabuf;

    /* gcvALLOC_FLAG_USERMEMORY */
    gctUINT64                  logical;
    gctUINT64                  physical;
    gctUINT32                  size;

    /* gcvALLOC_FLAG_EXTERNAL_MEMORY */
    gcsEXTERNAL_MEMORY_INFO    externalMemoryInfo;
}
gcsUSER_MEMORY_DESC;

/* gcvHAL_WRAP_USER_MEMORY. */
typedef struct _gcsHAL_WRAP_USER_MEMORY
{
    /* Description of user memory. */
    IN gcsUSER_MEMORY_DESC      desc;

    /* Video memory allocation type. */
    IN gctUINT32                type;

    /* Output video mmory node. */
    OUT gctUINT32               node;

    /* size of the node in bytes */
    OUT gctUINT64               bytes;
}
gcsHAL_WRAP_USER_MEMORY;

/* gcvHAL_RELEASE_VIDEO_MEMORY */
typedef struct _gcsHAL_RELEASE_VIDEO_MEMORY
{
    /* Allocated video memory. */
    IN gctUINT32                node;

#ifdef __QNXNTO__
    /* Mapped logical address to unmap in user space. */
    OUT gctUINT64               memory;

    /* Number of bytes to allocated. */
    OUT gctUINT64               bytes;
#endif
}
gcsHAL_RELEASE_VIDEO_MEMORY;

/* gcvHAL_LOCK_VIDEO_MEMORY */
typedef struct _gcsHAL_LOCK_VIDEO_MEMORY
{
    /* Allocated video memory. */
    IN gctUINT32                node;

    /* Cache configuration. */
    /* Only gcvPOOL_VIRTUAL can be configured */
    IN gctBOOL                  cacheable;

    /* Hardware specific address. */
    OUT gctUINT32               address;

    /* Mapped logical address. */
    OUT gctUINT64               memory;

    /* Customer priviate handle*/
    OUT gctUINT32               gid;

    /* Bus address of a contiguous video node. */
    OUT gctUINT64               physicalAddress;
}
gcsHAL_LOCK_VIDEO_MEMORY;

/* gcvHAL_UNLOCK_VIDEO_MEMORY */
typedef struct _gcsHAL_UNLOCK_VIDEO_MEMORY
{
    /* Allocated video memory. */
    IN gctUINT64                node;

    /* Video memory allocation type. */
    IN gctUINT32                type;

    /* Pool of the unlock node */
    OUT gctUINT32               pool;

    /* Bytes of the unlock node */
    OUT gctUINT64               bytes;

    /* Flag to unlock surface asynchroneously. */
    IN OUT gctBOOL              asynchroneous;
}
gcsHAL_UNLOCK_VIDEO_MEMORY;

/* gcvHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY: */
typedef struct _gcsHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY
{
    /* Allocated video memory. */
    IN gctUINT32                node;

    /* Video memory allocation type. */
    IN gctUINT32                type;
}
gcsHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY;

/* gcvHAL_EXPORT_VIDEO_MEMORY. */
typedef struct _gcsHAL_EXPORT_VIDEO_MEMORY
{
    /* Allocated video memory. */
    IN gctUINT32                node;

    /* Export flags */
    IN gctUINT32                flags;

    /* Exported dma_buf fd */
    OUT gctINT32                fd;
}
gcsHAL_EXPORT_VIDEO_MEMORY;

/* gcvHAL_NAME_VIDEO_MEMORY. */
typedef struct _gcsHAL_NAME_VIDEO_MEMORY
{
    IN gctUINT32                handle;
    OUT gctUINT32               name;
}
gcsHAL_NAME_VIDEO_MEMORY;

/* gcvHAL_IMPORT_VIDEO_MEMORY. */
typedef struct _gcsHAL_IMPORT_VIDEO_MEMORY
{
    IN gctUINT32                name;
    OUT gctUINT32               handle;
}
gcsHAL_IMPORT_VIDEO_MEMORY;

/* gcvHAL_MAP_MEMORY */
typedef struct _gcsHAL_MAP_MEMORY
{
    /* Physical memory address to map. Just a name on Linux/Qnx. */
    IN gctUINT32                physName;

    /* Number of bytes in physical memory to map. */
    IN gctUINT64                bytes;

    /* Address of mapped memory. */
    OUT gctUINT64               logical;
}
gcsHAL_MAP_MEMORY;

/* gcvHAL_UNMAP_MEMORY */
typedef struct _gcsHAL_UNMAP_MEMORY
{
    /* Physical memory address to unmap. Just a name on Linux/Qnx. */
    IN gctUINT32                physName;

    /* Number of bytes in physical memory to unmap. */
    IN gctUINT64                bytes;

    /* Address of mapped memory to unmap. */
    IN gctUINT64                logical;
}
gcsHAL_UNMAP_MEMORY;

/* gcvHAL_CACHE */
typedef struct _gcsHAL_CACHE
{
    IN gceCACHEOPERATION        operation;
    IN gctUINT64                process;
    IN gctUINT64                logical;
    IN gctUINT64                bytes;
    IN gctUINT32                node;
}
gcsHAL_CACHE;

/* gcvHAL_ATTACH */
typedef struct _gcsHAL_ATTACH
{
    /* Handle of context buffer object. */
    OUT gctUINT32               context;

    /* Maximum state in the buffer. */
    OUT gctUINT64               maxState;

    /* Number of states in the buffer. */
    OUT gctUINT32               numStates;

    /* Map context buffer to user or not. */
    IN gctBOOL                  map;

    /* Physical of context buffer. */
    OUT gctUINT64               logicals[2];

    /* Bytes of context buffer. */
    OUT gctUINT32               bytes;
}
gcsHAL_ATTACH;

/* gcvHAL_DETACH */
typedef struct _gcsHAL_DETACH
{
    /* Context buffer object gckCONTEXT. Just a name. */
    IN gctUINT32                context;
}
gcsHAL_DETACH;


/* gcvHAL_EVENT_COMMIT. */
typedef struct _gcsHAL_EVENT_COMMIT
{
    /* Event queue in gcsQUEUE. */
    IN gctUINT64                queue;
}
gcsHAL_EVENT_COMMIT;

typedef struct _gcsHAL_COMMAND_LOCATION
{
    gctUINT32                   priority;
    gctUINT32                   channelId;

    gctUINT32                   videoMemNode;

    gctUINT32                   address;
    gctUINT64                   logical;
    gctUINT32                   startOffset;
    /* size includes reservedHead and reservedTail. */
    gctUINT32                   size;

    gctUINT32                   reservedHead;
    gctUINT32                   reservedTail;

    /* Pointer to patch list. */
    gctUINT64                   patchHead;

    /*
     * Location index of exit commands, ie where to put the chipEnable/link back
     * commands in the reservedTail area.
     * It's used in fully shared command buffer for multiple cores.
     */
    gctUINT32                   exitIndex;
    gctUINT32                   entryPipe;
    gctUINT32                   exitPipe;

    /* struct _gcsHAL_COMMAND_LOCATION * next; */
    gctUINT64                   next;
}
gcsHAL_COMMAND_LOCATION;

typedef struct _gcsHAL_SUBCOMMIT
{
    gctUINT32                   coreId;

    /* user gcsSTATE_DELTA_PTR. */
    gctUINT64                   delta;

    /* Kernel gckCONTEXT. */
    gctUINT64                   context;

    /* Event queue in user gcsQUEUE *. */
    gctUINT64                   queue;

    /* Locate the commands. */
    gcsHAL_COMMAND_LOCATION     commandBuffer;

    /* struct _gcsHAL_SUBCOMMIT * next; */
    gctUINT64                   next;
}
gcsHAL_SUBCOMMIT;

/* gcvHAL_COMMIT */
typedef struct _gcsHAL_COMMIT
{
    gcsHAL_SUBCOMMIT            subCommit;

    gctBOOL                     shared;

    /* Commit stamp of this commit. */
    OUT gctUINT64               commitStamp;
}
gcsHAL_COMMIT;


typedef struct _gcsHAL_COMMIT_DONE
{
    IN gctUINT64                context;
}
gcsHAL_COMMIT_DONE;

/* gcvHAL_USER_SIGNAL  */
typedef struct _gcsHAL_USER_SIGNAL
{
    /* Command. */
    gceUSER_SIGNAL_COMMAND_CODES command;

    /* Signal ID. */
    IN OUT gctINT32             id;

    /* Reset mode. */
    IN gctBOOL                  manualReset;

    /* Wait timedout. */
    IN gctUINT32                wait;

    /* State. */
    IN gctBOOL                  state;
}
gcsHAL_USER_SIGNAL;

/* gcvHAL_SIGNAL. */
typedef struct _gcsHAL_SIGNAL
{
    /* Signal handle to signal gctSIGNAL. */
    IN gctUINT64                signal;

    /* Reserved gctSIGNAL. */
    IN gctUINT64                auxSignal;

    /* Process owning the signal gctHANDLE. */
    IN gctUINT64                process;

#if defined(__QNXNTO__)
    /* Client pulse side-channel connection ID. Set by client in gcoOS_CreateSignal. */
    IN gctINT32                 coid;

    /* Set by server. */
    IN gctINT32                 rcvid;
#endif
    /* Event generated from where of pipeline */
    IN gceKERNEL_WHERE          fromWhere;
}
gcsHAL_SIGNAL;

/* gcvHAL_WRITE_DATA. */
typedef struct _gcsHAL_WRITE_DATA
{
    /* Address to write data to. */
    IN gctUINT32                address;

    /* Data to write. */
    IN gctUINT32                data;
}
gcsHAL_WRITE_DATA;

/* gcvHAL_READ_REGISTER */
typedef struct _gcsHAL_READ_REGISTER
{
    /* Logical address of memory to write data to. */
    IN gctUINT32                address;

    /* Data read. */
    OUT gctUINT32               data;
}
gcsHAL_READ_REGISTER;

/* gcvHAL_WRITE_REGISTER */
typedef struct _gcsHAL_WRITE_REGISTER
{
    /* Logical address of memory to write data to. */
    IN gctUINT32                address;

    /* Data read. */
    IN gctUINT32                data;
}
gcsHAL_WRITE_REGISTER;

/* gcvHAL_READ_REGISTER_EX */
typedef struct _gcsHAL_READ_REGISTER_EX
{
    /* Logical address of memory to write data to. */
    IN gctUINT32                address;

    IN gctUINT32                coreSelect;

    /* Data read. */
    OUT gctUINT32               data[4];
}
gcsHAL_READ_REGISTER_EX;

/* gcvHAL_WRITE_REGISTER_EX */
typedef struct _gcsHAL_WRITE_REGISTER_EX
{
    /* Logical address of memory to write data to. */
    IN gctUINT32                address;

    IN gctUINT32                coreSelect;

    /* Data read. */
    IN gctUINT32                data[4];
}
gcsHAL_WRITE_REGISTER_EX;

#if VIVANTE_PROFILER
/* gcvHAL_GET_PROFILE_SETTING */
typedef struct _gcsHAL_GET_PROFILE_SETTING
{
    /* Enable profiling */
    OUT gctBOOL                 enable;
}
gcsHAL_GET_PROFILE_SETTING;

/* gcvHAL_SET_PROFILE_SETTING */
typedef struct _gcsHAL_SET_PROFILE_SETTING
{
    /* Enable profiling */
    IN gctBOOL                  enable;
}
gcsHAL_SET_PROFILE_SETTING;

/* gcvHAL_READ_PROFILER_REGISTER_SETTING */
typedef struct _gcsHAL_READ_PROFILER_REGISTER_SETTING
{
    /*Should Clear Register*/
    IN gctBOOL                  bclear;
}
gcsHAL_READ_PROFILER_REGISTER_SETTING;

typedef struct _gcsHAL_READ_ALL_PROFILE_REGISTERS_PART1
{
    /* Context buffer object gckCONTEXT. Just a name. */
    IN gctUINT32                context;

    /* Data read. */
    OUT gcsPROFILER_COUNTERS_PART1 Counters;
}
gcsHAL_READ_ALL_PROFILE_REGISTERS_PART1;

typedef struct _gcsHAL_READ_ALL_PROFILE_REGISTERS_PART2
{
    /* Context buffer object gckCONTEXT. Just a name. */
    IN gctUINT32                context;

    /* Data read. */
    OUT gcsPROFILER_COUNTERS_PART2 Counters;
}
gcsHAL_READ_ALL_PROFILE_REGISTERS_PART2;

/* gcvHAL_PROFILE_REGISTERS_2D */
typedef struct _gcsHAL_PROFILE_REGISTERS_2D
{
    /* Data read in gcs2D_PROFILE. */
    OUT gctUINT64               hwProfile2D;
}
gcsHAL_PROFILE_REGISTERS_2D;
#endif

/* gcvHAL_SET_POWER_MANAGEMENT_STATE */
typedef struct _gcsHAL_SET_POWER_MANAGEMENT
{
    /* Data read. */
    IN gceCHIPPOWERSTATE        state;
}
gcsHAL_SET_POWER_MANAGEMENT;

/* gcvHAL_QUERY_POWER_MANAGEMENT_STATE */
typedef struct _gcsHAL_QUERY_POWER_MANAGEMENT
{
    /* Data read. */
    OUT gceCHIPPOWERSTATE       state;

    /* Idle query. */
    OUT gctBOOL                 isIdle;
}
gcsHAL_QUERY_POWER_MANAGEMENT;

/* gcvHAL_CONFIG_POWER_MANAGEMENT. */
typedef struct _gcsHAL_CONFIG_POWER_MANAGEMENT
{
    IN gctBOOL                  enable;
}
gcsHAL_CONFIG_POWER_MANAGEMENT;

typedef struct _gcsFLAT_MAPPING_RANGE
{
    gctUINT64 start;
    gctUINT64 end;
    gctUINT32 size;
    gceFLATMAP_FLAG flag;
}
gcsFLAT_MAPPING_RANGE;

/* gcvHAL_GET_BASE_ADDRESS */
typedef struct _gcsHAL_GET_BASE_ADDRESS
{
    /* Physical memory address of internal memory. */
    OUT gctUINT32               baseAddress;

    OUT gctUINT32               flatMappingRangeCount;

    OUT gcsFLAT_MAPPING_RANGE   flatMappingRanges[gcdMAX_FLAT_MAPPING_COUNT];
}
gcsHAL_GET_BASE_ADDRESS;

typedef struct _gcsHAL_SET_DEBUG_LEVEL_ZONE
{
    IN gctUINT32                level;
    IN gctUINT32                zones;
    IN gctBOOL                  enable;
}
gcsHAL_SET_DEBUG_LEVEL_ZONE;

/* gcvHAL_DEBUG_DUMP. */
typedef struct _gcsHAL_DEBUG_DUMP
{
    /* gceDUMP_BUFFER_TYPE      type. */
    IN gctUINT32                type;

    IN gctUINT64                ptr;
    IN gctUINT32                address;
    IN gctUINT32                size;
}
gcsHAL_DEBUG_DUMP;


/* gcvHAL_TIMESTAMP */
typedef struct _gcsHAL_TIMESTAMP
{
    /* Timer select. */
    IN gctUINT32                timer;

    /* Timer request type (0-stop, 1-start, 2-send delta). */
    IN gctUINT32                request;

    /* Result of delta time in microseconds. */
    OUT gctINT32                timeDelta;
}
gcsHAL_TIMESTAMP;

/* gcvHAL_DATABASE */
typedef struct _gcsHAL_DATABASE
{
    /* Set to gcvTRUE if you want to query a particular process ID.
    ** Set to gcvFALSE to query the last detached process. */
    IN gctBOOL                  validProcessID;

    /* Process ID to query. */
    IN gctUINT32                processID;

    /* Information. */
    OUT gcuDATABASE_INFO        vidMem;
    OUT gcuDATABASE_INFO        nonPaged;
    OUT gcuDATABASE_INFO        gpuIdle;

    /* Detail information about video memory. */
    OUT gcuDATABASE_INFO        vidMemPool[3];
}
gcsHAL_DATABASE;

/* gcvHAL_GET_FRAME_INFO. */
typedef struct _gcsHAL_GET_FRAME_INFO
{
    /* gcsHAL_FRAME_INFO* */
    OUT gctUINT64     frameInfo;
}
gcsHAL_GET_FRAME_INFO;


typedef struct _gcsHAL_SET_FSCALE_VALUE
{
    IN gctUINT32                value;
}
gcsHAL_SET_FSCALE_VALUE;

typedef struct _gcsHAL_GET_FSCALE_VALUE
{
    OUT gctUINT32               value;
    OUT gctUINT32               minValue;
    OUT gctUINT32               maxValue;
}
gcsHAL_GET_FSCALE_VALUE;

/* gcvHAL_QUERY_RESET_TIME_STAMP. */
typedef struct _gcsHAL_QUERY_RESET_TIME_STAMP
{
    OUT gctUINT64               timeStamp;
    OUT gctUINT64               contextID;
}
gcsHAL_QUERY_RESET_TIME_STAMP;

/* gcvHAL_CREATE_NATIVE_FENCE. */
typedef struct _gcsHAL_CREATE_NATIVE_FENCE
{
    /* Signal id. */
    IN gctUINT64                signal;

    /* Native fence file descriptor. */
    OUT gctINT32                fenceFD;

}
gcsHAL_CREATE_NATIVE_FENCE;

/* gcvHAL_WAIT_NATIVE_FENCE. */
typedef struct _gcsHAL_WAIT_NATIVE_FENCE
{
    /* Native fence file descriptor. */
    IN gctINT32                 fenceFD;

    /* Wait timeout. */
    IN gctUINT32                timeout;
}
gcsHAL_WAIT_NATIVE_FENCE;

/* gcvHAL_SHBUF. */
typedef struct _gcsHAL_SHBUF
{
    gceSHBUF_COMMAND_CODES      command;

    /* Shared buffer. */
    IN OUT gctUINT64            id;

    /* User data to be shared. */
    IN gctUINT64                data;

    /* Data size. */
    IN OUT gctUINT32            bytes;
}
gcsHAL_SHBUF;

/* gcvHAL_GET_GRAPHIC_BUFFER_FD. */
/*
 * Fd representation of android graphic buffer contents.
 * Currently, it is only to reference video nodes, signal, etc to avoid being
 * destroyed when trasfering across processes.
 */
typedef struct _gcsHAL_GET_GRAPHIC_BUFFER_FD
{
    /* Max 3 video nodes, node handle here. */
    IN gctUINT32                node[3];

    /* A shBuf. */
    IN gctUINT64                shBuf;

    /* A signal. */
    IN gctUINT64                signal;

    OUT gctINT32                fd;
}
gcsHAL_GET_GRAPHIC_BUFFER_FD;

/* gcvHAL_GET_VIDEO_MEMORY_FD. */
typedef struct _gcsHAL_GET_VIDEO_MEMORY_FD
{
    IN gctUINT32                handle;
    OUT gctINT32                fd;
}
gcsHAL_GET_VIDEO_MEMORY_FD;

/* gcvHAL_DESTROY_MMU. */
typedef struct _gcsHAL_DESTROY_MMU
{
    /* Mmu object. */
    IN gctUINT64                mmu;
}
gcsHAL_DESTROY_MMU;

/* gcvHAL_WAIT_FENCE. */
typedef struct _gcsHAL_WAIT_FENCE
{
    IN gctUINT32                handle;
    IN gctUINT32                timeOut;
}
gcsHAL_WAIT_FENCE;


#if gcdDEC_ENABLE_AHB
/* gcvHAL_DEC300_READ. */
typedef struct _gcsHAL_DEC300_READ
{
    gctUINT32                   enable;
    gctUINT32                   readId;
    gctUINT32                   format;
    gctUINT32                   strides[3];
    gctUINT32                   is3D;
    gctUINT32                   isMSAA;
    gctUINT32                   clearValue;
    gctUINT32                   isTPC;
    gctUINT32                   isTPCCompressed;
    gctUINT32                   surfAddrs[3];
    gctUINT32                   tileAddrs[3];
}
DEC300Read;

/* gcvHAL_DEC300_WRITE. */
typedef struct _gcsHAL_DEC300_WRITE
{
    gctUINT32                   enable;
    gctUINT32                   readId;
    gctUINT32                   writeId;
    gctUINT32                   format;
    gctUINT32                   surfAddr;
    gctUINT32                   tileAddr;
}
DEC300Write;

/* gcvHAL_DEC300_FLUSH. */
typedef struct _gcsHAL_DEC300_FLUSH
{
    IN gctUINT8                 useless;
}
DEC300Flush;

/* gcvHAL_DEC300_FLUSH_WAIT. */
typedef struct _gcsHAL_DEC300_FLUSH_WAIT
{
    IN gctUINT32                done;
}
DEC300FlushWait;
#endif


typedef struct _gcsHAL_INTERFACE
{
    /* Command code. */
    gceHAL_COMMAND_CODES        command;

    /* Hardware type. */
    gceHARDWARE_TYPE            hardwareType;

    /* Core index for current hardware type. */
    gctUINT32                   coreIndex;

    /* Status value. */
    gceSTATUS                   status;

    /* Engine */
    gceENGINE                   engine;

    /* Ignore information from TSL when doing IO control */
    gctBOOL                     ignoreTLS;

    /* Union of command structures. */
    union _u
    {
        gcsHAL_CHIP_INFO                    ChipInfo;
        gcsHAL_VERSION                      Version;
        gcsHAL_SET_TIMEOUT                  SetTimeOut;

        gcsHAL_QUERY_VIDEO_MEMORY           QueryVideoMemory;
        gcsHAL_QUERY_CHIP_IDENTITY          QueryChipIdentity;
        gcsHAL_QUERY_CHIP_OPTIONS           QueryChipOptions;
        gcsHAL_QUERY_CHIP_FREQUENCY         QueryChipFrequency;

        gcsHAL_ALLOCATE_NON_PAGED_MEMORY    AllocateNonPagedMemory;
        gcsHAL_FREE_NON_PAGED_MEMORY        FreeNonPagedMemory;

        gcsHAL_ALLOCATE_LINEAR_VIDEO_MEMORY AllocateLinearVideoMemory;
        gcsHAL_WRAP_USER_MEMORY             WrapUserMemory;
        gcsHAL_RELEASE_VIDEO_MEMORY         ReleaseVideoMemory;

        gcsHAL_LOCK_VIDEO_MEMORY            LockVideoMemory;
        gcsHAL_UNLOCK_VIDEO_MEMORY          UnlockVideoMemory;
        gcsHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY BottomHalfUnlockVideoMemory;

        gcsHAL_EXPORT_VIDEO_MEMORY          ExportVideoMemory;
        gcsHAL_NAME_VIDEO_MEMORY            NameVideoMemory;
        gcsHAL_IMPORT_VIDEO_MEMORY          ImportVideoMemory;

        gcsHAL_MAP_MEMORY                   MapMemory;
        gcsHAL_UNMAP_MEMORY                 UnmapMemory;

        gcsHAL_CACHE                        Cache;

        gcsHAL_ATTACH                       Attach;
        gcsHAL_DETACH                       Detach;

        gcsHAL_EVENT_COMMIT                 Event;
        gcsHAL_COMMIT                       Commit;
        gcsHAL_COMMIT_DONE                  CommitDone;

        gcsHAL_USER_SIGNAL                  UserSignal;
        gcsHAL_SIGNAL                       Signal;

        gcsHAL_WRITE_DATA                   WriteData;
        gcsHAL_READ_REGISTER                ReadRegisterData;
        gcsHAL_WRITE_REGISTER               WriteRegisterData;
        gcsHAL_READ_REGISTER_EX             ReadRegisterDataEx;
        gcsHAL_WRITE_REGISTER_EX            WriteRegisterDataEx;

#if VIVANTE_PROFILER
        gcsHAL_GET_PROFILE_SETTING          GetProfileSetting;
        gcsHAL_SET_PROFILE_SETTING          SetProfileSetting;
        gcsHAL_READ_PROFILER_REGISTER_SETTING SetProfilerRegisterClear;
        gcsHAL_READ_ALL_PROFILE_REGISTERS_PART1 RegisterProfileData_part1;
        gcsHAL_READ_ALL_PROFILE_REGISTERS_PART2 RegisterProfileData_part2;
        gcsHAL_PROFILE_REGISTERS_2D         RegisterProfileData2D;
#endif

        gcsHAL_SET_POWER_MANAGEMENT         SetPowerManagement;
        gcsHAL_QUERY_POWER_MANAGEMENT       QueryPowerManagement;
        gcsHAL_CONFIG_POWER_MANAGEMENT      ConfigPowerManagement;

        gcsHAL_GET_BASE_ADDRESS             GetBaseAddress;

        gcsHAL_SET_DEBUG_LEVEL_ZONE         DebugLevelZone;
        gcsHAL_DEBUG_DUMP                   DebugDump;

        gcsHAL_TIMESTAMP                    TimeStamp;
        gcsHAL_DATABASE                     Database;

        gcsHAL_GET_FRAME_INFO               GetFrameInfo;


        /* gcsHAL_DUMP_GPU_STATE */
        /* gcsHAL_DUMP_EVENT */

        gcsHAL_SET_FSCALE_VALUE             SetFscaleValue;
        gcsHAL_GET_FSCALE_VALUE             GetFscaleValue;

        gcsHAL_QUERY_RESET_TIME_STAMP       QueryResetTimeStamp;

        gcsHAL_CREATE_NATIVE_FENCE          CreateNativeFence;
        gcsHAL_WAIT_NATIVE_FENCE            WaitNativeFence;
        gcsHAL_SHBUF                        ShBuf;
        gcsHAL_GET_GRAPHIC_BUFFER_FD        GetGraphicBufferFd;
        gcsHAL_GET_VIDEO_MEMORY_FD          GetVideoMemoryFd;

        gcsHAL_DESTROY_MMU                  DestroyMmu;

        gcsHAL_WAIT_FENCE                   WaitFence;


#if gcdDEC_ENABLE_AHB
        gcsHAL_DEC300_READ                  DEC300Read;
        gcsHAL_DEC300_WRITE                 DEC300Write;
        gcsHAL_DEC300_FLUSH                 DEC300Flush;
        gcsHAL_DEC300_FLUSH_WAIT            DEC300FlushWait;
#endif
    }
    u;
}
gcsHAL_INTERFACE;


#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_driver_h_ */



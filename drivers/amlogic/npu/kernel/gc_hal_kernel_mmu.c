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


#include "gc_hal_kernel_precomp.h"

#define _GC_OBJ_ZONE    gcvZONE_MMU

#define gcdMMU_PGTABLE_POOL     gcvPOOL_VIRTUAL

typedef enum _gceMMU_TYPE
{
    gcvMMU_USED     = (0 << 4),
    gcvMMU_SINGLE   = (1 << 4),
    gcvMMU_FREE     = (2 << 4),
}
gceMMU_TYPE;

#define gcmENTRY_TYPE(x) (x & 0xF0)

#define gcmENTRY_COUNT(x) ((x & 0xFFFFFF00) >> 8)

#define gcdMMU_TABLE_DUMP       0

#define gcdVERTEX_START      (128 << 10)

typedef struct _gcsMMU_STLB_CHUNK *gcsMMU_STLB_CHUNK_PTR;

typedef struct _gcsMMU_STLB_CHUNK
{
    gckVIDMEM_NODE  videoMem;
    gctUINT32_PTR   logical;
    gctSIZE_T       size;
    gctPHYS_ADDR_T  physBase;
    gctSIZE_T       pageCount;
    gctUINT32       mtlbIndex;
    gctUINT32       mtlbEntryNum;
    gcsMMU_STLB_CHUNK_PTR next;
} gcsMMU_STLB_CHUNK;

#if gcdSHARED_PAGETABLE
typedef struct _gcsSharedPageTable * gcsSharedPageTable_PTR;
typedef struct _gcsSharedPageTable
{
    /* Shared gckMMU object. */
    gckMMU          mmu;

    /* Hardwares which use this shared pagetable. */
    gckHARDWARE     hardwares[gcdMAX_GPU_COUNT];

    /* Number of cores use this shared pagetable. */
    gctUINT32       reference;
}
gcsSharedPageTable;

static gcsSharedPageTable_PTR sharedPageTable = gcvNULL;
#endif

typedef struct _gcsFreeSpaceNode * gcsFreeSpaceNode_PTR;
typedef struct _gcsFreeSpaceNode
{
    gctUINT32       start;
    gctINT32        entries;
}
gcsFreeSpaceNode;

#if gcdENDIAN_BIG

#  define _WritePageEntry(pageEntry, entryValue) \
    *(gctUINT32_PTR)(pageEntry) = gcmBSWAP32((gctUINT32)(entryValue))

#  define _ReadPageEntry(pageEntry) \
    gcmBSWAP32(*(gctUINT32_PTR)(pageEntry))

#else

#  define _WritePageEntry(pageEntry, entryValue) \
    *(gctUINT32_PTR)(pageEntry) = (gctUINT32)(entryValue)

#  define _ReadPageEntry(pageEntry) \
    *(gctUINT32_PTR)(pageEntry)

#endif

static gceSTATUS
_FillPageTable(
    IN gctUINT32_PTR PageTable,
    IN gctUINT32     PageCount,
    IN gctUINT32     EntryValue
)
{
    gctUINT i;

    for (i = 0; i < PageCount; i++)
    {
        _WritePageEntry(PageTable + i, EntryValue);
    }

    return gcvSTATUS_OK;
}

static gceSTATUS
_FillMap(
    IN gctUINT32_PTR Map,
    IN gctUINT32     PageCount,
    IN gctUINT32     EntryValue
)
{
    gctUINT i;

    for (i = 0; i < PageCount; i++)
    {
        Map[i] = EntryValue;
    }

    return gcvSTATUS_OK;
}

static gceSTATUS
_Link(
    IN gcsADDRESS_AREA_PTR Area,
    IN gctUINT32 Index,
    IN gctUINT32 Next
    )
{
    if (Index >= Area->stlbEntries)
    {
        /* Just move heap pointer. */
        Area->heapList = Next;
    }
    else
    {
        /* Address page table. */
        gctUINT32_PTR map = Area->mapLogical;

        /* Dispatch on node type. */
        switch (gcmENTRY_TYPE(map[Index]))
        {
        case gcvMMU_SINGLE:
            /* Set single index. */
            map[Index] = (Next << 8) | gcvMMU_SINGLE;
            break;

        case gcvMMU_FREE:
            /* Set index. */
            map[Index + 1] = Next;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", Index);
            return gcvSTATUS_HEAP_CORRUPTED;
        }
    }

    /* Success. */
    return gcvSTATUS_OK;
}

static gceSTATUS
_AddFree(
    IN gcsADDRESS_AREA_PTR Area,
    IN gctUINT32 Index,
    IN gctUINT32 Node,
    IN gctUINT32 Count
    )
{
    gctUINT32_PTR map = Area->mapLogical;

    if (Count == 1)
    {
        /* Initialize a single page node. */
        map[Node] = (~((1U<<8)-1)) | gcvMMU_SINGLE;
    }
    else
    {
        /* Initialize the node. */
        map[Node + 0] = (Count << 8) | gcvMMU_FREE;
        map[Node + 1] = ~0U;
    }

    /* Append the node. */
    return _Link(Area, Index, Node);
}

static gceSTATUS
_Collect(
    IN gcsADDRESS_AREA_PTR Area
    )
{
    gctUINT32_PTR map = Area->mapLogical;
    gceSTATUS status;
    gctUINT32 i, previous, start = 0, count = 0;

    previous = Area->heapList = ~0U;
    Area->freeNodes = gcvFALSE;

    /* Walk the entire page table. */
    for (i = 0; i < Area->stlbEntries; ++i)
    {
        /* Dispatch based on type of page. */
        switch (gcmENTRY_TYPE(map[i]))
        {
        case gcvMMU_USED:
            /* Used page, so close any open node. */
            if (count > 0)
            {
                /* Add the node. */
                gcmkONERROR(_AddFree(Area, previous, start, count));

                /* Reset the node. */
                previous = start;
                count    = 0;
            }
            break;

        case gcvMMU_SINGLE:
            /* Single free node. */
            if (count++ == 0)
            {
                /* Start a new node. */
                start = i;
            }
            break;

        case gcvMMU_FREE:
            /* A free node. */
            if (count == 0)
            {
                /* Start a new node. */
                start = i;
            }

            /* Advance the count. */
            count += map[i] >> 8;

            /* Advance the index into the page table. */
            i     += (map[i] >> 8) - 1;
            break;

        default:
            gcmkFATAL("MMU page table correcupted at index %u!", i);
            return gcvSTATUS_HEAP_CORRUPTED;
        }
    }

    /* See if we have an open node left. */
    if (count > 0)
    {
        /* Add the node to the list. */
        gcmkONERROR(_AddFree(Area, previous, start, count));
    }

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_MMU,
                   "Performed a garbage collection of the MMU heap.");

    /* Success. */
    return gcvSTATUS_OK;

OnError:
    /* Return the staus. */
    return status;
}

static gctUINT32
_SetPage(gctUINT32 PageAddress, gctUINT32 PageAddressExt, gctBOOL Writable)
{
    gctUINT32 entry = PageAddress
                    /* AddressExt */
                    | (PageAddressExt << 4)
                    /* Ignore exception */
                    | (0 << 1)
                    /* Present */
                    | (1 << 0);

    if (Writable)
    {
        /* writable */
        entry |= (1 << 2);
    }
#if gcdUSE_MMU_EXCEPTION
    else
    {
        /* If this page is read only, set exception bit to make exception happens
        ** when writing to it. */
        entry |= gcdMMU_STLB_EXCEPTION;
    }
#endif

    return entry;
}

static gctUINT32
_MtlbOffset(
    gctUINT32 Address
    )
{
    return (Address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT;
}

gctUINT32
_AddressToIndex(
    IN gcsADDRESS_AREA_PTR Area,
    IN gctUINT32 Address
    )
{
    gctUINT32 mtlbOffset = (Address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT;
    gctUINT32 stlbOffset = (Address & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT;

    return (mtlbOffset - Area->mappingStart) * gcdMMU_STLB_4K_ENTRY_NUM + stlbOffset;
}

static gctUINT32_PTR
_StlbEntry(
    gcsADDRESS_AREA_PTR Area,
    gctUINT32 Address
    )
{
    gctUINT32 index = _AddressToIndex(Area, Address);

    return &Area->stlbLogical[index];
}

static gctBOOL
_IsRangeInsected(
    gctUINT64 baseAddress1,
    gctSIZE_T size1,
    gctUINT64 baseAddress2,
    gctSIZE_T size2
    )
{
    gctUINT64 endAddress1 = baseAddress1 + size1 - 1;
    gctUINT64 endAddress2 = baseAddress2 + size2 - 1;

    if (!size1 || !size2)
    {
        return gcvFALSE;
    }

    return (((baseAddress2 <= endAddress1) && (endAddress2 >= baseAddress1)) ||
            ((baseAddress1 <= endAddress2) && (endAddress1 >= baseAddress2)));
}

static gceSTATUS
_FillFlatMappingInMap(
    gcsADDRESS_AREA_PTR Area,
    gctUINT32 Index,
    gctUINT32 NumPages
    )
{
    gceSTATUS status;
    gctUINT32 i;
    gctBOOL gotIt = gcvFALSE;
    gctUINT32 index = Index;
    gctUINT32_PTR map = Area->mapLogical;
    gctUINT32 previous = ~0U;

    /* Find node which contains index. */
    for (i = 0; !gotIt && (i < Area->stlbEntries);)
    {
        gctUINT32 numPages;

        switch (gcmENTRY_TYPE(map[i]))
        {
        case gcvMMU_SINGLE:
            if (i == index)
            {
                gotIt = gcvTRUE;
            }
            else
            {
                previous = i;
                i = map[i] >> 8;
            }
            break;

        case gcvMMU_FREE:
            numPages = map[i] >> 8;
            if (index >= i && index + NumPages - 1 < i + numPages)
            {
                gotIt = gcvTRUE;
            }
            else
            {
                previous = i;
                i = map[i + 1];
            }
            break;

        case gcvMMU_USED:
            i++;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", index);
            gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
        }
    }

    switch (gcmENTRY_TYPE(map[i]))
    {
    case gcvMMU_SINGLE:
        /* Unlink single node from free list. */
        gcmkONERROR(
            _Link(Area, previous, map[i] >> 8));
        break;

    case gcvMMU_FREE:
        /* Split the node. */
        {
            gctUINT32 start;
            gctUINT32 next = map[i+1];
            gctUINT32 total = map[i] >> 8;
            gctUINT32 countLeft = index - i;
            gctUINT32 countRight = total - countLeft - NumPages;

            if (countLeft)
            {
                start = i;
                _AddFree(Area, previous, start, countLeft);
                previous = start;
            }

            if (countRight)
            {
                start = index + NumPages;
                _AddFree(Area, previous, start, countRight);
                previous = start;
            }

            _Link(Area, previous, next);
        }
        break;
    }

    _FillMap(&map[index], NumPages, gcvMMU_USED);

    return gcvSTATUS_OK;
OnError:
    return status;
}

static gceSTATUS
_CollectFreeSpace(
    IN gckMMU Mmu,
    OUT gcsFreeSpaceNode_PTR *Array,
    OUT gctINT * Size
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctPOINTER pointer = gcvNULL;
    gcsFreeSpaceNode_PTR array = gcvNULL;
    gcsFreeSpaceNode_PTR node = gcvNULL;
    gctINT size = 0;
    gctINT i = 0;

    for (i = 0; i < gcdMMU_MTLB_ENTRY_NUM; i++)
    {
        if (!Mmu->mtlbLogical[i])
        {
            if (!node)
            {
                /* This is the first entry of the free space. */
                node += 1;
                size++;

            }
        }
        else if (node)
        {
            /* Reset the start. */
            node = gcvNULL;
        }
    }

    /* Allocate memory for the array. */
    gcmkONERROR(gckOS_Allocate(Mmu->os,
                               gcmSIZEOF(*array) * size,
                               &pointer));

    array = (gcsFreeSpaceNode_PTR)pointer;
    node  = gcvNULL;

    for (i = 0, size = 0; i < gcdMMU_MTLB_ENTRY_NUM; i++)
    {
        if (!Mmu->mtlbLogical[i])
        {
            if (!node)
            {
                /* This is the first entry of the free space. */
                node = &array[size++];

                node->start   = i;
                node->entries = 0;
            }

            node->entries++;
        }
        else if (node)
        {
            /* Reset the start. */
            node = gcvNULL;
        }
    }

#if gcdMMU_TABLE_DUMP
    for (i = 0; i < size; i++)
    {
        gckOS_Print("%s(%d): [%d]: start=%d, entries=%d.\n",
                __FUNCTION__, __LINE__,
                i,
                array[i].start,
                array[i].entries);
    }
#endif

    *Array = array;
    *Size  = size;

    return gcvSTATUS_OK;

OnError:
    if (pointer != gcvNULL)
    {
        gckOS_Free(Mmu->os, pointer);
    }

    return status;
}

gceSTATUS
_GetMtlbFreeSpace(
    IN gckMMU Mmu,
    IN gctUINT32 NumEntries,
    OUT gctUINT32 *MtlbStart,
    OUT gctUINT32 *MtlbEnd
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gcsFreeSpaceNode_PTR nodeArray = gcvNULL;
    gctINT i, nodeArraySize = 0;
    gctINT numEntries = gcdMMU_MTLB_ENTRY_NUM;
    gctINT32 mStart = -1;
    gctINT32 mEnd = -1;

    gcmkONERROR(_CollectFreeSpace(Mmu, &nodeArray, &nodeArraySize));

    /* Find the smallest space for NumEntries */
    for (i = 0; i < nodeArraySize; i++)
    {
        if (nodeArray[i].entries < numEntries && NumEntries <= (gctUINT32)nodeArray[i].entries)
        {
            numEntries = nodeArray[i].entries;

            mStart = nodeArray[i].start;
            mEnd   = nodeArray[i].start + NumEntries - 1;
        }
    }

    if (mStart == -1 && mEnd == -1)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    *MtlbStart = (gctUINT32)mStart;
    *MtlbEnd   = (gctUINT32)mEnd;

OnError:
    if (nodeArray)
    {
        gckOS_Free(Mmu->os, (gctPOINTER)nodeArray);
    }

    return status;
}

static gceSTATUS
_FillFlatMapping(
    IN gckMMU Mmu,
    IN gctUINT64 PhysBase,
    IN gctSIZE_T Size,
    IN gctBOOL   reserved,
    IN gctBOOL   ableToShift,
    OUT gctUINT32 *GpuBaseAddress
    )
{
    gceSTATUS status;
    gctUINT32 mtlb;
    gctUINT32 physBase;
    gckKERNEL kernel = Mmu->hardware->kernel;
    gcsADDRESS_AREA_PTR area = &Mmu->dynamicArea;
    gctBOOL physicalRangeOverlapped = gcvFALSE;
    gctBOOL virtualRangeOverlapped = gcvFALSE;
    gctBOOL specificFlatMapping = gcvFALSE;
    gctBOOL needShiftMapping = gcvFALSE;
    gctUINT64 flatBase = PhysBase;
    gctUINT32 flatSize = (gctUINT32) Size;
    gctUINT64 base = flatBase;
    gctUINT64 end  = base + flatSize;
    gctUINT32 reqVirtualBase = 0;
    gctUINT32 flatVirtualBase = 0;
    gctUINT32 i;

    /************************ Get flat mapping type and range. ************************/
    {
        for (i = 0; i < Mmu->gpuPhysicalRangeCount; i++)
        {
            if (base < Mmu->gpuPhysicalRanges[i].start)
            {
                if (end > Mmu->gpuPhysicalRanges[i].start)
                {
                    physicalRangeOverlapped = gcvTRUE;
                    if (Mmu->gpuPhysicalRanges[i].flag == gcvFLATMAP_DIRECT)
                    {
                        /* Overlapped part is direct mapping, continue direct mapping */
                        end = Mmu->gpuPhysicalRanges[i].start;
                    }
                    else
                    {
                        /* Overlapped part is shift mapping, do entire shift mapping */
                        needShiftMapping = gcvTRUE;
                    }
                }

                flatSize = (gctUINT32) (end - base);
            }
            else if (end > Mmu->gpuPhysicalRanges[i].end)
            {
                if (base < Mmu->gpuPhysicalRanges[i].end)
                {
                    physicalRangeOverlapped = gcvTRUE;
                    if (Mmu->gpuPhysicalRanges[i].flag == gcvFLATMAP_DIRECT)
                    {
                        /* Overlapped part is direct mapping, continue direct mapping */
                        base = Mmu->gpuPhysicalRanges[i].end + 1;
                    }
                    else
                    {
                        /* Overlapped part is shift mapping, do entire shift mapping */
                        needShiftMapping = gcvTRUE;
                    }

                }

                flatBase = base;
                flatSize = (gctUINT32) (end - base);
            }
            else
            {
                /* it is already inside existing flat mapping ranges. */
                flatSize = 0;
            }

            if (flatSize == 0)
            {
                if (GpuBaseAddress)
                {
                    *GpuBaseAddress = (gctUINT32) PhysBase;
                }

                return gcvSTATUS_OK;
            }
        }
    }

    /* overwrite the orignal parameters */
    PhysBase = flatBase;
    physBase = (gctUINT32)flatBase;

    mtlb = _MtlbOffset(physBase);

    if (GpuBaseAddress)
    {
        reqVirtualBase = *GpuBaseAddress;
    }

    /*
     * if no partcial physical range overlap to request entire shift mapping,
     * it is specific shift mapping or directly mapping by default.
     */
    if (!needShiftMapping)
    {
        flatVirtualBase = reqVirtualBase ? reqVirtualBase : (gctUINT32) flatBase;
    }

    for (i = 0; i < Mmu->gpuAddressRangeCount; i++)
    {
        if (_IsRangeInsected(flatVirtualBase, flatSize,
            Mmu->gpuAddressRanges[i].start,  Mmu->gpuAddressRanges[i].size))
        {
            virtualRangeOverlapped = gcvTRUE;
        }
    }

    /* If gpu virtual range overlapped or gpu physical over 4G, still need entire shift mapping */
    if ((!physicalRangeOverlapped && virtualRangeOverlapped) ||
        PhysBase + flatSize - 1 > 0xffffffff)
    {
        needShiftMapping = gcvTRUE;
    }

    if (needShiftMapping && !ableToShift)
    {
        /*
         * Return without mapping any address.
         * By now, only physBase physSize could run here.
         */
        return gcvSTATUS_OK;
    }

    specificFlatMapping = (reqVirtualBase && !virtualRangeOverlapped && !physicalRangeOverlapped);

    /************************ Setup flat mapping in dynamic range. ****************/

    if (area->mappingStart != gcvINVALID_ADDRESS && mtlb >= area->mappingStart && mtlb < area->mappingEnd)
    {
        gctUINT32_PTR stlbEntry;

        stlbEntry = _StlbEntry(area, physBase);

        /* Must be aligned to page. */
        gcmkASSERT((flatSize & 0xFFF) == 0);

        for (i = 0; i < (flatSize / gcdMMU_PAGE_4K_SIZE); i++)
        {
            /* Flat mapping in page table. */
            _WritePageEntry(stlbEntry, _SetPage(physBase + i * gcdMMU_PAGE_4K_SIZE, 0, gcvTRUE));
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert MTLB[%d] STLB[%d]: %08x\n",
                __FUNCTION__, __LINE__,
                (physBase & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT,
                ((physBase & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT) + i,
                _ReadPageEntry(stlbEntry));
#endif
            stlbEntry++;
        }

#if gcdDUMP_IN_KERNEL
        {
            gctPHYS_ADDR_T physical;
            gctUINT32 data = _SetPage(physBase, 0, gcvTRUE) & ~0xF;
            gctUINT32 step = (_SetPage(physBase + gcdMMU_PAGE_4K_SIZE, 0, gcvTRUE) & ~0xF) - data;
            gctUINT32 mask = _SetPage(physBase, 0, gcvTRUE) & 0xF;

            physical  = area->stlbPhysical + 4 * _AddressToIndex(area, physBase);

            gcmkDUMP(Mmu->os,
                     "#[mmu-stlb: flat-mapping in dynamic: 0x%08X - 0x%08X]",
                     physBase, physBase - 1 + flatSize);

            gcmkDUMP(Mmu->os,
                     "@[physical.step 0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X",
                     physical, data, flatSize / gcdMMU_PAGE_4K_SIZE * sizeof(gctUINT32), step, mask);
        }
#endif

        /* Flat mapping in map. */
        _FillFlatMappingInMap(area, _AddressToIndex(area, physBase), flatSize / gcdMMU_PAGE_4K_SIZE);

        return gcvSTATUS_OK;
    }

    /************************ Setup flat mapping in non dynamic range. **************/
    {
        gctBOOL mutex = gcvFALSE;
        gctUINT32 physBaseExt = (gctUINT32) (PhysBase >> 32);
        gctUINT32 start = physBase & ~gcdMMU_PAGE_1M_MASK;
        gctUINT32 end = (gctUINT32) (physBase + flatSize - 1) & ~gcdMMU_PAGE_1M_MASK;
        gctUINT32 mStart = start >> gcdMMU_MTLB_SHIFT;
        gctUINT32 mEnd = end >> gcdMMU_MTLB_SHIFT;
        gctUINT32 sStart = (start & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT;
        gctUINT32 sEnd = (end & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT;
        gctPHYS_ADDR_T physical;
        gcsMMU_STLB_CHUNK_PTR newStlbChunk = gcvNULL;
        gctUINT32 stlbIndex = 0;
        gctUINT32 totalNewStlbs = 0;
        gctINT32 firstMtlbEntry = -1;
        gctUINT32 mtlbCurEntry;
        gcsMMU_STLB_CHUNK_PTR curStlbChunk = gcvNULL;
        gceFLATMAP_FLAG mapFlag = gcvFLATMAP_DIRECT;
        enum
        {
            COLOR_NONE   = 0,
            COLOR_RED    = 1, /* occupied entry */
            COLOR_BLUE   = 2, /* empty entry */
            COLOR_MAX    = COLOR_BLUE,
        } lastColor = COLOR_NONE;
        gctUINT32 colorNumber = 0;

        /* Grab the mutex. */
        gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
        mutex = gcvTRUE;

        if (needShiftMapping)
        {
            gctUINT32 mEntries;
            gctUINT32 sEntries;

            mEntries = (flatSize + (1 << gcdMMU_MTLB_SHIFT) - 1) / (1 << gcdMMU_MTLB_SHIFT);

            gcmkONERROR(_GetMtlbFreeSpace(Mmu, mEntries, &mStart, &mEnd));

            sStart = 0;
            sEntries = (flatSize + gcdMMU_PAGE_1M_SIZE - 1) / gcdMMU_PAGE_1M_SIZE;
            sEnd = (sEntries - 1) % gcdMMU_STLB_1M_ENTRY_NUM;
            mapFlag = gcvFLATMAP_SHIFT;
        }

        if (specificFlatMapping)
        {
            start    = reqVirtualBase & ~gcdMMU_PAGE_1M_MASK;
            end      = (reqVirtualBase + flatSize - 1) & ~gcdMMU_PAGE_1M_MASK;
            mStart   = start >> gcdMMU_MTLB_SHIFT;
            mEnd     = end >> gcdMMU_MTLB_SHIFT;
            sStart   = (start & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT;
            sEnd     = (end & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT;
            mapFlag  = gcvFLATMAP_SHIFT;
        }

        /* No matter direct mapping or shift mapping or specific mapping, store gpu virtual ranges */
        flatVirtualBase = (mStart << gcdMMU_MTLB_SHIFT)
                        | (sStart << gcdMMU_STLB_1M_SHIFT)
                        | (physBase & gcdMMU_PAGE_1M_MASK);

        /* Return GPU virtual base address if necessary */
        if (GpuBaseAddress)
        {
            *GpuBaseAddress = flatVirtualBase;
        }

        mtlbCurEntry = mStart;

        /* find all new stlbs, part of new flat mapping range may already have stlbs*/
        while (mtlbCurEntry <= mEnd)
        {
            if (*(Mmu->mtlbLogical + mtlbCurEntry) == 0)
            {
                if (lastColor != COLOR_BLUE)
                {
                    if (colorNumber < COLOR_MAX)
                    {
                        lastColor = COLOR_BLUE;
                        colorNumber++;
                    }
                    else
                    {
                        gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                    }
                }

                totalNewStlbs++;
                if (-1 == firstMtlbEntry)
                {
                    firstMtlbEntry = mtlbCurEntry;
                }
            }
            else
            {
                if (lastColor != COLOR_RED)
                {
                    if (colorNumber < COLOR_MAX)
                    {
                        lastColor = COLOR_RED;
                        colorNumber++;
                    }
                    else
                    {
                        gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                    }
                }
            }
            mtlbCurEntry++;
        }

        /* Need allocate a new chunk of stlbs */
        if (totalNewStlbs)
        {
            gcePOOL pool = gcdMMU_PGTABLE_POOL;
            gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;

            gcmkONERROR(
                gckOS_Allocate(Mmu->os,
                               sizeof(struct _gcsMMU_STLB_CHUNK),
                               (gctPOINTER *)&newStlbChunk));

            newStlbChunk->mtlbEntryNum = totalNewStlbs;
            newStlbChunk->next = gcvNULL;
            newStlbChunk->videoMem = gcvNULL;
            newStlbChunk->logical = gcvNULL;
            newStlbChunk->size = gcdMMU_STLB_1M_SIZE * newStlbChunk->mtlbEntryNum;
            newStlbChunk->pageCount = 0;
            newStlbChunk->mtlbIndex = firstMtlbEntry;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
            allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

            gcmkONERROR(gckKERNEL_AllocateVideoMemory(
                kernel,
                gcdMMU_STLB_1M_SIZE,
                gcvVIDMEM_TYPE_COMMAND,
                allocFlag,
                &newStlbChunk->size,
                &pool,
                &newStlbChunk->videoMem));

            /* Lock for kernel side CPU access. */
            gcmkONERROR(gckVIDMEM_NODE_LockCPU(
                kernel,
                newStlbChunk->videoMem,
                gcvFALSE,
                gcvFALSE,
                (gctPOINTER *)&newStlbChunk->logical));

            gcmkONERROR(gckOS_ZeroMemory(newStlbChunk->logical, newStlbChunk->size));

            /* Get CPU physical address. */
            gcmkONERROR(gckVIDMEM_NODE_GetPhysical(
                kernel,
                newStlbChunk->videoMem,
                0,
                &physical));

            gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(
                Mmu->os,
                physical,
                &physical));

            newStlbChunk->physBase = physical;
        }

        while (mStart <= mEnd)
        {
            gctUINT32 last = (mStart == mEnd) ? sEnd : (gcdMMU_STLB_1M_ENTRY_NUM - 1);
            gctPHYS_ADDR_T stlbPhyBase;
            gctUINT32_PTR stlbLogical;

            gcmkASSERT(mStart < gcdMMU_MTLB_ENTRY_NUM);

            if (*(Mmu->mtlbLogical + mStart) == 0)
            {
                gctUINT32 mtlbEntry;
                curStlbChunk = newStlbChunk;
                stlbPhyBase = curStlbChunk->physBase + (stlbIndex * gcdMMU_STLB_1M_SIZE);
                stlbLogical = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical + (stlbIndex * gcdMMU_STLB_1M_SIZE));

                physical  = stlbPhyBase
                          /* 1MB page size */
                          | (1 << 3)
                          /* Ignore exception */
                          | (0 << 1)
                          /* Present */
                          | (1 << 0);

                gcmkSAFECASTPHYSADDRT(mtlbEntry, physical);

                _WritePageEntry(Mmu->mtlbLogical + mStart, mtlbEntry);

#if gcdMMU_TABLE_DUMP
                gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                    __FUNCTION__, __LINE__,
                    mStart,
                    _ReadPageEntry(Mmu->mtlbLogical + mStart));

                gckOS_Print("%s(%d): STLB: logical:%08x -> physical:%08x\n",
                        __FUNCTION__, __LINE__,
                        stlbLogical,
                        stlbPhyBase);
#endif

                gcmkDUMP(Mmu->os, "#[mmu-mtlb: flat-mapping, slot: %d]", mStart);

                gcmkDUMP(Mmu->os, "@[physical.fill 0x%010X 0x%08X 0x%08X]",
                         Mmu->mtlbPhysical + mStart * 4, Mmu->mtlbLogical[mStart], 4);

                ++stlbIndex;
            }
            else
            {
                gctUINT32 mtlbEntry = _ReadPageEntry(Mmu->mtlbLogical + mStart);
                gctUINT stlbOffset;

                curStlbChunk = (gcsMMU_STLB_CHUNK_PTR)Mmu->staticSTLB;

                while (curStlbChunk)
                {
                    if ((mStart >= curStlbChunk->mtlbIndex) &&
                        (mStart < (curStlbChunk->mtlbIndex + curStlbChunk->mtlbEntryNum)))
                    {
                        break;
                    }
                    curStlbChunk = curStlbChunk->next;
                }
                gcmkASSERT(curStlbChunk);

                if (!curStlbChunk)
                    gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

                stlbOffset = mStart - curStlbChunk->mtlbIndex;

                stlbPhyBase = curStlbChunk->physBase + (stlbOffset * gcdMMU_STLB_1M_SIZE);
                stlbLogical = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical + (stlbOffset * gcdMMU_STLB_1M_SIZE));
                if (stlbPhyBase != (mtlbEntry & gcdMMU_MTLB_ENTRY_STLB_MASK))
                {
                    gcmkASSERT(0);
                }
            }

#if gcdDUMP_IN_KERNEL
            /* Start index. */
            i = sStart;

            gcmkDUMP(Mmu->os, "#[mmu-stlb: flat-mapping: 0x%08X - 0x%08X]",
                     start, start + (last - sStart) * gcdMMU_PAGE_1M_SIZE - 1);
#endif

            while (sStart <= last)
            {
                gcmkASSERT(!(start & gcdMMU_PAGE_1M_MASK));
                if (reserved)
                {
                    /* program NOT_PRESENT | EXCEPTION  for reserved entries */
                    _WritePageEntry(stlbLogical + sStart, 1 << 1);
                }
                else
                {
                    _WritePageEntry(stlbLogical + sStart, _SetPage(start, physBaseExt, gcvTRUE));
                }
#if gcdMMU_TABLE_DUMP
                gckOS_Print("%s(%d): insert STLB[%d]: %08x\n",
                    __FUNCTION__, __LINE__,
                    sStart,
                    _ReadPageEntry(stlbLogical + sStart));
#endif
                /* next page. */
                start += gcdMMU_PAGE_1M_SIZE;
                if (start == 0)
                {
                    physBaseExt++;
                }
                sStart++;
                curStlbChunk->pageCount++;
            }

#if gcdDUMP_IN_KERNEL
            {
                gctUINT32 data = stlbLogical[i] & ~0xF;
                gctUINT32 step = (last > i) ? (stlbLogical[i + 1] - stlbLogical[i]) : 0;
                gctUINT32 mask = stlbLogical[i] & 0xF;

                gcmkDUMP(Mmu->os,
                         "@[physical.step 0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X]",
                         stlbPhyBase + i * 4, data, (last - i) * 4, step, mask);
            }
#endif

            gcmkONERROR(gckVIDMEM_NODE_CleanCache(
                kernel,
                curStlbChunk->videoMem,
                0,
                curStlbChunk->logical,
                curStlbChunk->size
                ));

            sStart = 0;
            ++mStart;
        }

        gcmkASSERT(totalNewStlbs == stlbIndex);

        if (newStlbChunk)
        {
            /* Insert the stlbChunk into staticSTLB. */
            if (Mmu->staticSTLB == gcvNULL)
            {
                Mmu->staticSTLB = newStlbChunk;
            }
            else
            {
                gcmkASSERT(newStlbChunk != gcvNULL);
                gcmkASSERT(newStlbChunk->next == gcvNULL);
                newStlbChunk->next = Mmu->staticSTLB;
                Mmu->staticSTLB = newStlbChunk;
            }
        }

        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

#if gcdENABLE_TRUST_APPLICATION
        if (Mmu->hardware->options.secureMode == gcvSECURE_IN_TA)
        {
            gckKERNEL_SecurityMapMemory(Mmu->hardware->kernel, gcvNULL, physBase, flatSize / gcdMMU_PAGE_4K_SIZE, &physBase);
        }
#endif

        /* Store the gpu physical ranges */
        Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].start = flatBase;
        Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].end   = flatBase + flatSize - 1;
        Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].size  = flatSize;
        Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].flag  = mapFlag;
        Mmu->gpuPhysicalRangeCount++;

        gcmkASSERT(Mmu->gpuPhysicalRangeCount <= gcdMAX_FLAT_MAPPING_COUNT);

        /* Store the gpu virtual ranges */
        Mmu->gpuAddressRanges[Mmu->gpuAddressRangeCount].start = flatVirtualBase;
        Mmu->gpuAddressRanges[Mmu->gpuAddressRangeCount].end   = flatVirtualBase + flatSize - 1;
        Mmu->gpuAddressRanges[Mmu->gpuAddressRangeCount].size  = flatSize;
        Mmu->gpuAddressRanges[Mmu->gpuAddressRangeCount].flag  = mapFlag;
        Mmu->gpuAddressRangeCount++;

        gcmkASSERT(Mmu->gpuAddressRangeCount <= gcdMAX_FLAT_MAPPING_COUNT);


        return gcvSTATUS_OK;
OnError:
        /* Roll back the allocation.
        ** We don't need roll back mtlb programming as gckmONERROR
        ** is only used during allocation time.
        */
        if (newStlbChunk)
        {
            if (newStlbChunk->videoMem)
            {
                gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
                    kernel,
                    newStlbChunk->videoMem
                    ));
            }

            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, newStlbChunk));
        }
        if (mutex)
        {
            /* Release the mutex. */
            gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
        }
        return status;
    }
}

static gceSTATUS
_SetupAddressArea(
    IN gckOS Os,
    IN gcsADDRESS_AREA_PTR Area,
    IN gctUINT32 NumMTLBEntries
    )
{
    gceSTATUS status;
    gctUINT32_PTR map;

    gcmkHEADER();
    Area->stlbSize = NumMTLBEntries * gcdMMU_STLB_4K_SIZE;

    gcmkSAFECASTSIZET(Area->stlbEntries, Area->stlbSize / gcmSIZEOF(gctUINT32));

    gcmkONERROR(gckOS_Allocate(Os, Area->stlbSize, (void **)&Area->mapLogical));

    /* Initialization. */
    map      = Area->mapLogical;
    map[0]   = (Area->stlbEntries << 8) | gcvMMU_FREE;
    map[1]   = ~0U;
    Area->heapList  = 0;
    Area->freeNodes = gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_SetupDynamicSpace(
    IN gckMMU Mmu
    )
{
    gceSTATUS status;
    gcsFreeSpaceNode_PTR nodeArray = gcvNULL;
    gctINT i, nodeArraySize = 0;
    gctUINT32 address;
    gctINT numEntries = 0;
    gctBOOL acquired = gcvFALSE;
    gctUINT32 mtlbEntry;
    gckKERNEL kernel = Mmu->hardware->kernel;
    gcsADDRESS_AREA_PTR area = &Mmu->dynamicArea;
    gcePOOL pool = gcdMMU_PGTABLE_POOL;
    gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;

    /* Find all the free address space. */
    gcmkONERROR(_CollectFreeSpace(Mmu, &nodeArray, &nodeArraySize));

    for (i = 0; i < nodeArraySize; i++)
    {
        if (nodeArray[i].entries > numEntries)
        {
            area->mappingStart = nodeArray[i].start;
            numEntries         = nodeArray[i].entries;
            area->mappingEnd   = area->mappingStart + numEntries;
        }
    }

    gckOS_Free(Mmu->os, (gctPOINTER)nodeArray);

#if gcdENABLE_TRUST_APPLICATION
    if (gckHARDWARE_IsFeatureAvailable(Mmu->hardware, gcvFEATURE_SECURITY) == gcvSTATUS_TRUE)
    {
        /* Setup secure address area when needed. */
        gctUINT32 secureAreaSize = gcdMMU_SECURE_AREA_SIZE;
        gcsADDRESS_AREA_PTR secureArea = &Mmu->secureArea;

        gcmkASSERT(numEntries > (gctINT)secureAreaSize);

        secureArea->mappingStart = area->mappingStart
                                 + (numEntries - secureAreaSize);

        gcmkONERROR(_SetupAddressArea(Mmu->os, secureArea, secureAreaSize));

        numEntries -= secureAreaSize;
    }
#endif

    /* Setup normal address area. */
    gcmkONERROR(_SetupAddressArea(Mmu->os, area, numEntries));

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
    allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

    /* Construct Slave TLB. */
    gcmkONERROR(gckKERNEL_AllocateVideoMemory(
                kernel,
                64,
                gcvVIDMEM_TYPE_COMMAND,
                allocFlag,
                &area->stlbSize,
                &pool,
                &area->stlbVideoMem));

    /* Lock for kernel side CPU access. */
    gcmkONERROR(gckVIDMEM_NODE_LockCPU(
                kernel,
                area->stlbVideoMem,
                gcvFALSE,
                gcvFALSE,
                (gctPOINTER *)&area->stlbLogical));

#if gcdUSE_MMU_EXCEPTION
    gcmkONERROR(_FillPageTable(area->stlbLogical,
                               area->stlbEntries,
                               /* Enable exception */
                               1 << 1));
#else
    /* Invalidate all entries. */
    gcmkONERROR(gckOS_ZeroMemory(area->stlbLogical,
                area->stlbSize));
#endif

    /* Get stlb table physical. */
    gcmkONERROR(gckVIDMEM_NODE_GetPhysical(
                kernel,
                area->stlbVideoMem,
                0,
                &area->stlbPhysical));

    gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Mmu->os,
                  area->stlbPhysical,
                  &area->stlbPhysical));


    gcmkDUMP(Mmu->os, "#[mmu: dynamic space: 0x%08X - 0x%08X]",
             (area->mappingStart << gcdMMU_MTLB_SHIFT),
             (area->mappingEnd << gcdMMU_MTLB_SHIFT) - 1);

    gcmkDUMP(Mmu->os, "#[mmu-stlb]");

    gcmkDUMP(Mmu->os, "@[physical.fill 0x%010X 0x%08X 0x%08X]",
             area->stlbPhysical, area->stlbLogical[0], area->stlbSize / 4);

    gcmkSAFECASTPHYSADDRT(address, area->stlbPhysical);

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    acquired = gcvTRUE;

    /* Map to Master TLB. */
    for (i = (gctINT)area->mappingStart;
         i < (gctINT)area->mappingStart + numEntries;
         i++)
    {
        mtlbEntry = address
                  /* 4KB page size */
                  | (0 << 2)
                  /* Ignore exception */
                  | (0 << 1)
                  /* Present */
                  | (1 << 0);

        _WritePageEntry(Mmu->mtlbLogical + i, mtlbEntry);

#if gcdMMU_TABLE_DUMP
        gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                __FUNCTION__, __LINE__,
                i,
                _ReadPageEntry(Mmu->mtlbLogical + i));
#endif
        address += gcdMMU_STLB_4K_SIZE;
    }

    gcmkDUMP(Mmu->os, "#[mmu-mtlb: slot: %d - %d]",
             area->mappingStart, area->mappingEnd - 1);

#if gcdDUMP_IN_KERNEL
    {
        gctUINT32 data = Mmu->mtlbLogical[area->mappingStart] & ~0x3F;
        gctUINT32 step = 0;
        gctUINT32 mask = Mmu->mtlbLogical[area->mappingStart] & 0x3F;

        if (numEntries > 1)
        {
            step = Mmu->mtlbLogical[area->mappingStart + 1]
                 - Mmu->mtlbLogical[area->mappingStart];
        }

        gcmkDUMP(Mmu->os,
                 "@[physical.step 0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X]",
                 (unsigned long long)(Mmu->mtlbPhysical + area->mappingStart * 4),
                 data, numEntries * 4, step, mask);
    }
#endif

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    return gcvSTATUS_OK;

OnError:
    if (area->mapLogical)
    {
        gcmkVERIFY_OK(
            gckOS_Free(Mmu->os, (gctPOINTER) area->mapLogical));

        gcmkVERIFY_OK(
            gckVIDMEM_NODE_Dereference(kernel,
                                       area->stlbVideoMem));
    }

    if (acquired)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }

    return status;
}

gctUINT32
_GetPageCountOfUsedNode(
    gctUINT32_PTR Node
    )
{
    gctUINT32 count;

    count = gcmENTRY_COUNT(*Node);

    if ((count << 8) == (~((1U<<8)-1)))
    {
        count = 1;
    }

    return count;
}

static gcsADDRESS_AREA_PTR
_GetProcessArea(
    IN gckMMU Mmu,
    IN gctBOOL Secure
    )
{
#if gcdENABLE_TRUST_APPLICATION
    if (Secure == gcvTRUE)
    {
        return &Mmu->secureArea;
    }
#endif

    return &Mmu->dynamicArea;
}

/*******************************************************************************
**
**  _Construct
**
**  Construct a new gckMMU object.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSIZE_T MmuSize
**          Number of bytes for the page table.
**
**  OUTPUT:
**
**      gckMMU * Mmu
**          Pointer to a variable that receives the gckMMU object pointer.
*/
gceSTATUS
_Construct(
    IN gckKERNEL Kernel,
    IN gctSIZE_T MmuSize,
    OUT gckMMU * Mmu
    )
{
    gckOS os;
    gckHARDWARE hardware;
    gceSTATUS status;
    gckMMU mmu = gcvNULL;
    gctUINT32_PTR map;
    gctPOINTER pointer = gcvNULL;
    gctPHYS_ADDR_T physBase;
    gctSIZE_T physSize;
    gctPHYS_ADDR_T contiguousBase;
    gctSIZE_T contiguousSize = 0;
    gctPHYS_ADDR_T externalBase;
    gctSIZE_T externalSize = 0;
    gctUINT32 gpuAddress;
    gctPHYS_ADDR_T gpuPhysical;
    gcsADDRESS_AREA_PTR area = gcvNULL;
    gcePOOL pool;
    gctUINT64 data;
    gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;
    gctUINT64 mmuEnabled;

    gcmkHEADER_ARG("Kernel=0x%x MmuSize=%lu", Kernel, MmuSize);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(MmuSize > 0);
    gcmkVERIFY_ARGUMENT(Mmu != gcvNULL);

    /* Extract the gckOS object pointer. */
    os = Kernel->os;
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    /* Extract the gckHARDWARE object pointer. */
    hardware = Kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    /* Allocate memory for the gckMMU object. */
    gcmkONERROR(gckOS_Allocate(os, sizeof(struct _gckMMU), &pointer));

    gckOS_ZeroMemory(pointer, sizeof(struct _gckMMU));

    mmu = pointer;

    /* Initialize the gckMMU object. */
    mmu->object.type      = gcvOBJ_MMU;
    mmu->os               = os;
    mmu->hardware         = hardware;
    mmu->pageTableMutex   = gcvNULL;
    mmu->mtlbLogical      = gcvNULL;
    mmu->staticSTLB       = gcvNULL;
    mmu->enabled          = gcvFALSE;

    mmu->dynamicAreaSetuped = gcvFALSE;

    gcsLIST_Init(&mmu->hardwareList);

    area = &mmu->dynamicArea;
    area->mapLogical  = gcvNULL;
    area->stlbLogical = gcvNULL;

    /* Create the page table mutex. */
    gcmkONERROR(gckOS_CreateMutex(os, &mmu->pageTableMutex));

    gcmkONERROR(gckOS_QueryOption(os, "mmu", &mmuEnabled));

    if (hardware->mmuVersion == 0)
    {
        area->stlbSize = MmuSize;

        /* Construct address space management table. */
        gcmkONERROR(gckOS_Allocate(mmu->os,
                                   area->stlbSize,
                                   &pointer));

        area->mapLogical = pointer;

        pool = gcdMMU_PGTABLE_POOL;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

        /* Construct page table read by GPU. */
        gcmkONERROR(gckKERNEL_AllocateVideoMemory(
                    Kernel,
                    4096,
                    gcvVIDMEM_TYPE_COMMAND,
                    allocFlag,
                    &area->stlbSize,
                    &pool,
                    &area->stlbVideoMem));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(
                    Kernel,
                    area->stlbVideoMem,
                    gcvFALSE,
                    gcvFALSE,
                    &pointer));

        area->stlbLogical = pointer;

        /* Get CPU physical address. */
        gcmkONERROR(gckVIDMEM_NODE_GetPhysical(
                    Kernel,
                    area->stlbVideoMem,
                    0,
                    &area->stlbPhysical));

        gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(mmu->os,
                    area->stlbPhysical,
                    &area->stlbPhysical));

        /* Compute number of entries in page table. */
        gcmkSAFECASTSIZET(area->stlbEntries, area->stlbSize / sizeof(gctUINT32));

        /* Mark all pages as free. */
        map      = area->mapLogical;

        _FillPageTable(area->stlbLogical, area->stlbEntries, mmu->safeAddress);

        gcmkDUMP(mmu->os, "#[mmu0: fill with safe address]");
        gcmkDUMP(mmu->os, "@[physical.fill 0x%010X 0x%08X 0x%08X]",
                 area->stlbPhysical, area->stlbLogical[0], area->stlbSize / 4);

        map[0] = (area->stlbEntries << 8) | gcvMMU_FREE;
        map[1] = ~0U;
        area->heapList  = 0;
        area->freeNodes = gcvFALSE;

        status = gckOS_QueryOption(mmu->os, "contiguousBase", &contiguousBase);

        if (gcmIS_SUCCESS(status))
        {
            status = gckOS_QueryOption(mmu->os, "contiguousSize", &data);
            contiguousSize = (gctSIZE_T)data;
        }

        if (gcmIS_SUCCESS(status) && contiguousSize)
        {
            /* Convert to GPU address. */
            mmu->contiguousBaseAddress = (gctUINT32)(contiguousBase - Kernel->hardware->baseAddress);
        }
    }
    else
    {
        mmu->mtlbSize = gcdMMU_MTLB_SIZE;

        pool = gcdMMU_PGTABLE_POOL;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

        gcmkONERROR(gckKERNEL_AllocateVideoMemory(
                    Kernel,
                    4096,
                    gcvVIDMEM_TYPE_COMMAND,
                    allocFlag,
                    &mmu->mtlbSize,
                    &pool,
                    &mmu->mtlbVideoMem));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(
                    Kernel,
                    mmu->mtlbVideoMem,
                    gcvFALSE,
                    gcvFALSE,
                    &pointer));

        mmu->mtlbLogical = pointer;

        area->mappingStart = gcvINVALID_ADDRESS;

        /* Get mtlb table physical. */
        gcmkONERROR(gckVIDMEM_NODE_GetPhysical(
                    Kernel,
                    mmu->mtlbVideoMem,
                    0,
                    &mmu->mtlbPhysical));

        gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(
                      mmu->os,
                      mmu->mtlbPhysical,
                      &mmu->mtlbPhysical));

        /* Invalid all the entries. */
        gcmkONERROR(
            gckOS_ZeroMemory(pointer, mmu->mtlbSize));

        gcmkONERROR(
            gckOS_QueryOption(mmu->os, "physBase", &physBase));

        gcmkONERROR(
            gckOS_QueryOption(mmu->os, "physSize", &data));
        physSize = (gctSIZE_T)data;

        gcmkONERROR(
            gckOS_CPUPhysicalToGPUPhysical(mmu->os, physBase, &gpuPhysical));

        gcmkSAFECASTPHYSADDRT(gpuAddress, gpuPhysical);

        if (physSize)
        {
            /* Setup user specified flat mapping. */
            gcmkONERROR(_FillFlatMapping(mmu, gpuAddress, physSize, gcvFALSE, gcvFALSE, gcvNULL));
        }

#ifndef EMULATOR
        if (!_ReadPageEntry(mmu->mtlbLogical + 0))
        {
            gctUINT32 mtlbEntry;
            /*
             * Reserved the first mtlb.
             * 1MB page size, Ingore exception, Not Present.
             */
            mtlbEntry = (1 << 3)
                      | (0 << 1)
                      | (0 << 0);

            _WritePageEntry(mmu->mtlbLogical + 0, mtlbEntry);

            gcmkDUMP(mmu->os, "#[mmu-mtlb: reserved 16M space, slot: 0]");
            gcmkDUMP(mmu->os,
                     "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     (unsigned long long)mmu->mtlbPhysical,
                     mmu->mtlbLogical[0], 4);

            /* Store the gpu virtual ranges */
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].start = 0;
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].end   = (16 << 20) - 1;
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].size  = (16 << 20);
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].flag  = gcvFLATMAP_DIRECT;
            mmu->gpuAddressRangeCount++;
        }
#endif

        status = gckOS_QueryOption(mmu->os, "contiguousBase", &contiguousBase);

        if (gcmIS_SUCCESS(status))
        {
            status = gckOS_QueryOption(mmu->os, "contiguousSize", &data);
            contiguousSize = (gctSIZE_T)data;
        }

        if (gcmIS_SUCCESS(status) && contiguousSize)
        {
            gctUINT64 gpuContiguousBase;
            gctUINT32 contiguousBaseAddress = 0;

            gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(mmu->os, contiguousBase, &gpuContiguousBase));

            /* Setup flat mapping for reserved memory (VIDMEM). */
            gcmkONERROR(_FillFlatMapping(mmu, gpuContiguousBase, contiguousSize, gcvFALSE, gcvTRUE, &contiguousBaseAddress));

            if (mmuEnabled)
            {
                mmu->contiguousBaseAddress = contiguousBaseAddress;
            }
            else
            {
                gcmkSAFECASTPHYSADDRT(mmu->contiguousBaseAddress, gpuContiguousBase);
            }
        }

        status = gckOS_QueryOption(mmu->os, "externalBase", &externalBase);

        if (gcmIS_SUCCESS(status))
        {
            status = gckOS_QueryOption(mmu->os, "externalSize", &data);
            externalSize = (gctSIZE_T)data;
        }

        if (gcmIS_SUCCESS(status) && externalSize)
        {
            gctUINT64 gpuExternalBase;
            gctUINT32 externalBaseAddress = 0;

            gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(mmu->os, externalBase, &gpuExternalBase));

            /* Setup flat mapping for external memory. */
            gcmkONERROR(_FillFlatMapping(mmu, gpuExternalBase, externalSize, gcvFALSE, gcvTRUE, &externalBaseAddress));

            mmu->externalBaseAddress = externalBaseAddress;
        }
    }

    /* A 64 byte for safe address, we use 256 here. */
    mmu->safePageSize = 256;

    pool = gcdMMU_PGTABLE_POOL;

    /* Allocate safe page from video memory. */
    gcmkONERROR(gckKERNEL_AllocateVideoMemory(
        Kernel,
        256,
        gcvVIDMEM_TYPE_COMMAND,
        gcvALLOC_FLAG_CONTIGUOUS,
        &mmu->safePageSize,
        &pool,
        &mmu->safePageVideoMem
        ));

    /* Lock for kernel side CPU access. */
    gcmkONERROR(gckVIDMEM_NODE_LockCPU(
        Kernel,
        mmu->safePageVideoMem,
        gcvFALSE,
        gcvFALSE,
        &mmu->safePageLogical
        ));

    /* Get CPU physical address. */
    gcmkONERROR(gckVIDMEM_NODE_GetPhysical(
        Kernel,
        mmu->safePageVideoMem,
        0,
        &mmu->safePagePhysical
        ));

    gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(
        os,
        mmu->safePagePhysical,
        &mmu->safePagePhysical
        ));

    gcmkSAFECASTPHYSADDRT(mmu->safeAddress, mmu->safePagePhysical);

    gckOS_ZeroMemory(mmu->safePageLogical, mmu->safePageSize);

    gcmkDUMP(mmu->os, "#[safe page]");
    gcmkDUMP(
        mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
        mmu->safePagePhysical, 0, 4096
        );

    gcmkDUMP_BUFFER(
        mmu->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        mmu->safePageLogical,
        mmu->safeAddress,
        4096
        );

    gcmkONERROR(gckQUEUE_Allocate(os, &mmu->recentFreedAddresses, 16));

    mmu->sRAMMapped = gcvFALSE;

    /* Return the gckMMU object pointer. */
    *Mmu = mmu;

    /* Success. */
    gcmkFOOTER_ARG("*Mmu=0x%x", *Mmu);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    if (mmu != gcvNULL)
    {
        if (area != gcvNULL && area->mapLogical != gcvNULL)
        {
            gcmkVERIFY_OK(
                gckOS_Free(os, (gctPOINTER) area->mapLogical));

            gcmkVERIFY_OK(
                gckVIDMEM_NODE_Dereference(Kernel,
                                           area->stlbVideoMem));
        }

        if (mmu->mtlbLogical != gcvNULL)
        {
            gcmkVERIFY_OK(
                gckVIDMEM_NODE_Dereference(Kernel,
                                           mmu->mtlbVideoMem));
        }

        if (mmu->pageTableMutex != gcvNULL)
        {
            /* Delete the mutex. */
            gcmkVERIFY_OK(
                gckOS_DeleteMutex(os, mmu->pageTableMutex));
        }

        gcmkVERIFY_OK(gckQUEUE_Free(os, &mmu->recentFreedAddresses));

        /* Mark the gckMMU object as unknown. */
        mmu->object.type = gcvOBJ_UNKNOWN;

        /* Free the allocates memory. */
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, mmu));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_FreeAddressArea(
    gckKERNEL Kernel,
    gcsADDRESS_AREA * Area
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (Area->mapLogical != gcvNULL)
    {
        gcmkVERIFY_OK(
            gckOS_Free(Kernel->os, (gctPOINTER) Area->mapLogical));
    }

    if (Area->stlbLogical != gcvNULL)
    {
        /* Free page table. */
        gcmkVERIFY_OK(
            gckVIDMEM_NODE_Dereference(Kernel,
                                       Area->stlbVideoMem));
    }

    return status;
}

/*******************************************************************************
**
**  _Destroy
**
**  Destroy a gckMMU object.
**
**  INPUT:
**
**      gckMMU Mmu
**          Pointer to an gckMMU object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
_Destroy(
    IN gckMMU Mmu
    )
{
    gckKERNEL kernel = Mmu->hardware->kernel;

    gcmkHEADER_ARG("Mmu=0x%x", Mmu);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    while (Mmu->staticSTLB != gcvNULL)
    {
        gcsMMU_STLB_CHUNK_PTR pre = Mmu->staticSTLB;
        Mmu->staticSTLB = pre->next;

        if (pre->videoMem)
        {
            gcmkVERIFY_OK(
                gckVIDMEM_NODE_Dereference(kernel,
                                           pre->videoMem));
        }

        if (pre->mtlbEntryNum != 0)
        {
            gctUINT i;
            for (i = 0; i < pre->mtlbEntryNum; ++i)
            {
                _WritePageEntry(Mmu->mtlbLogical + pre->mtlbIndex + i, 0);
#if gcdMMU_TABLE_DUMP
                gckOS_Print("%s(%d): clean MTLB[%d]\n",
                    __FUNCTION__, __LINE__,
                    pre->mtlbIndex + i);
#endif
            }

            gcmkDUMP(Mmu->os,
                     "#[mmu-mtlb: clean up slot: %d - %d]",
                     pre->mtlbIndex,
                     pre->mtlbIndex + pre->mtlbEntryNum - 1);

            gcmkDUMP(Mmu->os,
                     "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     Mmu->mtlbPhysical + pre->mtlbIndex * 4,
                     Mmu->mtlbLogical[pre->mtlbIndex],
                     pre->mtlbEntryNum * 4);
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, pre));
    }

    if (Mmu->hardware->mmuVersion != 0)
    {
        gcmkVERIFY_OK(
            gckVIDMEM_NODE_Dereference(kernel,
                                       Mmu->mtlbVideoMem));
    }

    /* Free address area. */
    gcmkVERIFY_OK(_FreeAddressArea(kernel, &Mmu->dynamicArea));
    gcmkVERIFY_OK(_FreeAddressArea(kernel, &Mmu->secureArea));

    /* Delete the page table mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Mmu->os, Mmu->pageTableMutex));

    if (Mmu->safePageLogical != gcvNULL)
    {
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
            kernel,
            Mmu->safePageVideoMem
            ));
    }

    gcmkVERIFY_OK(gckQUEUE_Free(Mmu->os, &Mmu->recentFreedAddresses));

    /* Mark the gckMMU object as unknown. */
    Mmu->object.type = gcvOBJ_UNKNOWN;

    /* Free the gckMMU object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, Mmu));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
** _AdjstIndex
**
**  Adjust the index from which we search for a usable node to make sure
**  index allocated is greater than Start.
*/
gceSTATUS
_AdjustIndex(
    IN gckMMU Mmu,
    IN gctUINT32 Index,
    IN gctUINT32 PageCount,
    IN gctUINT32 Start,
    OUT gctUINT32 * IndexAdjusted
    )
{
    gceSTATUS status;
    gctUINT32 index = Index;
    gcsADDRESS_AREA_PTR area = &Mmu->dynamicArea;
    gctUINT32_PTR map = area->mapLogical;

    gcmkHEADER();

    for (; index < area->stlbEntries;)
    {
        gctUINT32 result = 0;
        gctUINT32 nodeSize = 0;

        if (index >= Start)
        {
            break;
        }

        switch (gcmENTRY_TYPE(map[index]))
        {
        case gcvMMU_SINGLE:
            nodeSize = 1;
            break;

        case gcvMMU_FREE:
            nodeSize = map[index] >> 8;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", index);
            gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
        }

        if (nodeSize > PageCount)
        {
            result = index + (nodeSize - PageCount);

            if (result >= Start)
            {
                break;
            }
        }

        switch (gcmENTRY_TYPE(map[index]))
        {
        case gcvMMU_SINGLE:
            index = map[index] >> 8;
            break;

        case gcvMMU_FREE:
            index = map[index + 1];
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", index);
            gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
        }
    }

    *IndexAdjusted = index;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_Construct(
    IN gckKERNEL Kernel,
    IN gctSIZE_T MmuSize,
    OUT gckMMU * Mmu
    )
{
#if gcdSHARED_PAGETABLE
    gceSTATUS status;
    gctPOINTER pointer;

    gcmkHEADER_ARG("Kernel=0x%08x", Kernel);

    if (sharedPageTable == gcvNULL)
    {
        gcmkONERROR(
                gckOS_Allocate(Kernel->os,
                               sizeof(struct _gcsSharedPageTable),
                               &pointer));
        sharedPageTable = pointer;

        gcmkONERROR(
                gckOS_ZeroMemory(sharedPageTable,
                    sizeof(struct _gcsSharedPageTable)));

        gcmkONERROR(_Construct(Kernel, MmuSize, &sharedPageTable->mmu));
    }

    *Mmu = sharedPageTable->mmu;

    sharedPageTable->hardwares[sharedPageTable->reference] = Kernel->hardware;

    sharedPageTable->reference++;

    gcmkFOOTER_ARG("sharedPageTable->reference=%lu", sharedPageTable->reference);
    return gcvSTATUS_OK;

OnError:
    if (sharedPageTable)
    {
        if (sharedPageTable->mmu)
        {
            gcmkVERIFY_OK(gckMMU_Destroy(sharedPageTable->mmu));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, sharedPageTable));
    }

    gcmkFOOTER();
    return status;
#else
    return _Construct(Kernel, MmuSize, Mmu);
#endif
}

gceSTATUS
gckMMU_Destroy(
    IN gckMMU Mmu
    )
{
#if gcdSHARED_PAGETABLE
    gckOS os = Mmu->os;

    sharedPageTable->reference--;

    if (sharedPageTable->reference == 0)
    {
        if (sharedPageTable->mmu)
        {
            gcmkVERIFY_OK(_Destroy(Mmu));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, sharedPageTable));
    }

    return gcvSTATUS_OK;
#else
    return _Destroy(Mmu);
#endif
}

/*******************************************************************************
**
**  gckMMU_AllocatePages
**
**  Allocate pages inside the page table.
**
**  INPUT:
**
**      gckMMU Mmu
**          Pointer to an gckMMU object.
**
**      gctSIZE_T PageCount
**          Number of pages to allocate.
**
**  OUTPUT:
**
**      gctPOINTER * PageTable
**          Pointer to a variable that receives the base address of the page
**          table.
**
**      gctUINT32 * Address
**          Pointer to a variable that receives the hardware specific address.
*/
gceSTATUS
_AllocatePages(
    IN gckMMU Mmu,
    IN gctSIZE_T PageCount,
    IN gceVIDMEM_TYPE Type,
    IN gctBOOL Secure,
    OUT gctPOINTER * PageTable,
    OUT gctUINT32 * Address
    )
{
    gceSTATUS status;
    gctBOOL mutex = gcvFALSE;
    gctUINT32 index = 0, previous = ~0U, left;
    gctUINT32_PTR map;
    gctBOOL gotIt;
    gctUINT32 address;
    gctUINT32 pageCount;
    gcsADDRESS_AREA_PTR area = _GetProcessArea(Mmu, Secure);

    gcmkHEADER_ARG("Mmu=0x%x PageCount=%lu", Mmu, PageCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageCount > 0);
    gcmkVERIFY_ARGUMENT(PageTable != gcvNULL);

    if (PageCount > area->stlbEntries)
    {
        /* Not enough pages avaiable. */
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    gcmkSAFECASTSIZET(pageCount, PageCount);

#if gcdBOUNDARY_CHECK
    /* Extra pages as bounary. */
    pageCount += gcdBOUNDARY_CHECK * 2;
#endif

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    mutex = gcvTRUE;

    /* Cast pointer to page table. */
    for (map = area->mapLogical, gotIt = gcvFALSE; !gotIt;)
    {
        index = area->heapList;

        if ((Mmu->hardware->mmuVersion == 0) &&
            (Type == gcvVIDMEM_TYPE_VERTEX_BUFFER))
        {
            gcmkONERROR(_AdjustIndex(
                Mmu,
                index,
                pageCount,
                gcdVERTEX_START / gcmSIZEOF(gctUINT32),
                &index
                ));
        }

        /* Walk the heap list. */
        for (; !gotIt && (index < area->stlbEntries);)
        {
            /* Check the node type. */
            switch (gcmENTRY_TYPE(map[index]))
            {
            case gcvMMU_SINGLE:
                /* Single odes are valid if we only need 1 page. */
                if (pageCount == 1)
                {
                    gotIt = gcvTRUE;
                }
                else
                {
                    /* Move to next node. */
                    previous = index;
                    index    = map[index] >> 8;
                }
                break;

            case gcvMMU_FREE:
                /* Test if the node has enough space. */
                if (pageCount <= (map[index] >> 8))
                {
                    gotIt = gcvTRUE;
                }
                else
                {
                    /* Move to next node. */
                    previous = index;
                    index    = map[index + 1];
                }
                break;

            default:
                gcmkFATAL("MMU table correcupted at index %u!", index);
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }
        }

        /* Test if we are out of memory. */
        if (index >= area->stlbEntries)
        {
            if (area->freeNodes)
            {
                /* Time to move out the trash! */
                gcmkONERROR(_Collect(area));

                /* We are going to search from start, so reset previous to start. */
                previous = ~0U;
            }
            else
            {
                /* Out of resources. */
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }
        }
    }

    switch (gcmENTRY_TYPE(map[index]))
    {
    case gcvMMU_SINGLE:
        /* Unlink single node from free list. */
        gcmkONERROR(
            _Link(area, previous, map[index] >> 8));
        break;

    case gcvMMU_FREE:
        /* Check how many pages will be left. */
        left = (map[index] >> 8) - pageCount;
        switch (left)
        {
        case 0:
            /* The entire node is consumed, just unlink it. */
            gcmkONERROR(
                _Link(area, previous, map[index + 1]));
            break;

        case 1:
            /* One page will remain.  Convert the node to a single node and
            ** advance the index. */
            map[index] = (map[index + 1] << 8) | gcvMMU_SINGLE;
            index ++;
            break;

        default:
            /* Enough pages remain for a new node.  However, we will just adjust
            ** the size of the current node and advance the index. */
            map[index] = (left << 8) | gcvMMU_FREE;
            index += left;
            break;
        }
        break;
    }

    /* Mark node as used. */
    gcmkONERROR(_FillMap(&map[index], pageCount, gcvMMU_USED));

#if gcdBOUNDARY_CHECK
    index += gcdBOUNDARY_CHECK;
#endif

    /* Record pageCount of allocated node at the beginning of node. */
    if (pageCount == 1)
    {
        map[index] = (~((1U<<8)-1)) | gcvMMU_USED;
    }
    else
    {
        map[index] = (pageCount << 8) | gcvMMU_USED;
    }

    if (area->stlbLogical != gcvNULL)
    {
        /* Return pointer to page table. */
        *PageTable = &area->stlbLogical[index];
    }
    else
    {
        /* Page table for secure area is handled in trust application. */
        *PageTable = gcvNULL;
    }

    /* Build virtual address. */
    if (Mmu->hardware->mmuVersion == 0)
    {
        gcmkONERROR(
                gckHARDWARE_BuildVirtualAddress(Mmu->hardware, index, 0, &address));
    }
    else
    {
        gctUINT32 masterOffset = index / gcdMMU_STLB_4K_ENTRY_NUM
                               + area->mappingStart;
        gctUINT32 slaveOffset = index % gcdMMU_STLB_4K_ENTRY_NUM;

        address = (masterOffset << gcdMMU_MTLB_SHIFT)
                | (slaveOffset << gcdMMU_STLB_4K_SHIFT);
    }

    if (Address != gcvNULL)
    {
        *Address = address;
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    /* Success. */
    gcmkFOOTER_ARG("*PageTable=0x%x *Address=%08x",
                   *PageTable, gcmOPT_VALUE(Address));
    return gcvSTATUS_OK;

OnError:

    if (mutex)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckMMU_FreePages
**
**  Free pages inside the page table.
**
**  INPUT:
**
**      gckMMU Mmu
**          Pointer to an gckMMU object.
**
**      gctPOINTER PageTable
**          Base address of the page table to free.
**
**      gctSIZE_T PageCount
**          Number of pages to free.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
_FreePages(
    IN gckMMU Mmu,
    IN gctBOOL Secure,
    IN gctUINT32 Address,
    IN gctPOINTER PageTable,
    IN gctSIZE_T PageCount
    )
{
    gctUINT32 index;
    gctUINT32_PTR node;
    gceSTATUS status;
    gctBOOL acquired = gcvFALSE;
    gctUINT32 pageCount;
    gcuQUEUEDATA data;
    gcsADDRESS_AREA_PTR area = _GetProcessArea(Mmu, Secure);

    gcmkHEADER_ARG("Mmu=0x%x PageTable=0x%x PageCount=%lu",
                   Mmu, PageTable, PageCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageCount > 0);

    gcmkSAFECASTSIZET(pageCount, PageCount);

#if gcdBOUNDARY_CHECK
    pageCount += gcdBOUNDARY_CHECK * 2;
#endif

    /* Get the node by index. */
    index = (gctUINT32)((gctUINT32_PTR)PageTable - area->stlbLogical);
    node = area->mapLogical + index;

    if (pageCount != _GetPageCountOfUsedNode(node))
    {
        gcmkONERROR(gcvSTATUS_INVALID_REQUEST);
    }

#if gcdBOUNDARY_CHECK
    node -= gcdBOUNDARY_CHECK;
#endif

    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (Mmu->hardware->mmuVersion == 0)
    {
        _FillPageTable(PageTable, pageCount, Mmu->safeAddress);
    }

    if (pageCount == 1)
    {
       /* Single page node. */
        node[0] = (~((1U<<8)-1)) | gcvMMU_SINGLE;

        if (PageTable != gcvNULL)
        {
#if gcdUSE_MMU_EXCEPTION
        /* Enable exception */
        _WritePageEntry(PageTable, (1 << 1));
#else
        _WritePageEntry(PageTable, 0);
#endif
        }
    }
    else
    {
        /* Mark the node as free. */
        node[0] = (pageCount << 8) | gcvMMU_FREE;
        node[1] = ~0U;

        if (PageTable != gcvNULL)
        {
#if gcdUSE_MMU_EXCEPTION
            /* Enable exception */
            gcmkVERIFY_OK(_FillPageTable(PageTable, (gctUINT32)PageCount, 1 << 1));
#else
            gcmkVERIFY_OK(_FillPageTable(PageTable, (gctUINT32)PageCount, 0));
#endif
        }
    }

    gcmkDUMP(Mmu->os, "#[mmu-stlb: free 0x%08X - 0x%08X]",
             Address, Address + pageCount * gcdMMU_PAGE_4K_SIZE - 1);

    gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
             area->stlbPhysical + index * 4, *(gctUINT32_PTR)PageTable, pageCount * 4);

    /* We have free nodes. */
    area->freeNodes = gcvTRUE;

    /* Record freed address range. */
    data.addressData.start = Address;
    data.addressData.end = Address + (gctUINT32)PageCount * gcdMMU_PAGE_4K_SIZE;
    gckQUEUE_Enqueue(&Mmu->recentFreedAddresses, &data);

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    acquired = gcvFALSE;

#if gcdENABLE_TRUST_APPLICATION
    if (Mmu->hardware->options.secureMode == gcvSECURE_IN_TA)
    {
        gckKERNEL_SecurityUnmapMemory(Mmu->hardware->kernel, Address, (gctUINT32)PageCount);
    }
#endif

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_AllocatePages(
    IN gckMMU Mmu,
    IN gctSIZE_T PageCount,
    OUT gctPOINTER * PageTable,
    OUT gctUINT32 * Address
    )
{
    return gckMMU_AllocatePagesEx(
                Mmu, PageCount, gcvVIDMEM_TYPE_GENERIC, gcvFALSE, PageTable, Address);
}

gceSTATUS
gckMMU_AllocatePagesEx(
    IN gckMMU Mmu,
    IN gctSIZE_T PageCount,
    IN gceVIDMEM_TYPE Type,
    IN gctBOOL Secure,
    OUT gctPOINTER * PageTable,
    OUT gctUINT32 * Address
    )
{
#if gcdDISABLE_GPU_VIRTUAL_ADDRESS
    gcmkPRINT("GPU virtual address is disabled.");
    return gcvSTATUS_NOT_SUPPORTED;
#else
    return _AllocatePages(Mmu, PageCount, Type, Secure, PageTable, Address);
#endif
}

gceSTATUS
gckMMU_FreePages(
    IN gckMMU Mmu,
    IN gctBOOL Secure,
    IN gctUINT32 Address,
    IN gctPOINTER PageTable,
    IN gctSIZE_T PageCount
    )
{
    return _FreePages(Mmu, Secure, Address, PageTable, PageCount);
}

gceSTATUS
gckMMU_SetPage(
    IN gckMMU Mmu,
    IN gctPHYS_ADDR_T PageAddress,
    IN gctBOOL Writable,
    IN gctUINT32 *PageEntry
    )
{
    gctUINT32 addressExt;
    gctUINT32 address;

    gcmkHEADER_ARG("Mmu=0x%x", Mmu);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageEntry != gcvNULL);
    gcmkVERIFY_ARGUMENT(!(PageAddress & 0xFFF));

    /* [31:0]. */
    address    = (gctUINT32)(PageAddress & 0xFFFFFFFF);
    /* [39:32]. */
    addressExt = (gctUINT32)((PageAddress >> 32) & 0xFF);

    if (Mmu->hardware->mmuVersion == 0)
    {
        _WritePageEntry(PageEntry, address);
    }
    else
    {
        _WritePageEntry(PageEntry, _SetPage(address, addressExt, gcvTRUE));
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckMMU_Flush(
    IN gckMMU Mmu,
    IN gceVIDMEM_TYPE Type
    )
{
    gckHARDWARE hardware;
    gctUINT32 mask;
    gctINT i;

    if (Type == gcvVIDMEM_TYPE_VERTEX_BUFFER ||
        Type == gcvVIDMEM_TYPE_INDEX_BUFFER ||
        Type == gcvVIDMEM_TYPE_COMMAND)
    {
        mask = gcvPAGE_TABLE_DIRTY_BIT_FE;
    }
    else
    {
        mask = gcvPAGE_TABLE_DIRTY_BIT_OTHER;
    }

#if gcdSHARED_PAGETABLE
    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        gctUINT j;
        hardware = sharedPageTable->hardwares[i];
        if (hardware)
        {
            for (j = 0; j < gcvENGINE_GPU_ENGINE_COUNT; j++)
            {
                gcmkVERIFY_OK(gckOS_AtomSetMask(hardware->pageTableDirty[j], mask));
            }
        }
    }
#else
    hardware = Mmu->hardware;

    for (i = 0 ; i < gcvENGINE_GPU_ENGINE_COUNT; i++)
    {
        gcmkVERIFY_OK(
            gckOS_AtomSetMask(hardware->pageTableDirty[i], mask));
    }

    {
        gcsLISTHEAD_PTR hardwareHead;
        gcmkLIST_FOR_EACH(hardwareHead, &Mmu->hardwareList)
        {
            hardware = gcmCONTAINEROF(hardwareHead, struct _gckHARDWARE, mmuHead);

            if (hardware != Mmu->hardware)
            {
                for (i = 0 ; i < gcvENGINE_GPU_ENGINE_COUNT; i++)
                {
                    gcmkVERIFY_OK(
                        gckOS_AtomSetMask(hardware->pageTableDirty[i], mask));
                }
            }
        }
    }
#endif

    return gcvSTATUS_OK;
}

gceSTATUS
gckMMU_DumpPageTableEntry(
    IN gckMMU Mmu,
    IN gctUINT32 Address
    )
{
    gctUINT32_PTR pageTable;
    gctUINT32 index;
    gctUINT32 mtlb, stlb;
    gcsADDRESS_AREA_PTR area = &Mmu->dynamicArea;

    gcmkHEADER_ARG("Mmu=0x%08X Address=0x%08X", Mmu, Address);
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    gcmkASSERT(Mmu->hardware->mmuVersion > 0);

    mtlb   = (Address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT;

    if (mtlb >= area->mappingStart)
    {
        stlb   = (Address & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT;

        pageTable = area->stlbLogical;

        index = (mtlb - area->mappingStart)
              * gcdMMU_STLB_4K_ENTRY_NUM
              + stlb;

        gcmkPRINT("    Page table entry = 0x%08X", _ReadPageEntry(pageTable + index));
    }
    else
    {
        gcsMMU_STLB_CHUNK_PTR stlbChunkObj = Mmu->staticSTLB;
        gctUINT32 entry = Mmu->mtlbLogical[mtlb];

        stlb = (Address & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT;

        entry &= 0xFFFFFFF0;

        while (stlbChunkObj)
        {
            gctUINT i;
            gctBOOL found = gcvFALSE;
            for (i = 0; i < stlbChunkObj->mtlbEntryNum; ++i)
            {
                gctPHYS_ADDR_T stlbPhysBase = stlbChunkObj->physBase + (i * gcdMMU_STLB_1M_SIZE);
                gctUINT32_PTR stlbLogical =
                    (gctUINT32_PTR)((gctUINT8_PTR)stlbChunkObj->logical + (i * gcdMMU_STLB_1M_SIZE));
                if (entry == stlbPhysBase)
                {
                    gcmkPRINT("    Page table entry = 0x%08X", stlbLogical[stlb]);
                    found = gcvTRUE;
                    break;
                }
            }
            if (found)
                break;
            stlbChunkObj = stlbChunkObj->next;
        }
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

void
gckMMU_CheckSaftPage(
    IN gckMMU Mmu
    )
{
    gctUINT8_PTR safeLogical = Mmu->safePageLogical;
    gctUINT32 offsets[] = {
        0,
        64,
        128,
        256,
        2560,
        4000
    };

    gctUINT32 i = 0;

    while (i < gcmCOUNTOF(offsets))
    {
        if (safeLogical[offsets[i]] != 0)
        {
            gcmkPRINT("%s(%d) safe page is over written [%d] = %x",
                      __FUNCTION__, __LINE__, i, safeLogical[offsets[i]]);
        }
    }
}

void
gckMMU_DumpAddressSpace(
    IN gckMMU Mmu
    )
{
    gctUINT i;
    gctUINT next;
    gcsADDRESS_AREA_PTR area = &Mmu->dynamicArea;
    gctUINT32_PTR map = area->mapLogical;
    gctBOOL used = gcvFALSE;
    gctUINT32 numPages;

    /* Grab the mutex. */
    gcmkVERIFY_OK(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));

    /* Find node which contains index. */
    for (i = 0; i < area->stlbEntries; i = next)
    {
        switch (gcmENTRY_TYPE(map[i]))
        {
        case gcvMMU_SINGLE:
            numPages = 1;
            next = i + numPages;
            used = gcvFALSE;
            break;

        case gcvMMU_FREE:
            numPages = map[i] >> 8;
            next = i + numPages;
            used = gcvFALSE;
            break;

        case gcvMMU_USED:
            numPages = 1;
            next = i + numPages;
            used = gcvTRUE;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", i);
            return;
        }

        if (!used)
        {
            gcmkPRINT("Available Range [%d - %d)", i, i + numPages);
        }
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

}

void
gckMMU_DumpRecentFreedAddress(
    IN gckMMU Mmu
    )
{
    gckQUEUE queue = &Mmu->recentFreedAddresses;
    gctUINT32 i;
    gcuQUEUEDATA *data;

    if (queue->count)
    {
        gcmkPRINT("    Recent %d freed GPU address ranges:", queue->count);

        for (i = 0; i < queue->count; i++)
        {
            gckQUEUE_GetData(queue, i, &data);

            gcmkPRINT("      [%08X - %08X]", data->addressData.start, data->addressData.end);
        }
    }
}

gceSTATUS
gckMMU_FillFlatMapping(
    IN gckMMU Mmu,
    IN gctUINT64 PhysBase,
    IN gctSIZE_T Size
    )
{
    gceSTATUS status;
    gckHARDWARE hardware = Mmu->hardware;

    if (hardware->mmuVersion)
    {
        gcmkONERROR(_FillFlatMapping(Mmu, PhysBase, Size, gcvFALSE, gcvTRUE, gcvNULL));
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckMMU_IsFlatMapped(
    IN gckMMU Mmu,
    OUT gctUINT64 Physical,
    OUT gctBOOL *In
    )
{
    gceSTATUS status;
    gctUINT32 i;
    gctBOOL inFlatmapping = gcvFALSE;
    gcmkHEADER();

    gcmkVERIFY_ARGUMENT(In != gcvNULL);

    if (gckHARDWARE_IsFeatureAvailable(Mmu->hardware, gcvFEATURE_MMU) == gcvFALSE)
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    for (i = 0; i < Mmu->gpuPhysicalRangeCount; i++)
    {
        if ((Physical >= Mmu->gpuPhysicalRanges[i].start) &&
            (Physical < Mmu->gpuPhysicalRanges[i].end))
        {
            inFlatmapping = gcvTRUE;
            break;
        }
    }

    *In = inFlatmapping;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_SetupPerHardware(
    IN gckMMU Mmu,
    IN gckHARDWARE Hardware,
    IN gckDEVICE Device
    )
{
    gctBOOL needMapInternalSRAM = gcvFALSE;
    gctPHYS_ADDR_T reservedBase = gcvINVALID_PHYSICAL_ADDRESS;
    gctUINT32 reservedSize = 0;
    gctUINT i = 0;
    gctUINT j = 0;
    gceSTATUS status;
    gckKERNEL kernel = Hardware->kernel;

    gcmkHEADER_ARG("Mmu=0x%x Hardware=0x%x", Mmu, Hardware);

    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    if (Hardware->mmuVersion == 0)
    {
        gcmkONERROR(gcvSTATUS_OK);
    }

    if (!Mmu->sRAMMapped)
    {
        /* Map all the SRAMs in MMU table. */
        for (i = 0; i < gcvCORE_COUNT; i++)
        {
            reservedBase = Device->sRAMBases[i][gcvSRAM_INTERNAL];
            reservedSize = Device->sRAMSizes[i][gcvSRAM_INTERNAL];
            needMapInternalSRAM = reservedSize && (reservedBase != gcvINVALID_PHYSICAL_ADDRESS);

            /* Map the internal SRAM. */
            if (needMapInternalSRAM)
            {
                gcmkPRINT("Galcore Info: MMU mapped core%d internal SRAM base=0x%llx size=0x%x",
                    i,
                    reservedBase,
                    reservedSize
                    );

                /*
                 * Default gpu virtual base = 0.
                 * It can be specified if not conflict with existing mapping.
                 */

                Device->sRAMBaseAddresses[i][gcvSRAM_INTERNAL] = 0;

                gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(
                    Mmu->os,
                    reservedBase,
                    &reservedBase
                    ));

                gcmkONERROR(_FillFlatMapping(
                    Mmu,
                    reservedBase,
                    reservedSize,
                    gcvTRUE,
                    gcvTRUE,
                    &Device->sRAMBaseAddresses[i][gcvSRAM_INTERNAL]
                    ));

                Device->sRAMBases[i][j] = reservedBase;
            }

            /* Map all the axi SRAMs in MMU table. */
            for (j = gcvSRAM_EXTERNAL0; j < gcvSRAM_COUNT; j++)
            {
                if (Device->sRAMSizes[i][j] &&
                   (Device->sRAMBases[i][j] != gcvINVALID_PHYSICAL_ADDRESS))
                {
                    gcmkPRINT("Galcore Info: MMU mapped core %d SRAM[%d] base=0x%llx size=0x%x",
                        i,
                        j,
                        Device->sRAMBases[i][j],
                        Device->sRAMSizes[i][j]
                        );

                    Device->sRAMBaseAddresses[i][j] = 0;

                    gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(
                        Mmu->os,
                        Device->sRAMBases[i][j],
                        &Device->sRAMBases[i][j]
                        ));

                    gcmkONERROR(_FillFlatMapping(
                        Mmu,
                        Device->sRAMBases[i][j],
                        Device->sRAMSizes[i][j],
                        gcvFALSE,
                        gcvTRUE,
                        &Device->sRAMBaseAddresses[i][j]
                        ));
                }
            }
        }

        Mmu->sRAMMapped = gcvTRUE;
    }

    /* Get per core SRAM hardware base address. */
    for (i = 0; i < gcvSRAM_COUNT; i++)
    {
        if (Device->sRAMSizes[Hardware->core][i] &&
           (Device->sRAMBases[Hardware->core][i] != gcvINVALID_PHYSICAL_ADDRESS))
        {
            kernel->sRAMBaseAddresses[i] = Hardware->options.sRAMBaseAddresses[i]
                                         = Device->sRAMBaseAddresses[Hardware->core][i];

            kernel->sRAMSizes[i] = Hardware->options.sRAMSizes[i]
                                 = Device->sRAMSizes[Hardware->core][i];

            Hardware->options.sRAMPhysicalBases[i] = Device->sRAMBases[Hardware->core][i];

            gcmkPRINT("Galcore Info: MMU mapped core %d SRAM[%d] hardware address=0x%x size=0x%x",
                Hardware->core,
                i,
                kernel->sRAMBaseAddresses[i],
                kernel->sRAMSizes[i]
                );
        }
    }

    gcsLIST_Add(&Hardware->mmuHead, &Mmu->hardwareList);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the error. */
    gcmkFOOTER();
    return status;
}


gceSTATUS
gckMMU_GetPageEntry(
    IN gckMMU Mmu,
    IN gctUINT32 Address,
    IN gctUINT32_PTR *PageTable
    )
{
    gctUINT32_PTR pageTable;
    gctUINT32 index;
    gctUINT32 mtlb, stlb;
    gcsADDRESS_AREA_PTR area = &Mmu->dynamicArea;

    gcmkHEADER_ARG("Mmu=0x%08X Address=0x%08X", Mmu, Address);
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    gcmkASSERT(Mmu->hardware->mmuVersion > 0);

    mtlb   = (Address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT;

    if (mtlb >= area->mappingStart)
    {
        stlb   = (Address & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT;

        pageTable = area->stlbLogical;

        index = (mtlb - area->mappingStart)
            * gcdMMU_STLB_4K_ENTRY_NUM
            + stlb;

        *PageTable = pageTable + index;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/******************************************************************************
****************************** T E S T   C O D E ******************************
******************************************************************************/


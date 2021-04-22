/**
*******************************************************************************
*
* @file bimsvc.c
*
* @brief Handlers for DSP BIM modules
* @author Mindspeed Technologies
*
* COPYRIGHT&copy; 2010 Mindspeed Technologies.
* ALL RIGHTS RESERVED
*
* This is Unpublished Proprietary Source Code of Mindspeed Technologies
*
******************************************************************************/


#include "bimsvc.h"
#include "scmgr.h"
#include "spudrv.h"
#include "ovlsvc.h"
#include "syslib.h"
#include "agc.h"

#define FIX_SDRAM_ADDR(p_bim, p_obj) ((U32)p_bim + sizeof(U32) * p_obj->sdram_addr)

extern SPEECHIO speechio;

extern BIMPLUGINTABLE BimPlugInTable[];

/********************************************************************************
*   NAME:			BIMSVC_Init													*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	None.														*
*																				*
*	DESCRIPTION:	Initialize device specific BIM table entries.				*
*																				*
********************************************************************************/ 

void BIMSVC_Init(void)
{
	extern U32 g726_0[];
	extern U32 g726_1[];
	extern U32 g726_coproc[];
	extern CODECOVLDESC CodecOverlayDesc[];
	extern CODECOVLDESC CodecOverlayDescWithPrefetch[];

	U32 idx = BIMSVC_GetCodecIndex(CODECID_G726);

	if (ISMIRODEVICE())
	{
		BimPlugInTable[CODECID_G726].nOverlays = 2;
		BimPlugInTable[CODECID_G726].pBimDesc = g726_0;
		BimPlugInTable[CODECID_G726+1].pBimDesc = g726_1;

		CodecOverlayDesc[idx].numDecoderOverlays = 1;
		CodecOverlayDesc[idx].numEncoderOverlays = 1;
		CodecOverlayDesc[idx].EncoderOverlays[0] = 1;

		CodecOverlayDescWithPrefetch[idx].numDecoderOverlays = 1;
		CodecOverlayDescWithPrefetch[idx].numEncoderOverlays = 1;
		CodecOverlayDescWithPrefetch[idx].EncoderOverlays[0] = 1;
	}
	else
	{
		BimPlugInTable[CODECID_G726].nOverlays = 1;
		BimPlugInTable[CODECID_G726].pBimDesc = g726_coproc;
		BimPlugInTable[CODECID_G726+1].pBimDesc = NULL;

		CodecOverlayDesc[idx].numDecoderOverlays = 1;
		CodecOverlayDesc[idx].numEncoderOverlays = 0;
		CodecOverlayDesc[idx].EncoderOverlays[0] = 0;

		CodecOverlayDescWithPrefetch[idx].numDecoderOverlays = 1;
		CodecOverlayDescWithPrefetch[idx].numEncoderOverlays = 0;
		CodecOverlayDescWithPrefetch[idx].EncoderOverlays[0] = 0;
	}
}

/********************************************************************************
*   NAME:			BIMSVC_GetCodecIndex										*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	CODEC's index.												*
*																				*
*	DESCRIPTION:	Converts CODEC ID to index.									*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetCodecIndex(U32 id)
{
	static U16 ConvertTable[] = {

		CODECID_G711,
		CODECID_G711SIGNALING,
		CODECID_G723,
		CODECID_G726,
		CODECID_G728,
		CODECID_G729,
		CODECID_G729_MSPD,
		CODECID_G729_WB,
		CODECID_G729EG,
		CODECID_CRBT,
		CODECID_EVRC,
		CODECID_QCELP8K,
		CODECID_QCELP13K,
		CODECID_SMV,
		CODECID_4GV,
		CODECID_GSMAMR,
		CODECID_GSMFR, 
		CODECID_ILBC,
		CODECID_G723STD,
		CODECID_GSMEFR,
		CODECID_AMRWB,
		CODECID_CSD,
#ifdef CPCT_BUILD
		CODECID_CPCT,
#endif
		CODECID_GSMHR,
		CODECID_G722
	};
	U32 i = 0;

	while (i < sizeof(ConvertTable))
	{
		if (ConvertTable[i] == id)
			break;
		i++;
	}

	if(i >= sizeof(ConvertTable))
		Alert (ALERT_VALUE_GE, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, AlertId_BIMSVC_GetCodecIndex_1, i,sizeof(ConvertTable));	

	return i;
}

/********************************************************************************
*   NAME:			BIMSVC_GetNumberOfOverlays									*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Number of CODEC's overlays.									*
*																				*
*	DESCRIPTION:	Returns number of overlays.									*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetNumberOfOverlays(U32 id)
{
	return BimPlugInTable[id].nOverlays;
}

/********************************************************************************
*   NAME:			BIMSVC_GetNumberOfDesc										*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Number of required SMC descriptors.							*
*																				*
*	DESCRIPTION:	Calculates total number of descriptors.						*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetNumberOfDesc(U32 id)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
	U32 nDesc = 0;
	PBIMOBJDESC pBimObj = NULL;
	U32 i;

	if (pBimDesc)
	{
		nDesc = pBimDesc->NumCodeObj;
		pBimObj = &pBimDesc->ObjDesc[nDesc];
		for (i = 0; i < pBimDesc->NumDataObj; i++)
		{
			if (pBimObj->Type & OBJTYPE_CDATA)
			{
				nDesc++;
			}
			else if (pBimObj->Type & (OBJTYPE_IOSTREAM | OBJTYPE_IOPARAMS | OBJTYPE_SDATA))
			{
				nDesc += 2;
			}
			else if (pBimObj->Type & OBJTYPE_IOSPEECH)
				nDesc += 4;

			pBimObj++;
		}
	}

	return nDesc;
}

/********************************************************************************
*   NAME:			BIMSVC_GetContextSize										*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Context size.												*
*																				*
*	DESCRIPTION:	Calculates required context size (incudes SDATA and IO).	*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetContextSize(U32 id)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
	PBIMOBJDESC pBimObj;
	U32 size = 0;
	U32 i;

	if (pBimDesc)
	{
		pBimObj = &pBimDesc->ObjDesc[pBimDesc->NumCodeObj];
		for (i = 0; i < pBimDesc->NumDataObj; i++)
		{
			if (pBimObj->Type & (OBJTYPE_IOPARAMS | OBJTYPE_SDATA))
			{
				size += ROUND_UP(BIMOBJSIZE_BYTES * pBimObj->size, BIMOBJALIGNMENT);
			}
			pBimObj++;
		}
	}

	return size;
}

/********************************************************************************
*   NAME:			BIMSVC_CheckObjLoadCollision								*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Number of collisions.										*
*																				*
*	DESCRIPTION:	Checks two sets of memory regions for their interception .	*
*																				*
********************************************************************************/ 

U32 BIMSVC_CheckObjLoadCollision(PSPUOBJDESC pSrcDesc, U32 nSrcDesc, PSPUOBJDESC pDstDesc, U32 nDstDesc)
{
	U32 i, j;
	U32 foundCollision = 0;

	for (i = 0; i < nSrcDesc; i++)
	{
		for (j = 0; j < nDstDesc; j++)
		{
			if ((U16)pSrcDesc->spu_addr < ((U16)pDstDesc[j].spu_addr+(U16)pDstDesc[j].size))
			{
				if (((U16)pSrcDesc->spu_addr+(U16)pSrcDesc->size) > (U16)pDstDesc[j].spu_addr)
					foundCollision++;
			}
		}
		pSrcDesc++;
	}

	return foundCollision;
}

/********************************************************************************
*   NAME:			BIMSVC_SortDesc												*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Resulting number of descriptors.							*
*																				*
*	DESCRIPTION:	Sort descriptors by their address.							*
*																				*
********************************************************************************/ 

void BIMSVC_SortDesc(PSPUOBJDESC pDesc, U32 nDesc)
{
	SPUOBJDESC tmpdesc;
	int i, j;

	for (j = 0; j < nDesc - 1; j++)
	{
		for (i = nDesc - 1; i > j; i--)
		{
			if ((U16)pDesc[i].spu_addr > (U16)pDesc[i-1].spu_addr && 
				(pDesc[i].size & SMC_DMA_FROM_SPU) == (pDesc[i-1].size & SMC_DMA_FROM_SPU))
			{
				tmpdesc = pDesc[i-1];
				pDesc[i-1] = pDesc[i];
				pDesc[i] = tmpdesc;
			}
		}
	}
}

/********************************************************************************
*   NAME:			BIMSVC_MergeDesc											*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Resulting number of descriptors.							*
*																				*
*	DESCRIPTION:	Coalesce adjacent descriptors.								*
*																				*
********************************************************************************/ 

U32 BIMSVC_MergeDesc(PSPUOBJDESC pDesc, U32 nDesc)
{
	U32 size;
	U32 byte_size;
	int i, j;

	for (i = nDesc - 1; i > 0; i--)
	{
		size = pDesc[i].size & SMC_SIZEMASK;
		byte_size = SMC2BCOUNT(size); // convert from SMC words to bytes
		if ((pDesc[i].sdram_addr+byte_size) == pDesc[i-1].sdram_addr && 
			((U16)pDesc[i].spu_addr+size) == (U16)pDesc[i-1].spu_addr &&
			(pDesc[i].size & SMC_DMA_FROM_SPU) == (pDesc[i-1].size & SMC_DMA_FROM_SPU))
		{
			size += pDesc[i-1].size & SMC_SIZEMASK;

			pDesc[i-1].sdram_addr = pDesc[i].sdram_addr;
			pDesc[i-1].spu_addr = pDesc[i].spu_addr;
			pDesc[i-1].size = (pDesc[i-1].size & ~SMC_SIZEMASK) | size;
			j = i;
			nDesc--;
			while (j < nDesc)
			{
				pDesc[j] = pDesc[j+1];
				j++;
			}
		}
	}

	return nDesc;
}

/********************************************************************************
*   NAME:			BIMSVC_GetCodeDesc											*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Number of CODE objects.										*
*																				*
*	DESCRIPTION:	Copies CODE descriptors to caller's buffer.					*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetCodeDesc(U32 id, PSPUOBJDESC pDesc)
{
	PDSPCODECDATADESC pCodeCDataDesc = BimPlugInTable[id].pCodeCDataDesc;
	U32 nDesc = 0;

	if (pCodeCDataDesc)
	{
		nDesc = pCodeCDataDesc->numCodeDesc;
		SFL_memcpy(pDesc, pCodeCDataDesc->Desc, nDesc*sizeof(SPUOBJDESC));
	}
	
	return nDesc;
}

/********************************************************************************
*   NAME:			BIMSVC_GetCDataDesc											*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Number of CDATA objects.									*
*																				*
*	DESCRIPTION:	Copies CDATA descriptors to caller's buffer.				*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetCDataDesc(U32 id, PSPUOBJDESC pDesc)
{
	PDSPCODECDATADESC pCodeCDataDesc = BimPlugInTable[id].pCodeCDataDesc;
	U32 nDesc = 0;

	if (pCodeCDataDesc)
	{
		nDesc = pCodeCDataDesc->numCDataDesc;
		SFL_memcpy(pDesc, 
			&pCodeCDataDesc->Desc[pCodeCDataDesc->numCodeDesc], 
			nDesc*sizeof(SPUOBJDESC));
	}
	
	return nDesc;
}

/********************************************************************************
*   NAME:			BIMSVC_GetIOPARAMSDesc										*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	1 - found IO descriptor.									*
*					0 - IO descriptor not found.								*
*																				*
*	DESCRIPTION:	Copies one IO descriptor to caller's buffer.				*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetIOPARAMSDesc(U32 id, PSPUOBJDESC pDesc, PVOID *p_context)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
	PBIMOBJDESC pBimObj = &pBimDesc->ObjDesc[pBimDesc->NumCodeObj];
	U32 size = 0;
	U32 nDesc = 0;
	U32 i;
	U8 *p_io = (U8 *)(*p_context);

	for (i = 0; i < pBimDesc->NumDataObj; i++)
	{
		if (pBimObj->Type & OBJTYPE_IOPARAMS)
		{
			size = ROUND_UP(BIMOBJSIZE_BYTES * pBimObj->size, BIMOBJALIGNMENT); // size in bytes

			pDesc->sdram_addr = (U32)p_io;
			pDesc->spu_addr = CONV_DRAM_ADDR(pBimObj->spu_addr);
			pDesc->size = BCOUNT2SMC(size); // convert to SMC words

			SFL_memcpy(p_io, (U32 *)FIX_SDRAM_ADDR(pBimDesc, pBimObj), size);
			*p_context = p_io + size;
			nDesc++;
			break;
		}
		pBimObj++;
	}

	return nDesc;
}

/********************************************************************************
*   NAME:			BIMSVC_GetIOSTREAMDesc										*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	1 - found IOSTREAM descriptor.								*
*					0 - IOSTREAM descriptor not found.							*
*																				*
*	DESCRIPTION:	Copies one IOSTREAM descriptor to caller's buffer.			*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetIOSTREAMDesc(U32 id, PSPUOBJDESC pDesc, U16 *p_spu_addr)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
	PBIMOBJDESC pBimObj;
	U32 i;
	U32 nDesc = 0;

	if (pBimDesc)
	{
		pBimObj = &pBimDesc->ObjDesc[pBimDesc->NumCodeObj];
		for (i = 0; i < pBimDesc->NumDataObj; i++)
		{
			if (pBimObj->Type & OBJTYPE_IOSTREAM)
			{
				pDesc->sdram_addr = 0;
				pDesc->spu_addr = CONV_DRAM_ADDR(pBimObj->spu_addr);
				pDesc->size = 0;
				*p_spu_addr = (U16)pDesc->spu_addr;

				nDesc = 1;
				break;
			}
			pBimObj++;
		}
	}

	return nDesc;
}

/********************************************************************************
*   NAME:			BIMSVC_GetIOSPEECHDesc										*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	2 - found IOSPEECH descriptor.								*
*					0 - IOSPEECH descriptor not found.							*
*																				*
*	DESCRIPTION:	Copies two IOSPEECH descriptors to caller's buffer.			*
*																				*
********************************************************************************/ 

U32 BIMSVC_GetIOSPEECHDesc(U32 id, PSPUOBJDESC pDesc, U16 *p_spu_addr)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
	PBIMOBJDESC pBimObj;
	U32 i;
	U32 nDesc = 0;

	if (pBimDesc)
	{
		pBimObj = &pBimDesc->ObjDesc[pBimDesc->NumCodeObj];
		for (i = 0; i < pBimDesc->NumDataObj; i++)
		{
			if (pBimObj->Type & OBJTYPE_IOSPEECH)
			{
				pDesc->sdram_addr = ERAM_BASE_ADDR;
				pDesc->spu_addr = CONV_DRAM_ADDR(pBimObj->spu_addr);
				pDesc->size = 1;
				pDesc++;
				pDesc->sdram_addr = ERAM_BASE_ADDR;
				pDesc->spu_addr = CONV_DRAM_ADDR(pBimObj->spu_addr);
				pDesc->size = 1;

				*p_spu_addr = (U16)pDesc->spu_addr;

				nDesc = 2;
				break;
			}
			pBimObj++;
		}
	}

	return nDesc;
}

/********************************************************************************
*   NAME:			BIMSVC_GetSDataDesc											*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Number of SDATA objects.									*
*																				*
*	DESCRIPTION:	Allocates memory for context and							*
*	            	copies SDATA descriptors to caller's buffer.				*
*																				*
********************************************************************************/ 

int BIMSVC_GetSDataDesc(U32 id, U32 idx, PSPUCONTEXTDESC pContextDescr)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
	PBIMOBJDESC pBimObj;

	if (pBimDesc)
	{
		pBimObj = &pBimDesc->ObjDesc[pBimDesc->NumCodeObj + idx];
		while (idx++ < pBimDesc->NumDataObj)
		{
			if (pBimObj->Type & OBJTYPE_SDATA)
			{
				pContextDescr->p_init_data = (PVOID)FIX_SDRAM_ADDR(pBimDesc, pBimObj);
				pContextDescr->size = ROUND_UP(BIMOBJSIZE_BYTES * pBimObj->size, BIMOBJALIGNMENT);
				pContextDescr->spu_addr = CONV_DRAM_ADDR(pBimObj->spu_addr);

				return idx;
			}
			pBimObj++;
		}
	}

	return -1;
}


BOOL BIMSVC_GetObjDesc(U32 id, U32 obj_idx, PSPUCONTEXTDESC pContextDescr)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
	PBIMOBJDESC pBimObj;

	if ((pBimDesc) && (obj_idx < pBimDesc->NumDataObj + pBimDesc->NumCodeObj))
	{
		pBimObj = &pBimDesc->ObjDesc[obj_idx];

		pContextDescr->p_init_data = (PVOID)FIX_SDRAM_ADDR(pBimDesc, pBimObj);
		pContextDescr->size = ROUND_UP(BIMOBJSIZE_BYTES * pBimObj->size, BIMOBJALIGNMENT);
		pContextDescr->spu_addr = CONV_DRAM_ADDR(pBimObj->spu_addr);
		return TRUE;
	}

	return FALSE;
}

/********************************************************************************
*   NAME:			BIMSVC_DecoderAddr											*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Decoder address.											*
*																				*
*	DESCRIPTION:	Entry point for decoder.									*
*																				*
********************************************************************************/ 

U32 BIMSVC_DecoderAddr(U32 id)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;

	return pBimDesc->DecoderEntryPoint;
}

/********************************************************************************
*   NAME:			BIMSVC_EncoderAddr											*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Encoder address.											*
*																				*
*	DESCRIPTION:	Entry point for encoder.									*
*																				*
********************************************************************************/ 

U32 BIMSVC_EncoderAddr(U32 id)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;

	return pBimDesc->EncoderEntryPoint;
}

/********************************************************************************
*   NAME:			BIMSVC_EntryPoint											*
*																				*
*   USAGE:			None														*
*																				*
*	RETURN VALUE: 	Task Address.												*
*																				*
*	DESCRIPTION:	Entry point for task.										*
*																				*
********************************************************************************/ 

U32 BIMSVC_EntryPoint(U32 id)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;

	return pBimDesc->EntryPoint;
}

U32 BIMSVC_get_num_code_obj(U32 id)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;

	return pBimDesc->NumCodeObj;
}

U32 BIMSVC_get_num_cdata_obj(U32 id)
{
	PBIMDESC pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
	PBIMOBJDESC pBimObj = pBimDesc->ObjDesc + pBimDesc->NumCodeObj;
	U32 num_cdata_obj = 0;
	U32 i;
	
	for (i = 0; i < pBimDesc->NumDataObj; i++, pBimObj++)
	{
		if (pBimObj->Type & OBJTYPE_CDATA)
		{
			num_cdata_obj++;
		}
	}

	return num_cdata_obj;
}

U32 BIMSVC_get_mem_size(U32 id, U32 numTasks)
{
	PBIMDESC pBimDesc;
	PBIMOBJDESC pBimObj;
	U32 mem_size=0;
	U32 i;

	while (numTasks--)
	{
		pBimDesc = (PBIMDESC)BimPlugInTable[id++].pBimDesc;
		pBimObj = pBimDesc->ObjDesc;
		for (i = 0; i < pBimDesc->NumDataObj + pBimDesc->NumCodeObj; i++, pBimObj++)
		{
			if (pBimObj->Type & (OBJTYPE_CCODE | OBJTYPE_CDATA))
				mem_size += ROUND_UP(BIMOBJSIZE_BYTES*pBimObj->size, BIMOBJALIGNMENT);
		}
	}

	return mem_size;
}


U32 BIMSVC_AGC_CDATA_UPDATE()
{
	PDSPCODECDATADESC pCodeCDataDesc,pCodeCDataDesc_WB;
	PBIMDESC pBimDesc,pBimDesc_WB;
	PSPUOBJDESC pDesc,pDesc_WB;
	U8 *pBuffer,*pBuffer_WB;

	
	pCodeCDataDesc = BimPlugInTable[DSPTASKID_AGC].pCodeCDataDesc;
	pCodeCDataDesc_WB = BimPlugInTable[DSPTASKID_AGC_WB].pCodeCDataDesc;
		if (pCodeCDataDesc == NULL || pCodeCDataDesc_WB == NULL )
		{
			return SUCCESS;
		}
	
	pBimDesc = (PBIMDESC)BimPlugInTable[DSPTASKID_AGC].pBimDesc;
	pDesc = &pCodeCDataDesc->Desc[pBimDesc->NumCodeObj];
	pBuffer = (U8*)pDesc->sdram_addr;

	pBimDesc_WB = (PBIMDESC)BimPlugInTable[DSPTASKID_AGC_WB].pBimDesc;
	pDesc_WB = &pCodeCDataDesc_WB->Desc[pBimDesc_WB->NumCodeObj];
	pBuffer_WB = (U8*)pDesc_WB->sdram_addr;
	
	pDevicedesc->AGC_tune_control = (PIOAGC_TUNE_CONTROL)(pBuffer + (XAGCNEWCDATA<<1));
	pDevicedesc->AGC_WB_tune_control = (PIOAGC_TUNE_CONTROL)(pBuffer_WB + (XAGCNEWCDATA<<1));
	Compute_AGC_ControlParams(1,1); // set AGCTUNE_PARAMS and AGC_CONTROL by default for wb agc task
	Compute_AGC_ControlParams(1,0); // set AGCTUNE_PARAMS and AGC_CONTROL by default for nb agc task
	return SUCCESS;
	
}

void BIMSVC_plug_in_tasks_nofail(U32 ID, U32 numTasks)
{
	U32 ret;
	U32 lr=get_link_register();
	ret=BIMSVC_plug_in_tasks(ID, numTasks);
	if(ret!=SUCCESS)
		Alert (ALERT_VALUE_NE|ALERT_CRASH, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, AlertId_BIMSVC_plug_in_tasks_fail, ret,lr);
}


U32 BIMSVC_plug_in_tasks(U32 ID, U32 numTasks)
{
	U8 *pDescPull;
	PDSPCODECDATADESC pCodeCDataDesc;
	PBIMOBJDESC pBimObj;
	PBIMDESC pBimDesc;
	PBIMDESC pHammingBimDesc;
	PSPUOBJDESC pDesc;
	U8 *pBuffer;
	U8 *pSrcBuffer;
	U32 mem_size;
	U32 size;
	U32 SizeArray[34];
	U32 i;
	U32 nDesc;
	PHEAPHEADER hHeap;
	int id;
	extern U32 hamming160[];
	extern U32 hamming240[];
	extern PVOID pEcCDataHamming240;

	if (numTasks > ARRAY_COUNT(SizeArray))
	{
		Alert (ALERT_NO_MEMORY | ALERT_CRASH, ALERT_ACT_CORE_DUMP, ALERT_UNKNOWN_CHANNEL, AlertId_BIMSVC_LocalArraySize, numTasks, ARRAY_COUNT(SizeArray));
	}

	BIMSVC_lock();

	if (BimPlugInTable[ID].pCodeCDataDesc == NULL)
	{
		// calculate memory size required for control structures
		for (i = 0, mem_size = 0; i < numTasks; i++)
		{
			id = ID + i;
			nDesc = BIMSVC_get_num_code_obj(id) + BIMSVC_get_num_cdata_obj(id);
			if (nDesc)
			{
				SizeArray[i] = ROUND_UP((sizeof(DSPCODECDATADESC)+(nDesc-1)*sizeof(SPUOBJDESC)), 8);
				mem_size += SizeArray[i];
			}
			else
				SizeArray[i] = 0;
		}

		if (!mem_size) //not require memory 
		{	BIMSVC_unlock();
			return SUCCESS;
		}
		
		pDescPull = (U8 *)Heap_Alloc(hNcNbGlobalHeap, mem_size);
		if (pDescPull == NULL)
		{
			BIMSVC_unlock();
			return ERR_MALLOC_SDRAM_GLOBAL_HEAP;
		}


		mem_size = BIMSVC_get_mem_size(ID, numTasks);
		// allocate space from ERAM
		hHeap = hGlobalEramHeap;		
		pBuffer = (U8 *) Heap_Alloc_Opt(hHeap, mem_size);		
		if (pBuffer == NULL)
		{

#if (defined(M829_BUILD) && !defined(M823_BUILD))
#ifdef M829_OPT
			hHeap = hGlobalCPramHeap;
			pBuffer = (U8 *) Heap_Alloc_Opt(hHeap, mem_size);
			if (pBuffer == NULL)
#endif
			{
				Heap_Free(hNcNbGlobalHeap, pDescPull);
				BIMSVC_unlock();
				//Alert (ALERT_NO_MEMORY|ALERT_CRASH, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, 0x911, 0,0);	
				return ERR_MALLOC_ERAM_GLOBAL_HEAP;
			}
#else
			hHeap = hNcNbGlobalHeap;
			pBuffer = (U8 *) Heap_Alloc(hHeap, mem_size);
			if (pBuffer == NULL)
			{
				Heap_Free(hNcNbGlobalHeap, pDescPull);
				BIMSVC_unlock();
				return ERR_MALLOC_ERAM_GLOBAL_HEAP;
			}
#endif
		}

		for (i = 0; i < numTasks; i++)
		{
			id = ID + i;
			if (SizeArray[i])
			{
				pCodeCDataDesc = (PDSPCODECDATADESC)pDescPull;
				pCodeCDataDesc->refcount = 0;
				pCodeCDataDesc->pEramBuffer = 0;
				pCodeCDataDesc->numCodeDesc = BIMSVC_get_num_code_obj(id);
				pCodeCDataDesc->numCDataDesc = BIMSVC_get_num_cdata_obj(id);

				BimPlugInTable[id].pCodeCDataDesc = pCodeCDataDesc;
				pDescPull += SizeArray[i];
			}
			else
				BimPlugInTable[id].pCodeCDataDesc = NULL;
		}
		
		BimPlugInTable[ID].pCodeCDataDesc->pEramBuffer = pBuffer;

		id = ID + numTasks;
		while (--id >= (int)ID)
		{
			pCodeCDataDesc = BimPlugInTable[id].pCodeCDataDesc;
			if (pCodeCDataDesc == NULL)
				continue;

			pDesc = pCodeCDataDesc->Desc;
			pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
			pBimObj = pBimDesc->ObjDesc;

			// collect CODE objects
			for (i = 0; i < pCodeCDataDesc->numCodeDesc; i++, pBimObj++)
			{
				size = ROUND_UP(BIMOBJSIZE_BYTES * pBimObj->size, BIMOBJALIGNMENT); // size in bytes

				pDesc->sdram_addr = (U32)pBuffer;
				pDesc->spu_addr = CONV_PRAM_ADDR(pBimObj->spu_addr);
				pDesc->size = BCOUNT2SMC(size);

				pSrcBuffer = (U8 *)FIX_SDRAM_ADDR(pBimDesc, pBimObj);
				SFL_memcpy(pBuffer, pSrcBuffer, size);

				pBuffer += size;
				pDesc++;
			}
		}

		// collect CDATA objects
		id = ID + numTasks;
		while (--id >= (int)ID)
		{
			pCodeCDataDesc = BimPlugInTable[id].pCodeCDataDesc;
			if (pCodeCDataDesc == NULL)
				continue;

			pBimDesc = (PBIMDESC)BimPlugInTable[id].pBimDesc;
			pBimObj = &pBimDesc->ObjDesc[pBimDesc->NumCodeObj];

			pDesc = &pCodeCDataDesc->Desc[pBimDesc->NumCodeObj];

			for (i = 0; i < pBimDesc->NumDataObj; i++, pBimObj++)
			{
				if (!(pBimObj->Type & OBJTYPE_CDATA))
					continue;

				size = ROUND_UP(BIMOBJSIZE_BYTES * pBimObj->size, BIMOBJALIGNMENT); // size in bytes

				pDesc->sdram_addr = (U32)pBuffer;
				pDesc->spu_addr = CONV_DRAM_ADDR(pBimObj->spu_addr);
				pDesc->size = BCOUNT2SMC(size);

				pSrcBuffer = (U8 *)FIX_SDRAM_ADDR(pBimDesc, pBimObj);
				SFL_memcpy(pBuffer, pSrcBuffer, size);

				if (id == DSPTASKID_EC_COMMON)
				{
					hHeap = hGlobalEramHeap;
					pEcCDataHamming240 = Heap_Alloc_Opt(hHeap, size);
					if (pEcCDataHamming240 == NULL)
					{
#if (defined(M829_BUILD) && !defined(M823_BUILD))
#ifdef M829_OPT
						hHeap = hGlobalCPramHeap;
						pEcCDataHamming240 = Heap_Alloc_Opt(hHeap, size);
						if (pEcCDataHamming240 == NULL)
#endif
						{
							BIMSVC_unlock();
							Alert (ALERT_NO_MEMORY|ALERT_CRASH, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, AlertId_BIMSVC_plug_in_tasks_2, 0,0);
							return ERR_MALLOC_ERAM_GLOBAL_HEAP;
						}
#else
						hHeap = hGlobalHeap;
						pEcCDataHamming240 = Heap_Alloc(hHeap, size);
						if (pEcCDataHamming240 == NULL)
						{
							BIMSVC_unlock();
							Alert (ALERT_NO_MEMORY|ALERT_CRASH, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, AlertId_BIMSVC_plug_in_tasks_2, 0,0);	
							return ERR_MALLOC_ERAM_GLOBAL_HEAP;
						}
#endif
					}
					SFL_memcpy(pEcCDataHamming240, pSrcBuffer, size);

					pHammingBimDesc = (PBIMDESC)hamming160;
					if(!(pHammingBimDesc->NumCodeObj == 0 && pHammingBimDesc->NumDataObj == 1))
						Alert (ALERT_VALUE_NE, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, AlertId_BIMSVC_plug_in_tasks_1, 0,0);	
					pSrcBuffer = (U8 *)FIX_SDRAM_ADDR(pHammingBimDesc, pHammingBimDesc->ObjDesc);
					SFL_memcpy(pBuffer, pSrcBuffer, sizeof(U32)*pHammingBimDesc->ObjDesc[0].size);
					pHammingBimDesc = (PBIMDESC)hamming240;
					pSrcBuffer = (U8 *)FIX_SDRAM_ADDR(pHammingBimDesc, pHammingBimDesc->ObjDesc);
					SFL_memcpy(pEcCDataHamming240, pSrcBuffer, sizeof(U32)*pHammingBimDesc->ObjDesc[0].size);
				}
				pBuffer += size;
				pDesc++;
			}
		}
	}

	if(numTasks == NUMDSPRESTASKS)
	{
		BimPlugInTable[DSPTASKID_UTD_ECDTMF].pCodeCDataDesc = BimPlugInTable[DSPTASKID_UTD].pCodeCDataDesc;
		BimPlugInTable[DSPTASKID_DTMFDET_ECDTMF].pCodeCDataDesc = BimPlugInTable[DSPTASKID_DTMFDET].pCodeCDataDesc;
	}

	BimPlugInTable[ID].pCodeCDataDesc->refcount++;

	BIMSVC_unlock();

	return SUCCESS;
}

U32 BIMSVC_unplug_tasks(U32 ID, U32 numTasks)
{
	PVOID p;

	BIMSVC_lock();
	
	if (BimPlugInTable[ID].pCodeCDataDesc)
	{
		if (--BimPlugInTable[ID].pCodeCDataDesc->refcount == 0)
		{
			if (ID == CODECID_DP) {
				p = BimPlugInTable[ID].pCodeCDataDesc->pEramBuffer;
				if (p)
				{
					Heap_Free_Unknown(p);
				}
				Heap_Free(hNcNbGlobalHeap, BimPlugInTable[ID].pCodeCDataDesc);
				BimPlugInTable[ID].pCodeCDataDesc = NULL;
			}
			else {
				if (ID <= DSPTASKID_LAST)
				{ // do not unplug features. This makes code more efficient and easier to work with speech channel template
					BimPlugInTable[ID].pCodeCDataDesc->refcount = 1;
				}
				else
				{
					OVLSVC_RemoveTemplateMatchingCodec(ID);
					p = BimPlugInTable[ID].pCodeCDataDesc->pEramBuffer;
					if (p)
					{
						Heap_Free_Unknown(p);
					}
					Heap_Free(hNcNbGlobalHeap, BimPlugInTable[ID].pCodeCDataDesc);
					BimPlugInTable[ID].pCodeCDataDesc = NULL;
				}
			}
		}

	}

	BIMSVC_unlock();
	
	return SUCCESS;
}

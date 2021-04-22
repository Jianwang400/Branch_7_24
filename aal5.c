/**
*******************************************************************************
*
* @file aal5.c
*
* @brief Implementation of the AAL5 layer
* @author Mindspeed Technologies
*
* COPYRIGHT&copy; 2010 Mindspeed Technologies.
* ALL RIGHTS RESERVED
*
* This is Unpublished Proprietary Source Code of Mindspeed Technologies
*
******************************************************************************/


#ifdef AAL5_SUPPORTED
#include "aal5.h"
#include "scmgr.h"
#include "supervisor.h"
#include "chanlib.h"
#include "tdmdrv.h"
#include "hdl.h"
#include "utillib.h"	// for memset

extern PAAL5_LAYER_CTXT pAAL5DescCtxt;				// Internal chained list of AAL5 clients

#if AAL5_TEST_CODE
static void GhostLayer(void);
#endif
#if AAL5_INTERNAL_LOOPBACK
static ATM_STATUS GhostATMLayer(PFDesc pFDesc, HANDLE Handle);
#endif

static ATM_STATUS AAL5_SendCells_GPSSync(PFDesc pFDesc, HANDLE Handle);
static void AAL5_InitCellRxMonitorVars(PAAL5DESCRIPTOR pAAL5Desc);

#ifndef NDEBUG
static void AAL5_StartConsoleDebug(void);
#endif

/********************************************************************************************
* AAL5 frame related functions
********************************************************************************************/

/****************************************************************************/
/*	Function	:	aal5_frame_enc											*/
/*	Parms		:															*/
/*																			*/
/*	Returns		:	Aal5 frame's size.										*/
/*	Purpose		:	Encapsulate data in an aal5 frame (data + padding +		*/
/*					aal5 footer).											*/
/*																			*/
/****************************************************************************/
static void aal5_frame_enc(AAL5_BUFFER_DESC *TempBufferDesc)
{
	U32 TrailerLength;
	U32 crc;
	U32 total_length;
	U32 length = TempBufferDesc->RFCHeaderLength + TempBufferDesc->BufferLength;
	U8 *p	SET_PTR_BAD(U8 *);
	U32 tempIndex;

	// find required total length expressed in multiple of ATM cell payload size (48 bytes)
	tempIndex = (length + AAL5_TRAILER_SIZE);
	if (tempIndex & 0xf)
		tempIndex = (tempIndex >> 4) + 1;	// adjust to next 16 multiple
	else
		tempIndex = tempIndex >> 4;
	total_length = pAAL5DescCtxt->Table16to48Multiple[tempIndex];

	// Trailer length is padding (x bytes) + trailer (8 bytes)
	TrailerLength = TempBufferDesc->AAL5TrailerLength = total_length - length;
	TempBufferDesc->AAL5FrameLength = total_length;

	// Write the data length in the Trailer (16 bits)
	if(length < 0xff)
	{
		p = &TempBufferDesc->pAAL5Trailer[TrailerLength - 5];
		*p = (U8)length;
	}
	else
	{
		p = &TempBufferDesc->pAAL5Trailer[TrailerLength - 6];
		*p++ = (U8)((length & 0x0000ff00)>>8);
		*p = (U8)(length & 0x000000ff);
	}

#if 1
	// Write the CRC-32 in the Trailer (32 bits)
	#if 0
	{
		U32 *pl	SET_PTR_BAD(U32 *);
		crc = aal5_calc_crcTx(TempBufferDesc->pRFCHeader, total_length);
		pl = (U32 *)&TempBufferDesc->pAAL5Trailer[TrailerLength - 4];
		*pl = crc;
	}
	#else
		crc = ~aal5_calc_crc(TempBufferDesc->pRFCHeader, total_length-4, ~0);
		// Write in byte per byte as the CRC might not be aligned when using the RFC header
		p = &TempBufferDesc->pAAL5Trailer[TrailerLength - 4];
		*p++ = crc>>24;
		*p++ = crc>>16;
		*p++ = crc>> 8;
		*p++ = crc;
	#endif

#else
	// Compute the CRC-32 (WORKS !!!!!!)

	if(TempBufferDesc->RFCHeaderLength)
		crc = ~aal5_calc_crc(TempBufferDesc->pRFCHeader, TempBufferDesc->RFCHeaderLength, ~0);	// CRC-32 of the RFC header
	crc = ~aal5_calc_crc(TempBufferDesc->pCurrentBuffer, TempBufferDesc->BufferLength+TrailerLength-4, ~crc);		// CRC-32 of the payload + AAL5 trailer

	// Write the CRC-32 in the Trailer (32 bits)
	p = &TempBufferDesc->pAAL5Trailer[TrailerLength - 4];
	*p++ = (crc & 0xff000000)>>24;
	*p++ = (crc & 0x00ff0000)>>16;
	*p++ = (crc & 0x0000ff00)>> 8;
	*p++ = (crc & 0x000000ff);
#endif
}

/****************************************************************************/
/*	Function	:	aal5_frame_dec											*/
/*	Parms		:															*/
/*																			*/
/*	Returns		:	Real size of the rebuilded aal5 frame.					*/
/*	Purpose		:	Decapsulate data from an aal5 frame (data + pad +		*/
/*					aal5 footer).											*/
/*																			*/
/****************************************************************************/
static int aal5_frame_dec(AAL5_BUFFER_DESC *TempBufferDesc)
{
	int real_length;
	U32 frame_crc, computed_crc, length;
	U32 *pl	SET_PTR_BAD(U32 *);

	length = TempBufferDesc->BufferLength;

#if PPPoA_EN
	length -= TempBufferDesc->HDLCHeaderLength;
#endif

	// CRC checking
	// Computed_Crc is the Crc for RFC+Payload+Padding+AAL5 Trailer as they are all in the same buffer.
	pl = (U32 *)&TempBufferDesc->pCurrentBuffer[length-4];
#if 1
	frame_crc = *pl;
	*pl = 0;
	computed_crc = aal5_calc_crcRx(TempBufferDesc->pCurrentBuffer, length);
#else
	frame_crc = ((U32)((U8*)pl)[0] << 24) |
				((U32)((U8*)pl)[1] << 16) |
				((U32)((U8*)pl)[2] << 8 ) |
				((U32)((U8*)pl)[3]);
	computed_crc = ~aal5_calc_crc(TempBufferDesc->pCurrentBuffer, length-4, ~0);
#endif
	// If not equal the aal5 frame is corrupted
	if(computed_crc != frame_crc)
	{
		debugPrintf((0, DEBUG_ALWAYS, "Checksum error!\n"));
		return(-1);
	}
	// Find the real length
	// That would be RFC + Data
	real_length = (((int)TempBufferDesc->pCurrentBuffer[length - 6])<<8)|((int)TempBufferDesc->pCurrentBuffer[length - 5]);

	return(real_length);
}

/********************************************************************************************
* AAL5 layer related functions
********************************************************************************************/


/****************************************************************************/
/*	Function	:	AAL5_GetDescriptorFromAAL5								*/
/*	Parms		:	HANDLE AAL5_ID : Client context descriptor.				*/
/*	Returns		:	PAAL5DESCRIPTOR : pointer to the client context.		*/
/*																			*/
/*	Purpose		:	Get the client context from the chained client context.	*/
/*																			*/
/****************************************************************************/
static PAAL5DESCRIPTOR AAL5_GetDescriptorFromAAL5(HANDLE AAL5_ID)
{
	PAAL5DESCRIPTOR pCurrentDescriptor = (PAAL5DESCRIPTOR) AAL5_ID;

	if(pCurrentDescriptor == NULL)
		return NULL;

	if(pCurrentDescriptor->Handle == AAL5_ID)
		return pCurrentDescriptor;

	return NULL;
}


/****************************************************************************/
/*	Function	:	AAL5_IsGPSServiceStarted								*/
/*	Parms		:	None.													*/
/*	Returns		:	BOOL: GPS Service started or not.						*/
/*																			*/
/*	Purpose		:	Get the status of the GPS service.						*/
/*																			*/
/****************************************************************************/
BOOL AAL5_IsGPSServiceStarted(void)
{
	if( (pAAL5DescCtxt) && (pAAL5DescCtxt->AAL5ServiceParam == 1) )
		return TRUE;

	return FALSE;
}


/****************************************************************************/
/*	Function	:	AAL5_IsAAL5Started										*/
/*	Parms		:	None.													*/
/*	Returns		:	BOOL: AAL5 layer started or not.						*/
/*																			*/
/*	Purpose		:	Get the status of the AAL5 layer.						*/
/*																			*/
/****************************************************************************/
BOOL AAL5_IsAAL5Started(void)
{
	if(pAAL5DescCtxt)
		return TRUE;

	return FALSE;
}


/****************************************************************************/
/*	Function	:	AAL5_Init												*/
/*	Parms		:	U16: Start GPS Sync task or not.						*/
/*	Returns		:	BOOL: Success/Failure.									*/
/*																			*/
/*	Purpose		:	Initialize the AAL5 Layer : Memory, Contexts etc...		*/
/*																			*/
/****************************************************************************/
BOOL AAL5_Init(U16 AAL5ServiceParam)
{
	U32 i,j,k;
	HANDLE heap = hNcNbGlobalHeap;

	if((pAAL5DescCtxt) || (AAL5ServiceParam > 1))
		return FALSE;

	debugPrintf((0, DEBUG_MODERATE, "%06d: AAL Init: Size of AAL5 Descriptor is %d bytes\n", KSE_gettime(), sizeof(AAL5DESCRIPTOR)));

	if((pAAL5DescCtxt = (PAAL5_LAYER_CTXT) Heap_Alloc(heap, sizeof(AAL5_LAYER_CTXT))) == NULL)
	{
		Alert( ALERT_NO_MEMORY,
			ALERT_ACT_INDICATION,
			ALERT_UNKNOWN_CHANNEL,
			AlertId_AAL5_Init_1,
			(U32) heap,
			sizeof(AAL5_LAYER_CTXT));
		return FALSE;
	}

	memset(pAAL5DescCtxt, 0, sizeof(AAL5_LAYER_CTXT));
	pAAL5DescCtxt->AAL5ServiceParam = AAL5ServiceParam;

	if(AAL5ServiceParam == 1)
	{	// Start the GPS Sync feature support
		if(Device_EnableATMGPSSync(&pAAL5DescCtxt->gpssynctaskID) == FALSE)
		{	// We had a problem starting the GPS sync feature support
			 Heap_Free(heap, pAAL5DescCtxt);
			 pAAL5DescCtxt = NULL;
			 return FALSE;
		}
	}

	// Set the AAL5 Rx Cell Monitor default values
	pDevicedesc->AAL5MonStartOpTimeOut = 12;	// 12 seconds
	pDevicedesc->AAL5MonNormalOpTimeOut = 5;	// 5 seconds

	// Init the tables that accelerate the AAL5 to Cell conversion
	// - 16to48Multiple to find the size (48 multiple) of an AAL5 frame
	// - Nb48Multiple to give the number of cells of an AAL5 frame
	j = 0; k=0;
	for (i=1; i<=(MAX_16to48_MULTIPLE-1); i+=3)
	{
		j += 48;
		k++;
		pAAL5DescCtxt->Table16to48Multiple[i] = j;
		pAAL5DescCtxt->Table16to48Multiple[i+1] = j;
		pAAL5DescCtxt->Table16to48Multiple[i+2] = j;
		pAAL5DescCtxt->TableNb48Multiple[i] = k;
		pAAL5DescCtxt->TableNb48Multiple[i+1] = k;
		pAAL5DescCtxt->TableNb48Multiple[i+2] = k;
	}

#if AAL5_PROFILING
	HAL_gpio_enable_output(GPIO_3);
	HAL_gpio_set_0(GPIO_3);
#endif

#ifndef NDEBUG
	AAL5_StartConsoleDebug();
#endif

#if AAL5_TEST_CODE
	GhostLayer();
#endif

	pAAL5DescCtxt->Param6Def = 0;

	return TRUE;
}


/****************************************************************************/
/*	Function	:	AAL5_Register											*/
/*	Parms		:															*/
/*	Returns		:	HANDLE : Client context descriptor.						*/
/*																			*/
/*	Purpose		:	Register an AAL5 client									*/
/*																			*/
/****************************************************************************/
HANDLE AAL5_Register(HANDLE pRegStruct)
{
	PAAL5_REG_STRUCT pAAL5Client = (PAAL5_REG_STRUCT)pRegStruct;
	PAAL5DESCRIPTOR pNewDescriptor	SET_PTR_BAD(PAAL5DESCRIPTOR);
	ATM_reg_Client ATMLayerReg;
	HANDLE hHeap	SET_PTR_BAD(HANDLE);

	if (KSE_inqcpuid())
	{
		// Running on ARM1 .. need to call the registration through inter CPU process
		InterCPUProcess((PCCB)pAAL5Client->RefData, AAL5_Register, (HANDLE)pAAL5Client, sizeof(AAL5_REG_STRUCT));
		return((HANDLE)AAL5_REG_ON_ARM0);	// wait for AAL5 de-registration on ARM0
	}
	else
	{	// Running on ARM0 : standard process

		if(!pAAL5DescCtxt)
			return (HANDLE)AAL5_REG_SERVICE_DOWN;
		if(pAAL5DescCtxt->CurrentEnabledDescriptor == MAX_AAL5_CLIENTS)
			return (HANDLE)AAL5_REG_MAX_CLIENTS;

		// Test Only : auto allocate VPI/VCI (allows batch VOPENA command)
		if ((pAAL5Client->VPITx == 0) && (pAAL5Client->VCITx == 0))
 		{
			if (ATM_Get_New_VPIVCI(&pAAL5Client->VPITx, &pAAL5Client->VCITx, FALSE))
				return((HANDLE)AAL5_REG_MAX_CLIENTS);
		}
		if ((pAAL5Client->VPIRx == 0) && (pAAL5Client->VCIRx == 0))
 		{
			if (ATM_Get_New_VPIVCI(&pAAL5Client->VPIRx, &pAAL5Client->VCIRx, TRUE))
				return((HANDLE)AAL5_REG_MAX_CLIENTS);
		}

		if(pAAL5Client->ClientDescriptor <= HDL_DESCRIPTOR_MAX || IsValidVoIPSvcDesc(pAAL5Client->ClientDescriptor))
		{// Alloc the VoIP and HDL client descriptors from cacheable memory
			hHeap = hGlobalHeap;
		}
		else
		{// Alloc the client descriptor from NcNb memory
			hHeap = hNcNbGlobalHeap;
		}

		if((pNewDescriptor = (PAAL5DESCRIPTOR)Heap_Alloc(hHeap, sizeof(AAL5DESCRIPTOR))) == NULL)
		{	Alert (ALERT_NO_MEMORY,
				ALERT_ACT_INDICATION,
				pVoipdesc->chanID,
				AlertId_AAL5_Register_2,
				(U32) hHeap,
				sizeof(AAL5DESCRIPTOR_));
			return (HANDLE)AAL5_REG_ALLOC_CLIENT;
		}

		// Initialize the client descriptor
		memset(pNewDescriptor, 0, sizeof(AAL5DESCRIPTOR));
		pNewDescriptor->hHeap = hHeap;

		ATMLayerReg.pcr = pAAL5Client->PCR;
		ATMLayerReg.vpi_tx = pAAL5Client->VPITx;
		ATMLayerReg.vci_tx = pAAL5Client->VCITx;
		ATMLayerReg.vpi_rx = pAAL5Client->VPIRx;
		ATMLayerReg.vci_rx = pAAL5Client->VCIRx;
		ATMLayerReg.qos = pAAL5Client->QoS;
		ATMLayerReg.tos = pAAL5Client->ToS;
		ATMLayerReg.GPSOffset = pAAL5Client->GPSOffset;	// GPS Sync only
		ATMLayerReg.TxDone = NULL;
		ATMLayerReg.TxDataCB = NULL;
		ATMLayerReg.phy = pAAL5Client->UTPCalendarDay;

		// Use a callback for the dispatch (Rx)
		ATMLayerReg.RxDispatch = AAL5_RxDispatch;
		ATMLayerReg.RxDataCB = (void *)pNewDescriptor;

#if AAL5_INTERNAL_LOOPBACK
		pNewDescriptor->BDescIndex = 1;
#else

		// Check if RT needs to be configured
		if(ATM_register_routing_tags(pAAL5Client->chanID, pAAL5Client->NumRoutingTags, pAAL5Client->RoutingTags) == FALSE)
		{// Problem during ATM layer Routing Tags registration.
			// Free the client descriptor
			Heap_Free(pNewDescriptor->hHeap, pNewDescriptor);
			return (HANDLE)AAL5_REG_ATM_REGISTRATION;
		}

		// Register the VCC with the ATM layer
		if( (pNewDescriptor->ATMLayerHandle = ATM_registration_request((HANDLE)&ATMLayerReg)) == NULL )
		{// Problem during ATM layer registration.
			// Free the client descriptor
			Heap_Free(pNewDescriptor->hHeap, pNewDescriptor);
			return (HANDLE)AAL5_REG_ATM_REGISTRATION;
		}

		pNewDescriptor->BDescIndex = ATM_GetBDescHeader() + 1;
#endif

		if( pAAL5DescCtxt->pFirstAAL5Descriptor == NULL )
		{// Sanity check
			if(pAAL5DescCtxt->pLastAAL5Descriptor != NULL)
			{
				Alert(ALERT_VALUE_NE,
					ALERT_ACT_INDICATION,
					ALERT_UNKNOWN_CHANNEL,
					AlertId_AAL5_Register_1,
					(U32) pAAL5DescCtxt->pLastAAL5Descriptor,
					(U32) NULL);
			}

			pAAL5DescCtxt->pFirstAAL5Descriptor = pNewDescriptor;
		}
		else
			pAAL5DescCtxt->pLastAAL5Descriptor->Next = pNewDescriptor;

		pAAL5DescCtxt->pLastAAL5Descriptor = pNewDescriptor;

		pNewDescriptor->Handle = (HANDLE) pNewDescriptor;
		pNewDescriptor->Next = NULL;
		pNewDescriptor->ClientDescriptor = pAAL5Client->ClientDescriptor;
		pNewDescriptor->chanID = pAAL5Client->chanID;
		pNewDescriptor->VPITx = pAAL5Client->VPITx;
		pNewDescriptor->VCITx = pAAL5Client->VCITx;
		pNewDescriptor->VPIRx = pAAL5Client->VPIRx;
		pNewDescriptor->VCIRx = pAAL5Client->VCIRx;
		pNewDescriptor->QoS = pAAL5Client->QoS;
		pNewDescriptor->ToS = pAAL5Client->ToS;
		pNewDescriptor->PCR= pAAL5Client->PCR;
		pNewDescriptor->UTPCalendarDay = pAAL5Client->UTPCalendarDay;
		pNewDescriptor->GPSOffset = pAAL5Client->GPSOffset;
		pNewDescriptor->ReceiveCallback = pAAL5Client->ReceiveCallback;
		pNewDescriptor->RefData = pAAL5Client->RefData;
		pNewDescriptor->RfcEncapsulationMode = pAAL5Client->RfcEncapsulationMode;
		pNewDescriptor->Include_FCS = pAAL5Client->Include_FCS;
		pNewDescriptor->ProtocolID = pAAL5Client->ProtocolID; 
		pNewDescriptor->InFDescPart = pAAL5Client->InFDescPart;
		pNewDescriptor->OutFDescPart = pAAL5Client->OutFDescPart;
		pNewDescriptor->NumRoutingTags = pAAL5Client->NumRoutingTags;

		if (IsValidVoIPSvcDesc(pNewDescriptor->ClientDescriptor) ||
			(pNewDescriptor->ClientDescriptor == ETHERNET_DESCRIPTOR) ||
			(pNewDescriptor->ClientDescriptor == ATMARP_DESCRIPTOR))
		{// This is a VoIPoAAL5, VoIPoETHoAAL5 or ATMARP client
			pNewDescriptor->FDescPayloadSize = ETH_MAXFRAMESIZE;

			// Set the ATM lower layer (use the "standard" ATM layer)
			pNewDescriptor->SendFrame = ATM_Tx_Request;
			pNewDescriptor->SendFrameHandle = pNewDescriptor->ATMLayerHandle;
		}
		else if(pNewDescriptor->ClientDescriptor <= HDL_DESCRIPTOR_MAX)
		{// This is a HDL client
			pNewDescriptor->FDescPayloadSize = MAX_HDL_INTERNAL_MSG_LEN;

			// Set the ATM lower layer (use the "standard" ATM layer)
			pNewDescriptor->SendFrame = ATM_Tx_Request;
			pNewDescriptor->SendFrameHandle = pNewDescriptor->ATMLayerHandle;
		}
		else
		{// This is a CDMAoAAL5 client
			pNewDescriptor->FDescPayloadSize = ATM_BUFFCELL_SIZE;

			if(pAAL5Client->GPSOffset == ATM_NO_GPS_OFFSET)
			{// Set the ATM lower layer (use the "standard" ATM layer)
				pNewDescriptor->SendFrame = ATM_Tx_Request;
				pNewDescriptor->SendFrameHandle = pNewDescriptor->ATMLayerHandle;
			}
			else
			{// Set the ATM lower layer (use the CDMA GPS sync ATM layer)
				pNewDescriptor->SendFrame = AAL5_SendCells_GPSSync;
				pNewDescriptor->SendFrameHandle = pNewDescriptor->Handle;
			}
		}

#if AAL5_INTERNAL_LOOPBACK
		pNewDescriptor->SendFrame = GhostATMLayer;
		pNewDescriptor->SendFrameHandle = pNewDescriptor->Handle;
#endif

#if _AAL5_STATS_
		// Reset AAL5 Stats
		memset(&pNewDescriptor->AAL5Stats, 0, sizeof(AAL5STATS));

		pAAL5DescCtxt->pStatsClient[AAL5DescToIndex(pAAL5Client->ClientDescriptor)] = &pNewDescriptor->AAL5Stats;
#endif	// _AAL5_STATS_

		// No Rx buffer allocated yet
		memset(&pNewDescriptor->RxBufferDesc, 0, sizeof(AAL5_BUFFER_DESC));
		pNewDescriptor->RxReassemblyFDesc = NULL;

		pAAL5DescCtxt->CurrentEnabledDescriptor++;

		// Thanks to that, the VoIP channels will be able to retrieve the AAL5 handle
		pAAL5DescCtxt->ClientsHandle[AAL5DescToIndex(pAAL5Client->ClientDescriptor)] = pNewDescriptor->Handle;

		// Init Cell Monitor
		AAL5_InitCellRxMonitorVars(pNewDescriptor);
	}
	return pNewDescriptor->Handle;
}


/****************************************************************************/
/*	Function	:	AAL5_Unregister											*/
/*	Parms		:	HANDLE : Client context descriptor.						*/
/*	Returns		:	HANDLE : Success / Failure.								*/
/*																			*/
/*	Purpose		:	Un-register an AAL5 client								*/
/*																			*/
/****************************************************************************/
HANDLE AAL5_Unregister(HANDLE pDeregStruct)
{
	PAAL5_DEREG_STRUCT pAAL5Dereg = (PAAL5_DEREG_STRUCT)pDeregStruct;
	HANDLE ClientID = pAAL5Dereg->hClient;

	if(KSE_inqcpuid())
	{
		// Running on ARM1 .. need to call the de-registration through inter CPU process
		InterCPUProcess((PCCB) pAAL5Dereg->RefClient, AAL5_Unregister, (HANDLE)pAAL5Dereg, sizeof(AAL5_DEREG_STRUCT));
		return((HANDLE)AAL5_DEREG_ON_ARM0);		// wait for AAL5 de-registration on ARM0
	}
	else
	{	// Running on ARM0 : standard process
		PAAL5DESCRIPTOR pCurrentDescriptor;
		PAAL5DESCRIPTOR pPreviousDescriptor;

		// Sanity check
		if(!pAAL5DescCtxt)
			return ((HANDLE)AAL5_DEREG_SERVICE_DOWN);

		pCurrentDescriptor = pAAL5DescCtxt->pFirstAAL5Descriptor;
		pPreviousDescriptor = pCurrentDescriptor;

		if(!pCurrentDescriptor)
		{// Sanity check
			if(pAAL5DescCtxt->CurrentEnabledDescriptor != 0)
			{
				debugPrintf((0, DEBUG_ALWAYS, "%06d: AAL5 Unregister: Error %d in AAL5DescCtxt (%d)\n", KSE_gettime(),
								AAL5_DEREG_NO_REGISTERED_CLIENT, pAAL5DescCtxt->CurrentEnabledDescriptor));
			}

			return ((HANDLE)AAL5_DEREG_NO_REGISTERED_CLIENT);
		}

		// Go through all the AAL5 descriptors to find the one we want to free
		while(pCurrentDescriptor != NULL)
		{
			if(pCurrentDescriptor->Handle == ClientID)
			{// This is the descriptor we want to free
				if(pCurrentDescriptor == pAAL5DescCtxt->pFirstAAL5Descriptor)
				{
					pAAL5DescCtxt->pFirstAAL5Descriptor = pAAL5DescCtxt->pFirstAAL5Descriptor->Next;
					if(pAAL5DescCtxt->pFirstAAL5Descriptor == NULL)
					{// Sanity check
						if(pAAL5DescCtxt->CurrentEnabledDescriptor != 1)
						{
							Alert(ALERT_VALUE_NE,
								ALERT_ACT_INDICATION,
								ALERT_UNKNOWN_CHANNEL,
								AlertId_AAL5_Unregister_1,
								pAAL5DescCtxt->CurrentEnabledDescriptor,
								1);
						}

						pAAL5DescCtxt->pLastAAL5Descriptor = NULL;
					}
				}
				else if(pCurrentDescriptor == pAAL5DescCtxt->pLastAAL5Descriptor)
				{
					pAAL5DescCtxt->pLastAAL5Descriptor = pPreviousDescriptor;
					pPreviousDescriptor->Next = NULL;
				}
				else
					pPreviousDescriptor->Next = pCurrentDescriptor->Next;

#if !AAL5_INTERNAL_LOOPBACK
				if(ATM_deregistration_request(pCurrentDescriptor->ATMLayerHandle) != NULL)
				{
					Alert(ALERT_RSRC_SHUT_DOWN,
						ALERT_ACT_INDICATION,
						ALERT_UNKNOWN_CHANNEL,
						AlertId_AAL5_Unregister_2,
						(U32) pCurrentDescriptor->ATMLayerHandle,
						ALERT_VAL_UNUSED);
				}
#endif

#if _AAL5_STATS_
				pAAL5DescCtxt->pStatsClient[AAL5DescToIndex(pCurrentDescriptor->ClientDescriptor)] = NULL;
#endif
				pAAL5DescCtxt->ClientsHandle[AAL5DescToIndex(pCurrentDescriptor->ClientDescriptor)] = NULL;

				if((pCurrentDescriptor->ClientDescriptor > HDL_DESCRIPTOR_MAX) && ((pCurrentDescriptor->ClientDescriptor-CDMAoAAL5_DESCRIPTOR_START) < MAX_CDMA_CLIENTS)) 
				{// This is a CDMAoAAL5 client
					if( (pCurrentDescriptor->ClientDescriptor-CDMAoAAL5_DESCRIPTOR_START) < MAX_CDMA_CLIENTS)
						pAAL5DescCtxt->CDMAoAAL5ClientDescriptor[(pCurrentDescriptor->ClientDescriptor-CDMAoAAL5_DESCRIPTOR_START)] = FALSE;
				}

				// Free any re-assembly buffer currently in use
				if(pCurrentDescriptor->RxReassemblyFDesc)
				{
					FDESC_Free(pCurrentDescriptor->RxReassemblyFDesc);
					pCurrentDescriptor->RxReassemblyFDesc = NULL;
				}

				// Set this Client as deregistered
				pCurrentDescriptor->Handle = NULL;
				pCurrentDescriptor->Next = NULL;

				// Free the current client descriptor
				Heap_Free(pCurrentDescriptor->hHeap, pCurrentDescriptor);

				pAAL5DescCtxt->CurrentEnabledDescriptor--;

				// Return success
				return NULL;
			}

			pPreviousDescriptor = pCurrentDescriptor;
			pCurrentDescriptor = pCurrentDescriptor->Next;
		}

		// We haven't found the descriptor
		return ((HANDLE)AAL5_DEREG_CLIENT_NOT_FOUND);
	}
}


/****************************************************************************/
/*	Function	:	AAL5_GetHandle											*/
/*	Parms		:	U32 ClientDescriptor : Client descriptor (VoIP, etc...)	*/
/*	Returns		:	HANDLE : AAL5 Handle.									*/
/*																			*/
/*	Purpose		:	This is called by AAL5 clients (VoIP ...) to retrieve	*/
/*					the handle to use when sending packets.					*/
/*																			*/
/****************************************************************************/
HANDLE AAL5_GetHandle(U32 ClientDescriptor)
{
	PAAL5DESCRIPTOR pCurrentDescriptor	SET_PTR_BAD(PAAL5DESCRIPTOR);
	HANDLE AAL5Handle;

	if(!pAAL5DescCtxt)
		return NULL;

	// ClientsHandle map:
	// Array index    Corresponding descriptor
	//    0              0x00      (default)
	//    1-8            0x01-0x08 (HDL)
	//    9-128                    (CDMA)
	//    129            0x81      (Ethernet)
	//    130            0x82      (ATMARP)
	//    131-137        0xC0-0xC6 (VOIP1-VOIP7)
	AAL5Handle = pAAL5DescCtxt->ClientsHandle[AAL5DescToIndex(ClientDescriptor)];

	if( AAL5Handle != NULL )
	{
		pCurrentDescriptor = AAL5_GetDescriptorFromAAL5(AAL5Handle);
		if (pCurrentDescriptor == NULL)		// AAL5_GetDescriptorFromAAL5 can return NULL
			return NULL;
		if (pCurrentDescriptor->ClientDescriptor == ClientDescriptor)
			return AAL5Handle;
	}

	return NULL;
}


/****************************************************************************/
/*	Function	:	AAL5_GetClientDescriptor								*/
/*	Parms		:	U32 ClientType : Client type (CDMA, etc...)				*/
/*	Returns		:	U32 : a client descriptor to be used at registration.	*/
/*																			*/
/*	Purpose		:	This is called by a client before registration to get a	*/
/*					client descriptor. The CD is used during the			*/
/*					registration to determine where the context should		*/
/*					be allocated from (SDRAM or IRAM).						*/
/*																			*/
/****************************************************************************/
U32 AAL5_GetClientDescriptor(U32 ClientType)
{	U32 ClientDescriptor = 0xFFFFFFFF;	// Assume error
	U32 i;
	BOOL SlotFound = FALSE;

	if(!pAAL5DescCtxt)
		return SlotFound;

	switch(ClientType)
	{
		case CDMAoAAL5_ID:
			for(i=0; i<MAX_CDMA_CLIENTS; i++)
			{
				if(pAAL5DescCtxt->CDMAoAAL5ClientDescriptor[i] == FALSE)
				{
					pAAL5DescCtxt->CDMAoAAL5ClientDescriptor[i] = TRUE;
					SlotFound = TRUE;
					break;
				}
			}

			if(SlotFound)
				ClientDescriptor = i + CDMAoAAL5_DESCRIPTOR_START;
			else
			{// Problem....there's no more CD available !
				Alert	(ALERT_NORSRC_GENERAL,
					ALERT_ACT_INDICATION,
					ALERT_UNKNOWN_CHANNEL,
					AlertId_AAL5_GetClientDescriptor_1,
					ALERT_VAL_UNUSED,
					ALERT_VAL_UNUSED);
				return 0xFFFFFFFF;
			}
			break;

		default:	// unknown client type
			Alert	(ALERT_BAD_PARAMETER,
				ALERT_ACT_INDICATION,
				ALERT_UNKNOWN_CHANNEL,
				AlertId_AAL5_GetClientDescriptor_2,
				1 ,	// which parameter
				ClientType);
			return 0;
		}

	return ClientDescriptor;
}


/****************************************************************************/
/*	Function	:	AAL5_ReleaseClientDescriptor							*/
/*	Parms		:	U32 ClientDescriptor: a client descriptor to should		*/
/*					have been used at registration.							*/
/*					U32 ClientType : Client type (CDMA, etc...)				*/
/*	Returns		:	BOOL: Success/Failure.									*/
/*																			*/
/*	Purpose		:															*/
/*																			*/
/****************************************************************************/
BOOL AAL5_ReleaseClientDescriptor(U32 ClientDescriptor, U32 ClientType)
{	BOOL bReturnCode = FALSE;
	U32 Offset;

	switch(ClientType)
	{
		case CDMAoAAL5_ID:
			if(ClientDescriptor < CDMAoAAL5_DESCRIPTOR_START)
				break;	// Problem

			Offset = ClientDescriptor - CDMAoAAL5_DESCRIPTOR_START;

			if(pAAL5DescCtxt->CDMAoAAL5ClientDescriptor[Offset] == TRUE)
				pAAL5DescCtxt->CDMAoAAL5ClientDescriptor[Offset] = FALSE;

			bReturnCode = TRUE;
			break;

		default:	// unknown client type
			Alert(ALERT_BAD_PARAMETER,
				ALERT_ACT_INDICATION,
				ALERT_UNKNOWN_CHANNEL,
				AlertId_AAL5_ReleaseClientDescriptor_1,
				2 ,	// which parameter
				ClientType);

			return FALSE;
		}

	return bReturnCode;
}


/****************************************************************************/
/*	Function	:	AAL5_GetStatistics										*/
/*	Parms		:	HANDLE AAL5_ID : Client context descriptor.				*/
/*	Returns		:	PAAL5STATS : Pointer on stats struc.					*/
/*																			*/
/*	Purpose		:	Get AAL5 layer stats for a given client.				*/
/*																			*/
/****************************************************************************/
PAAL5STATS AAL5_GetStatistics(HANDLE AAL5_ID)
{
	PAAL5DESCRIPTOR pCurrentDescriptor	SET_PTR_BAD(PAAL5DESCRIPTOR);

	if(!pAAL5DescCtxt)
		return NULL;

	if( (pCurrentDescriptor = AAL5_GetDescriptorFromAAL5(AAL5_ID)) == NULL)
		return NULL;

#if _AAL5_STATS_
	return (&pCurrentDescriptor->AAL5Stats);
#else
	return NULL;
#endif
}


/****************************************************************************/
/*	Function	:	AAL5_ResetStatistics									*/
/*	Parms		:	HANDLE AAL5_ID : Client context descriptor.				*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Reset AAL5 layer stats for a given client.				*/
/*																			*/
/****************************************************************************/
BOOL AAL5_ResetStatistics(HANDLE AAL5_ID)
{
	PAAL5DESCRIPTOR pCurrentDescriptor	SET_PTR_BAD(PAAL5DESCRIPTOR);

	if(!pAAL5DescCtxt)
		return FALSE;

	if( (pCurrentDescriptor = AAL5_GetDescriptorFromAAL5(AAL5_ID)) == NULL)
		return FALSE;

#if _AAL5_STATS_
	// Reset AAL5 Stats
	memset(&pCurrentDescriptor->AAL5Stats, 0, sizeof(AAL5STATS));

	return TRUE;
#else
	return FALSE;
#endif
}


/****************************************************************************/
/*	Function	:	AAL5_Encapsulate										*/
/*	Parms		:															*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Encapsulate a packet into an AAL5 frame (RFC + 			*/
/*					data + padding + footer).								*/
/*																			*/
/****************************************************************************/
static void AAL5_Encapsulate(PAAL5DESCRIPTOR pCurrentDescriptor, PFDesc ThisFdesc, AAL5_BUFFER_DESC * TempBufferDesc)
{
#if CHECK_TX_OVERFLOW
	U8 *pEndBuffer = ThisFdesc->Payload + pCurrentDescriptor->FDescPayloadSize;
#endif
	U8 *pTempAligned	SET_PTR_BAD(U8 *);

#if GPIO_AAL5_TX_ENCAP_PROF
	HAL_gpio_set_1(GPIO_3);
#endif

	TempBufferDesc->pCurrentBuffer = ThisFdesc->Payload+ ThisFdesc->Offset;
	TempBufferDesc->BufferLength = ThisFdesc->Length;
	TempBufferDesc->pAAL5Trailer = TempBufferDesc->pCurrentBuffer + TempBufferDesc->BufferLength;

	if(pCurrentDescriptor->FDescPayloadSize == ETH_MAXFRAMESIZE)
	{
		// This pointer might not be D-Word aligned. This is a problem as we use
		// the SFL_zeroinit funtion to initialize the Padding + UUI +CPI.
		pTempAligned = TempBufferDesc->pAAL5Trailer;

		// Set the Trailer to 0 (For UUI and CPI fiels)
		// This was previously done in the aal5_frame_enc function.

		// The SFL_6dwburstzeroinit function writes 24 bytes at a time so the total number of bytes
		// MUST be a 24 bytes (6 D-Word) multiple.
		if (TempBufferDesc->BufferLength == G711_10MS+RTP_IP_UDP_HEADER)	// G711 10 ms
			SFL_6dwburstzeroinit(&pTempAligned[0], &pTempAligned[24]);
		else
		{// So correct the pointer to be D-Word aligned.
			while( ((U32)pTempAligned & 0x03) != 0)
				*(pTempAligned++) = 0;
			SFL_6dwburstzeroinit(&pTempAligned[0], &pTempAligned[72]);
		}
#if 0
		pTempAligned[48] = 0;
#endif
	}
	else if(pCurrentDescriptor->FDescPayloadSize == MAX_HDL_INTERNAL_MSG_LEN)
	{// Handle HDL messages
		pTempAligned = TempBufferDesc->pAAL5Trailer;
		while( ((U32)pTempAligned & 0x03) != 0)
				*(pTempAligned++) = 0;
		SFL_6dwburstzeroinit(&pTempAligned[0], &pTempAligned[72]);
	}
	else
	{// CDMA0AAL5
		// Do not overwrite the buffer (and the next!) with zeroes...
		memset(TempBufferDesc->pAAL5Trailer, 0, (ATM_CELL_PAYLOAD_SIZE - ThisFdesc->Length));
	}

#if PPPoA_EN
	// check if the HDLC header is in the frame
	if( pCurrentDescriptor->HDLCCheck )
	{
		if(AAL5_CheckHDLC(TempBufferDesc))
			ThisFdesc->Offset += 2; // keep the offset updated
	}
#endif

	AAL5_TxProcessRFC(pCurrentDescriptor, TempBufferDesc);
	ThisFdesc->Offset -= TempBufferDesc->RFCHeaderLength; // keep the offset updated

#if GPIO_AAL5_TX_ENCAP_PROF
	HAL_gpio_set_0(GPIO_3);
#endif
#if GPIO_AAL5_TX_CRC_PROF
	HAL_gpio_set_1(GPIO_3);
#endif

	// Add the AAL5 Trailer
	aal5_frame_enc(TempBufferDesc); 

#if GPIO_AAL5_TX_CRC_PROF
	HAL_gpio_set_0(GPIO_3);
#endif

#if CHECK_TX_OVERFLOW
	// Verify a possible buffer overflow (that should never happen as everything SHOULD
	// have been calculated to let enough memory remaining).
	if( (TempBufferDesc->pCurrentBuffer + TempBufferDesc->AAL5FrameLength) > pEndBuffer)
	{
		Alert (ALERT_VALUE_GT,
			ALERT_ACT_INDICATION,
			ThisFdesc->chanID,
			AlertId_AAL5_Encapsulate_1,
			(U32) (TempBufferDesc->pCurrentBuffer + TempBufferDesc->AAL5FrameLength),
			(U32) pEndBuffer);
		return;
	}
#endif
}


/****************************************************************************/
/*	Function	:	AAL5_SegmentAAL5FrameIntoFDesc							*/
/*	Parms		:	AAL5_BUFFER_DESC * : Descriptor of the AAL5 frame		*/
/*	Returns		:	PFDesc : linked chain of FDesc.							*/
/*																			*/
/*	Purpose		:	Segment the AAL5 frame into 48 bytes payload. Put		*/
/*					each payload in a FDesc.								*/
/*																			*/
/****************************************************************************/
static PFDesc AAL5_SegmentAAL5FrameIntoFDesc(	PAAL5DESCRIPTOR pCurrentDescriptor, 
														PFDesc ThisFDesc,
														AAL5_BUFFER_DESC * TempBufferDesc)
{
	PFDesc	pCurrentFDesc	SET_PTR_BAD(PFDesc), 
		pFirstFDesc	SET_PTR_BAD(PFDesc);
	U32	cells;
	U32	BdescIndex = (U32)pCurrentDescriptor->BDescIndex;
	U8	*ThisBptr	SET_PTR_BAD(U8 *);

#if GPIO_AAL5_TX_SAR_PROF
	HAL_gpio_set_1(GPIO_3);
#endif

	cells = pAAL5DescCtxt->TableNb48Multiple[(TempBufferDesc->AAL5FrameLength-1)>>4];

	// that should point to the beginning of the datas (RFC header or payload if no RFC)
	ThisBptr = TempBufferDesc->pRFCHeader;

	if( ((pFirstFDesc = pCurrentFDesc = SFL_alloc_partForFDesc(GlobalTxATMFDescPart,cells,0)) != NULL) && (pFirstFDesc->nFdesc == cells))
	{
		// Put the ATM header in BDesc[0].Bptr (use the FDesc internal buffer)
		// The fields BDesc[0].BPtr & BDesc[0].BControl will be set by the ATM layer

		// Put the ATM payload data in BDesc[1].Bptr
		pCurrentFDesc->BDesc[BdescIndex].BControl = IDMA_BCONTROL_BLAST | ATM_CELL_PAYLOAD_SIZE;
		pCurrentFDesc->BDesc[BdescIndex].BPtr = ThisBptr;
		pCurrentFDesc->Tail->FControl |= IDMA_FCONTROL_FREADY;
		pCurrentFDesc->Fext = 0;
		pCurrentFDesc->chanID = pCurrentDescriptor->chanID;

		// Update the SAR pointer
		ThisBptr += ATM_CELL_PAYLOAD_SIZE;

		while(--cells)
		{
			// Put the ATM header in BDesc[0].Bptr (use the FDesc internal buffer)
			// The fields BDesc[0].BPtr & BDesc[0].BControl will be set by the ATM layer
			pCurrentFDesc = pCurrentFDesc->Next;

			// Put the ATM payload data in BDesc[1].Bptr
			pCurrentFDesc->BDesc[BdescIndex].BControl = IDMA_BCONTROL_BLAST | ATM_CELL_PAYLOAD_SIZE;
			pCurrentFDesc->BDesc[BdescIndex].BPtr = ThisBptr;
			pCurrentFDesc->Fext = 0;
			pCurrentFDesc->chanID = pCurrentDescriptor->chanID;

			// Update the SAR pointer
			ThisBptr += ATM_CELL_PAYLOAD_SIZE;
		}

		//This is the last cell so signal the ATM layer to set PTI to 1 by setting Fext to 1
		pCurrentFDesc->Fext = ATM_REQ_SET_PTI;

		// In this last FDesc, we also should put the address of the "Large" FDesc
		pCurrentFDesc->refFDesc = ThisFDesc;

		ThisFDesc->Next = NULL;	// cut this FDesc from the others

#if GPIO_AAL5_TX_SAR_PROF
		HAL_gpio_set_0(GPIO_3);
#endif
		return pFirstFDesc;
	}
	else
	{
		if(pFirstFDesc)
			SFL_free_partForFDesc(pFirstFDesc);
#if GPIO_AAL5_TX_SAR_PROF
		HAL_gpio_set_0(GPIO_3);
#endif
		return NULL;
	}
}


/****************************************************************************/
/*	Function	:	AAL5_SendFrame											*/
/*	Parms		:	PFDesc : linked chain of FDesc to send.					*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Send packets.											*/
/*																			*/
/****************************************************************************/
void AAL5_SendFrame(PFDesc ThisFdesc)
{
	PAAL5DESCRIPTOR pCurrentDescriptor = (PAAL5DESCRIPTOR)ThisFdesc->RefData;
	AAL5_BUFFER_DESC BufferDesc;
	PFDesc FirstFDescOfChain	SET_PTR_BAD(PFDesc);

	// Verify that the client is not already shut down
	if(pCurrentDescriptor->Handle != pCurrentDescriptor)
	{
		FDESC_FreeTx(ThisFdesc);
		return;
	}

#if GPIO_AAL5_TX_PROFILING
	HAL_gpio_set_1(GPIO_3);
#endif

#if _AAL5_STATS_
	#if _AAL5_AVG_STATS_
	// Update Stats
	CALC_AVG(pCurrentDescriptor->AAL5Stats.TxBytesPerClientFrame, ThisFdesc->Length);
	#endif
#endif

	// Do we really need this assignment? It could be in conflict with Rx part...
	if (pCurrentDescriptor->RfcEncapsulationMode == MPOA_METHOD_LLC_ENCAPS)
	{ // LLC: change on a fly the ProtocolID for the frame which is stored in FrameType
		pCurrentDescriptor->ProtocolID = (ProtocolIndex)ThisFdesc->FrameType;
	}

	if (ThisFdesc->FrameType == PROTID_ETH) // copy Ethernet header from BDesc
		SFL_memcpy(ThisFdesc->Payload + ThisFdesc->Offset, ThisFdesc->BDesc[1].BPtr, ThisFdesc->BDesc[1].BControl);	
	else if ((ThisFdesc->FrameType == PROTID_ICMP) || (ThisFdesc->FrameType == PROTID_ARP))
		{ 
			pCurrentDescriptor->ProtocolID = PROTID_ETH;
		}

	// Encapsulate the aal5 frame
	AAL5_Encapsulate(pCurrentDescriptor, ThisFdesc, &BufferDesc);

	// SAR the cells
	if( (FirstFDescOfChain = AAL5_SegmentAAL5FrameIntoFDesc(pCurrentDescriptor, ThisFdesc, &BufferDesc)) != NULL)
	{// Tranmsit the chained list of Cell Descriptors
#if _AAL5_STATS_
		// Update Stats
		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALTxGoodCells, FirstFDescOfChain->nFdesc);
		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALTxGood, 1);
	#if _AAL5_AVG_STATS_
		CALC_AVG(pCurrentDescriptor->AAL5Stats.TxCellsPerBuffer, FirstFDescOfChain->nFdesc);
		CALC_AVG(pCurrentDescriptor->AAL5Stats.TxBytesPerAAL5Frame, BufferDesc.AAL5FrameLength);
	#endif
#endif
#if GPIO_AAL5_TX_PROFILING
		HAL_gpio_set_0(GPIO_3);
#endif

		// Send cell(s) to lower layer
		pCurrentDescriptor->SendFrame(FirstFDescOfChain, pCurrentDescriptor->SendFrameHandle);
	}
	else
	{// Error during SAR process
#if _AAL5_STATS_
		// Update Stats
		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALTxDiscarded, 1);
		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALTxDiscardedByte, BufferDesc.AAL5FrameLength);
#endif
		// Free the FDesc from the client
		ThisFdesc->Next = NULL;
		FDESC_FreeTx(ThisFdesc);
	}
}

/**
@ingroup ATM_Dispatch
@defgroup AAL5_RxDispatch AAL5_RxDispatch
*/


/** @ingroup AAL5_RxDispatch
This routine is called back by the ATM layer.
It performs the collapse between the ATM layer and the AAL5 layer.


@param[in, out]  pAAL5Context -pointer to AAL5 client context descriptor
@param[in, out] pFDesc - pointer to frame descriptor
@return BOOL : Success/Failure.
*/
BOOL AAL5_RxDispatch(void* pAAL5Context, PFDesc pFDesc)
{
	PAAL5DESCRIPTOR pCurrentDescriptor = (PAAL5DESCRIPTOR)pAAL5Context;
	PFDesc pReassemblyFDesc;
	AAL5_BUFFER_DESC* pRxBufferDesc;
	int Length;
	U8 *pRx48BytePtr;
	BOOL IsLastCell;
	
#if GPIO_AAL5_RX_PROFILING
	HAL_gpio_set_1(GPIO_3);
#endif

	if( pCurrentDescriptor == NULL )
		return FALSE;

	pReassemblyFDesc = pCurrentDescriptor->RxReassemblyFDesc;
	pRxBufferDesc = &pCurrentDescriptor->RxBufferDesc;
	IsLastCell = pFDesc->BDesc[0].BPtr[pCurrentDescriptor->NumRoutingTags + ATM_PTI_BYTE] & ATM_PTI_BIT;

#if _AAL5_STATS_
	ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxGoodCells, 1);
#endif
	// Do we have a re-assembly buffer already in use for this VCC.
	if (!pReassemblyFDesc)
	{//No we don't already have a buffer -> Alloc a FDesc
#if GPIO_AAL5_RX_RFC_PROF
		HAL_gpio_set_1(GPIO_3);
#endif
		if( (pReassemblyFDesc = FDESC_Alloc(pCurrentDescriptor->InFDescPart)) == NULL)
		{
#if _AAL5_STATS_
			// Update Stats
			ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxNoBufferAvail, 1);
			ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxDiscarded, 1);
			ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxDiscardedByte, pFDesc->Length);
#endif
			goto RxEnd;
		}

		pCurrentDescriptor->RxReassemblyFDesc = pReassemblyFDesc;

		pRxBufferDesc->pStartBuffer = pReassemblyFDesc->Payload;
		pRxBufferDesc->pEndBuffer = pRxBufferDesc->pStartBuffer + pCurrentDescriptor->FDescPayloadSize;
		pRxBufferDesc->BufferLength = 0;
#if PPPoA_EN
		pRxBufferDesc->HDLCHeaderLength = 0;
#endif

		pRxBufferDesc->pCurrentBuffer = pRxBufferDesc->pStartBuffer;

			if(AAL5_RxProcessRFC(	pCurrentDescriptor,
									pRxBufferDesc,
									pFDesc->Payload + pFDesc->Offset)
				== FALSE)
			{ 
				FDESC_Free(pReassemblyFDesc);

				// No Rx buffer allocated anymore
				pCurrentDescriptor->RxReassemblyFDesc = NULL;
#if _AAL5_STATS_
				// Update Stats
				ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxInvalidRFC, 1);
				ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxDiscarded, 1);
				ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxDiscardedByte, pFDesc->Length);
#endif
				goto RxEnd;
			}

#if GPIO_AAL5_RX_RFC_PROF
		HAL_gpio_set_0(GPIO_3);
#endif
	}

	// On the receive side, pFDesc->Length should be set by the ATM layer.
	// It will only be the length of the payload (no header length included)
	// It should ALWAYS be 48.
	Length = pFDesc->Length;

	// This pointers is pointing to the data the empty buffer
	pRx48BytePtr = pRxBufferDesc->pCurrentBuffer;

	// Will the new X byte segment fit within the remaining buffer space?
	if ( ((U32)pRx48BytePtr + Length) <= (U32)(pRxBufferDesc->pEndBuffer) )
	{// The new 48 byte data segment will fit within the remaining buffer space.
#if _AAL5_STATS_
//		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxGoodCellsProcessed, 1);
#endif
#if GPIO_AAL5_RX_CPY_PROF
		HAL_gpio_set_1(GPIO_3);
#endif

	 	// TODO: This copy has to be done !
	 	SFL_6dwburstmemcpy(pRx48BytePtr, pFDesc->Payload + pFDesc->Offset, Length);

#if GPIO_AAL5_RX_CPY_PROF
		HAL_gpio_set_0(GPIO_3);
#endif
		// Update the RX buffer 48 byte cell segment pointer and RX buffer length with the value
		// of however much data we were given by ATM in the frame descriptor.
		pRxBufferDesc->pCurrentBuffer += Length;
		pRxBufferDesc->BufferLength += Length;

#if _AAL5_STATS_
		// Keep track of the number of cell that we tried to write in this buffer
		pReassemblyFDesc->Fext++;
#endif

		// Note that the whole AAL5 frame will be in this buffer (that includes RFC+Data+Padding+AAL5 trailer).

		// Is this the last cell of a frame (PTI is set to 1)
		if (IsLastCell)
		{// This was the last cell of the frame
			pRxBufferDesc->pCurrentBuffer = pRxBufferDesc->pStartBuffer;
#if PPPoA_EN
			pRxBufferDesc->pCurrentBuffer += pRxBufferDesc->HDLCHeaderLength;
#endif
#if GPIO_AAL5_RX_CRC_PROF
			HAL_gpio_set_1(GPIO_3);
#endif
			// Check the integrity of the AAL5 frame
			Length = aal5_frame_dec(pRxBufferDesc);

#if GPIO_AAL5_RX_CRC_PROF
			HAL_gpio_set_0(GPIO_3);
#endif
			if(Length != -1)
			{// The Reassembly was succesful. Prepare to send the data to the client.
#if _AAL5_STATS_
				// Update Stats
				ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxGood, 1);
//				ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxGoodByte, pRxBufferDesc->BufferLength);
#endif
				// Set Length field
				pReassemblyFDesc->Length = Length - pRxBufferDesc->RFCHeaderLength;
#if PPPoA_EN
				pReassemblyFDesc->Length += pRxBufferDesc->HDLCHeaderLength;
#endif
				pReassemblyFDesc->Offset += pRxBufferDesc->RFCHeaderLength;

#if GPIO_AAL5_RX_PROFILING
				HAL_gpio_set_0(GPIO_3);
#endif
				// Send the FDesc to the client
				pReassemblyFDesc->RefData = (HANDLE)pCurrentDescriptor->RefData;

				// setup the correct ReceiveCallback now!

				// TODO: Extend to ETHERNET and ATMARP?
				if (IsValidVoIPSvcDesc(pCurrentDescriptor->ClientDescriptor))
				{
					switch (pCurrentDescriptor->ProtocolID)
					{ // note in LLC case the ProtocolID can be changed after each frame
						case PROTID_ETH:
							Ethernet_SpecialDispatch(pReassemblyFDesc, (HANDLE)pCurrentDescriptor);
							break;
						case PROTID_IPV4:
							IPv4_Dispatch(pReassemblyFDesc, pCurrentDescriptor);
							break;
						case PROTID_ATMARP: 
							// RefData contains Service Descriptor ID
							ATMARP_Dispatch(pReassemblyFDesc, pCurrentDescriptor);
							break;
						case PROTID_RAW:
							if (pDevicedesc->SpecialPkt_ExtendedFormat & SPECIALPKT_HANDLING_EXT_FORMAT)
								IP_SpecialPacket_Forward(pReassemblyFDesc, pCurrentDescriptor);
							else
								FDESC_Free(pReassemblyFDesc);
							break;
						default:
							// we are not supposed to receive another packets.
							Alert(ALERT_VALUE_NE,
								ALERT_ACT_INDICATION,
								ALERT_UNKNOWN_CHANNEL,
								AlertId_AAL5_RxDispatch_1,
								pCurrentDescriptor->ProtocolID,
								ALERT_VAL_UNUSED);
							FDESC_Free(pReassemblyFDesc);
							break;
					}
				}
				else if (pCurrentDescriptor->ClientDescriptor <= HDL_DESCRIPTOR_MAX)
				{
					if((U32)pCurrentDescriptor->ReceiveCallback==(U32)IPv4_HDL_NoEncap_Dispatch)
					{//we are in packet mode - send to host everything
						IPv4_HDL_NoEncap_Dispatch(pReassemblyFDesc,NULL);
					}
					else
					{//dispatch according to protocol
						switch (pCurrentDescriptor->ProtocolID)
						{ // note in LLC case the ProtocolID can be changed after each frame
							case PROTID_ETH: 
       	       	        		Ethernet_HDL_SpecialDispatch(pReassemblyFDesc);
							    break;
							case PROTID_IPV4:
								IPv4_HDL_PreChannelDispatch(pReassemblyFDesc, pCurrentDescriptor);
								break;
							case PROTID_ATMARP:
								ATMARP_Dispatch(pReassemblyFDesc, pCurrentDescriptor);
								break;
							case PROTID_RAW:
								if (pDevicedesc->SpecialPkt_ExtendedFormat & SPECIALPKT_HANDLING_EXT_FORMAT)
									IP_SpecialPacket_Forward(pReassemblyFDesc, pCurrentDescriptor);
								else
									FDESC_Free(pReassemblyFDesc);
								break;
							default:
								// we are not supposed to receive another packets.
								Alert(ALERT_VALUE_NE,
									ALERT_ACT_INDICATION,
									ALERT_UNKNOWN_CHANNEL,
									AlertId_AAL5_RxDispatch_2,
									pCurrentDescriptor->ProtocolID,
									ALERT_VAL_UNUSED);
								FDESC_Free(pReassemblyFDesc);
								break;
						}
					}
				}
				else
				{
					// Because now all unknown packets are passed by AAL5_RxProcessRFC
					// we must catch them to avoid errors
					if (pCurrentDescriptor->ProtocolID == PROTID_RAW)
					{
						if (pDevicedesc->SpecialPkt_ExtendedFormat & SPECIALPKT_HANDLING_EXT_FORMAT)
							IP_SpecialPacket_Forward(pReassemblyFDesc, pCurrentDescriptor);
						else
							FDESC_Free(pReassemblyFDesc);
					}
					else
					{
						// otherwise it can be:
						// a. CDMAoAAL5
						// b. VCMux for another service descriptors - the upper layer can't be changed during frames processing
						pCurrentDescriptor->ReceiveCallback(pReassemblyFDesc, pCurrentDescriptor);
					}
				}
			}
			else	// >>> Length == -1
			{// The Reassembly was NOT succesful, free the buffer.
#if _AAL5_STATS_
				// Update Stats
				ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxInvalidCRC, 1);
				ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxDiscarded, pReassemblyFDesc->Fext);
				ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxDiscardedByte, pRxBufferDesc->BufferLength);
#endif
				FDESC_Free(pReassemblyFDesc);
			}
			// No Rx buffer allocated anymore
			pCurrentDescriptor->RxReassemblyFDesc = NULL;
		}	// >>> IsLastCell()
	}
	else
	{// The RX frame has exceeded the length of the reassembly buffer so flush the current reassembled frame.
#if _AAL5_STATS_
		// Update Stats
		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxInvalidLen, 1);
		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxDiscarded, pReassemblyFDesc->Fext+1);
		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.AALRxDiscardedByte, pRxBufferDesc->BufferLength);
#endif
		// Free Rx buffer
		FDESC_Free(pReassemblyFDesc);
		pCurrentDescriptor->RxReassemblyFDesc = NULL;
	}

RxEnd:
	FDESC_Free(pFDesc); 
	return TRUE;
}


/****************************************************************************/
/*	Function	:	AAL5_SendCells_GPSSync									*/
/*	Parms		:	PFDesc : linked chain of FDesc to send.					*/
/*					HANDLE : Client context descriptor.						*/
/*	Returns		:	ATM_STATUS : success/Failure.							*/
/*																			*/
/*	Purpose		:															*/
/*																			*/
/****************************************************************************/
static ATM_STATUS AAL5_SendCells_GPSSync(PFDesc pFDesc, HANDLE Handle)
{
	PAAL5DESCRIPTOR pCurrentDescriptor = (PAAL5DESCRIPTOR)Handle;

	if(ATM_Tx_Request_GPSsync(pFDesc, pCurrentDescriptor->ATMLayerHandle) == ATM_SUCCESS)
		return ATM_SUCCESS;
	else
	{
#if _AAL5_STATS_
		// Update Stats (Even if we discard the frame due to Overrun, do not increment the AALTxDiscardedByte counter
		ADD_TO_CTR(pCurrentDescriptor->AAL5Stats.TDM2ATMOverrun, 1);	// Overrun !
#endif
		FDESC_FreeTx(pFDesc);
	}

	// No cell to send
	return ATM_ERROR;
}


/********************************************************************************************
* AAL5 layer API functions
********************************************************************************************/

/****************************************************************************/
/*	Function	:	AAL5_QueryAAL5_SERVICE_CONFIG							*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles AAL5_SERVICE_CONFIG query API.		*/
/*																			*/
/****************************************************************************/
U8 AAL5_QueryAAL5_SERVICE_CONFIG(U16 * Payload)
{	U8 Length = 0;

	if(pAAL5DescCtxt)
	{
		*Payload++ = pAAL5DescCtxt->AAL5ServiceParam;
		Length = 2;
	}

	return Length;
}

/****************************************************************************/
/*	Function	:	AAL5_HandleAAL5_SERVICE_CONFIG							*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles AAL5_SERVICE_CONFIG API command.	*/
/*																			*/
/****************************************************************************/
U16 AAL5_HandleAAL5_SERVICE_CONFIG(U16 *p, U16 Length)
{	U16 AAL5ServiceParam;

	if (Length != 2)
 		return CMDDAT_CNF_ERROR_AAL5OPT_MSG_LEN;

	AAL5ServiceParam = *p++;

	if(AAL5_Init(AAL5ServiceParam))
	{
		pDevicedesc->ServiceConfig |= SVC_AAL5;
		return (U16) CMDDAT_CNF_OK;
	}
	else
 		return (U16) CMDDAT_CNF_ERROR_AAL5OPT_INIT;
}


/****************************************************************************/
/*	Function	:	AAL5_HandleAAL5OPT										*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles the AAL5_OPT API command.			*/
/*																			*/
/****************************************************************************/
U16 AAL5_HandleAAL5OPT(U16 *p, U16 Length) 
{
	HANDLE AAL5LayerHandle	SET_PTR_BAD(HANDLE);
	AAL5_REG_STRUCT AAL5Client;
	U16 QoS;
	U16 RFCEncap;
	U16 OnOff, Param6Def;
	U16 errnum;
	U32 busID, timeslot;
	U16 ts_in_bus, ts_hdw;
	extern TDMCONFIG TdmConfig[TDMBUSNUM];

	static int once = 0;

	if(Length != 14)
		return CMDDAT_CNF_ERROR_AAL5OPT_MSG_LEN;

	// we MUST be on ARM0....we should be check anyway
	if( KSE_inqcpuid() || (!pAAL5DescCtxt) )
 		return CMDDAT_CNF_ERROR_AAL5OPT_ARM0;

	OnOff = *p++;
	AAL5Client.ClientDescriptor = (U32)(OnOff & AAL5OPT_API_DESCRIPTOR);	// service descriptor from the Host

	pAAL5DescCtxt->Param6Def = Param6Def = OnOff & AAL5OPT_API_PARAM6_OLDNEW; // parameter 6 definition: 0 - old, 1 - new

	OnOff = OnOff & AAL5OPT_API_ONOFF;

	if(OnOff)
	{
		AAL5Client.VPITx = *p++;
		AAL5Client.VCITx = *p++;

		if(pAAL5DescCtxt->SetRxIdentifiers)
		{// Use different identifiers for Rx & Tx
			AAL5Client.VPIRx = pAAL5DescCtxt->VPIRx;
			AAL5Client.VCIRx = pAAL5DescCtxt->VCIRx;
			pAAL5DescCtxt->SetRxIdentifiers = FALSE;
		}
		else
		{// Use the same identifiers for Rx & Tx
			AAL5Client.VPIRx = AAL5Client.VPITx;
			AAL5Client.VCIRx = AAL5Client.VCITx;

			// Save Rx connection identifiers
			pAAL5DescCtxt->VPIRx = AAL5Client.VPITx;
			pAAL5DescCtxt->VCIRx = AAL5Client.VCITx;
		}

		QoS = *p++;

		if (Param6Def)
		{ // new definition - like for Monet
			if (QoS < 4)
				AAL5Client.QoS = (ATM_QoS) QoS;
			else
				return (CMDDAT_CNF_ERROR_AAL5_INVALID_QOS);
		}
		else
		{ // old definition
			if(QoS == 0xFFFF)
				AAL5Client.QoS = ATM_MAX_PCR;
			else
				AAL5Client.QoS = (ATM_QoS) QoS;
		}

		AAL5Client.PCR = *p++;

		RFCEncap = *p++;

		if (Param6Def)
		{ // setup the encaps mode, FCS presence and protocol ID for the new definition
			AAL5Client.RfcEncapsulationMode = RFCEncap & AAL5OPT_API_LLC_METHOD;
			if (RFCEncap & AAL5OPT_API_INCLUDE_FCS) // important in LLC bridged encaps only
				AAL5Client.Include_FCS = TRUE;
			else
				AAL5Client.Include_FCS = FALSE;

			AAL5Client.ProtocolID = (RFCEncap & AAL5OPT_API_PROTOCOLID) >> 8; // protocol index for VCMux mode

			if (AAL5Client.RfcEncapsulationMode == MPOA_METHOD_VC_MUX)
			{ //check if correct protocol was specified for VCMux service descriptor
				if ((AAL5Client.ClientDescriptor == ETHERNET_DESCRIPTOR) && (AAL5Client.ProtocolID != PROTID_ETH))
					return (CMDDAT_CNF_ERROR_AAL5_INVALID_PROTOCOL);
				if ((AAL5Client.ClientDescriptor == ATMARP_DESCRIPTOR) && (AAL5Client.ProtocolID != PROTID_ATMARP))
					return (CMDDAT_CNF_ERROR_AAL5_INVALID_PROTOCOL);
			}
			else // check - LLC is not enabled for Ethernet and ATMARP service descriptors
				if ((AAL5Client.ClientDescriptor == ATMARP_DESCRIPTOR) || (AAL5Client.ClientDescriptor == ETHERNET_DESCRIPTOR))
					return (CMDDAT_CNF_ERROR_AAL5OPT_AAL5_REG_RFC_ENCAP);
		}
		else
		{
			// if we use old param6 definition => convert to the new format
			if(((RFCEncap & HW_IO_INCLUDE_FCS_MASK) != 0) && ((RFCEncap & HW_IO_ENCAP_MASK) == HW_IO_ENCAPSULATION_BIPOA_LLC))
				AAL5Client.Include_FCS = TRUE; // if bit 5 of Param 6 is set and encaps is LLC bridged (i.e. 0x12)
			else
				AAL5Client.Include_FCS = FALSE;	

			if (RFCEncap == HW_IO_ENCAPSULATION_NONE)
			{ // when no encaps => no RFC header, process by IPv4
				AAL5Client.Include_FCS = FALSE;
				AAL5Client.ProtocolID = PROTID_IPV4;	
				AAL5Client.RfcEncapsulationMode = MPOA_METHOD_VC_MUX;
				AAL5Client.ReceiveCallback = IPv4_Dispatch;
			}
			else
			{
				switch(RFCEncap & HW_IO_ENCAP_MASK)
				{
					case HW_IO_ENCAPSULATION_RIPOA_VCMUX:
					case HW_IO_ENCAPSULATION_BIPOA_NONE:
					case HW_IO_ENCAPSULATION_RIPOA_NONE:
					// assume that if old param6 definition is used => new param definition can't be used at the same time
						AAL5Client.ProtocolID = PROTID_IPV4;	
						AAL5Client.RfcEncapsulationMode = MPOA_METHOD_VC_MUX;
						AAL5Client.ReceiveCallback = IPv4_Dispatch;
						break;
					case HW_IO_ENCAPSULATION_BIPOA_VCMUX:
						AAL5Client.ProtocolID = PROTID_ETH;	
						AAL5Client.RfcEncapsulationMode = MPOA_METHOD_VC_MUX;
						AAL5Client.ReceiveCallback = Ethernet_SpecialDispatch;
						break;

					case HW_IO_ENCAPSULATION_BIPOA_LLC:
					case HW_IO_ENCAPSULATION_RIPOA_LLC:
						AAL5Client.RfcEncapsulationMode = MPOA_METHOD_LLC_ENCAPS;
						break;
					default:
					// Not supported
						return CMDDAT_CNF_ERROR_AAL5OPT_AAL5_REG_RFC_ENCAP;
				} 
			}

		}
		
		AAL5Client.UTPCalendarDay = *p++;

		// we need to set up chanID and dispatch function and check the availability of service descriptor
		if (IsValidVoIPSvcDesc(AAL5Client.ClientDescriptor))
		{

			if (Param6Def && (AAL5Client.RfcEncapsulationMode == MPOA_METHOD_LLC_ENCAPS))
			{ 
				// if we use a new param6 and want to init SD with LLC encaps we must be sure 
				// that ATMARP and ETHERNET SDs were not registered
		 		if(AAL5_GetHandle(ETHERNET_DESCRIPTOR))
					return CMDDAT_CNF_ERROR_AAL5_DESCRIPTOR_UNAVAILABLE;
		 		if(AAL5_GetHandle(ATMARP_DESCRIPTOR))
					return CMDDAT_CNF_ERROR_AAL5_DESCRIPTOR_UNAVAILABLE;
			}

			AAL5Client.chanID = NUM_CHANS;

			if (Param6Def)
				AAL5Client.ReceiveCallback = IPv4_Dispatch; // in VCMux mode;in LLC mode it is changed basing on each Rx frame
		}
		else 
			if(AAL5Client.ClientDescriptor > HDL_DESCRIPTOR_MAX)
		{
			// if old parameter definition => we don't have Ethernet and ATMARP service descriptors
			if (!Param6Def) return CMDDAT_CNF_ERROR_AAL5OPT_CLIENT_DESCRIPTOR;

			// we use new param6 and register Eth or ATMARP SD. We must be sure that VOIP service 
			// descriptor is not registered or registered in VCMux mode to carry IPv4
	 		AAL5LayerHandle = AAL5_GetHandle(VOIP_DESCRIPTOR);

			if (AAL5LayerHandle != NULL)
			{
				PAAL5DESCRIPTOR pCurrentDescriptor = AAL5_GetDescriptorFromAAL5(AAL5LayerHandle);
				if (pCurrentDescriptor == NULL)
					return CMDDAT_CNF_ERROR_AAL5_DESCRIPTOR_UNAVAILABLE;
					
				if((pCurrentDescriptor->ClientDescriptor == VOIP_DESCRIPTOR) && (pCurrentDescriptor->RfcEncapsulationMode == MPOA_METHOD_LLC_ENCAPS))
					return CMDDAT_CNF_ERROR_AAL5_DESCRIPTOR_UNAVAILABLE;
			}
			
			if (AAL5Client.ClientDescriptor == ETHERNET_DESCRIPTOR)
			{
				AAL5Client.chanID = NUM_CHANS-MAX_HDL_CLIENTS-1;
				AAL5Client.ReceiveCallback = Ethernet_SpecialDispatch;
			}
			else if (AAL5Client.ClientDescriptor == ATMARP_DESCRIPTOR)
				{
					AAL5Client.chanID = NUM_CHANS-MAX_HDL_CLIENTS-2;
					AAL5Client.ReceiveCallback = ATMARP_Dispatch; 
				}
			}
		
		if (IsValidVoIPSvcDesc(AAL5Client.ClientDescriptor) || (AAL5Client.ClientDescriptor == ETHERNET_DESCRIPTOR) || (AAL5Client.ClientDescriptor == ATMARP_DESCRIPTOR))
		{// VOIP/ATMARP client
			AAL5Client.ToS = ATM_AAL5_VOICE;
			AAL5Client.GPSOffset = ATM_NO_GPS_OFFSET;
			AAL5Client.RefData = AAL5Client.ClientDescriptor;
			AAL5Client.InFDescPart = GlobalRxETHFDescPart;
			AAL5Client.OutFDescPart = GlobalTxETHFDescPart;
			AAL5Client.NumRoutingTags = pAAL5DescCtxt->NumRoutingTags[AAL5DescToTag(AAL5Client.ClientDescriptor)];
			if(AAL5Client.NumRoutingTags)
				SFL_memcpy(AAL5Client.RoutingTags, pAAL5DescCtxt->RoutingTags[AAL5DescToTag(AAL5Client.ClientDescriptor)], AAL5Client.NumRoutingTags);

			if (!once) 
			{
				once = 1;
			// For VoIPoAAL5, only enable TDM bus(es) in ERAM.
			TDMDRV_disable_bus(0);
			TDMDRV_disable_bus(1);
			TDMDRV_disable_bus(2);
			TDMDRV_disable_bus(3);
			TDMDRV_enable_bus(0, hGlobalEramHeap);
			if (!ISMIRODEVICE())
			{
				// On Picasso, we need to enable one extra bus, but depending on the TDM config,
				// it could be bus 1 or bus 2.
				timeslot = (TdmConfig[0].realTPF <= NUM_TS_PER_FRAME)? TdmConfig[0].realTPF : NUM_TS_PER_FRAME;
				busID = GetBusID(timeslot, &ts_in_bus, &ts_hdw);
				TDMDRV_enable_bus(busID, hGlobalEramHeap);
			}
		}
		}
		else if(AAL5Client.ClientDescriptor <= HDL_DESCRIPTOR_MAX)
		{// HDL channels
			// check if the partition is created
			// the paritition is created as "on-demand"
			if(!pDevicedesc->HDL_part_defined)
			{
				if( !HDL_part_init(&errnum) )	//can't initialize the HDL part, give up.
					return errnum;
#ifdef hnDebug
				HDL_start_console_debug();
#endif
			}

			AAL5Client.chanID = (NUM_CHANS-AAL5Client.ClientDescriptor); 
			AAL5Client.ToS = ATM_AAL5_VOICE;		//have to do this because of low layer handling
			AAL5Client.GPSOffset = ATM_NO_GPS_OFFSET;
			AAL5Client.ReceiveCallback = IPv4_HDL_PreChannelDispatch;
			AAL5Client.ProtocolID = PROTID_IPV4;	
			AAL5Client.RefData = AAL5Client.ClientDescriptor;
			AAL5Client.InFDescPart = GlobalAAL5PCIFDescPart;
			AAL5Client.OutFDescPart = GlobalAAL5PCIFDescPart;
			AAL5Client.NumRoutingTags = pAAL5DescCtxt->HDL_NumRoutingTags[AAL5Client.ClientDescriptor-1];
			if(AAL5Client.NumRoutingTags)
				SFL_memcpy(AAL5Client.RoutingTags, pAAL5DescCtxt->HDL_RoutingTags[AAL5Client.ClientDescriptor-1], AAL5Client.NumRoutingTags);
		}
		else
		{// Invalid Client Descriptor
			return CMDDAT_CNF_ERROR_AAL5OPT_CLIENT_DESCRIPTOR;
		}

		AAL5LayerHandle = AAL5_Register((HANDLE)&AAL5Client);
		if((U32)AAL5LayerHandle < AAL5_REG_ERROR)
			return (CMDDAT_CNF_ERROR_AAL5OPT_AAL5_REG + (U32)AAL5LayerHandle);

		// This is not possible, but check anyway!
		if((U32)AAL5LayerHandle == AAL5_REG_ON_ARM0)
			Alert (ALERT_VALUE_EQ, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, AlertId_AAL5HandleAAL5OPT_1, (U32) AAL5LayerHandle,(U32) AAL5_REG_ON_ARM0);

		return CMDDAT_CNF_OK;
	}
	else
	{
		AAL5_DEREG_STRUCT AAL5Dereg;
		HANDLE AAL5DeregErrorCode = NULL;

		AAL5LayerHandle = AAL5_GetHandle(AAL5Client.ClientDescriptor);
		if(AAL5LayerHandle == NULL)
			return CMDDAT_CNF_ERROR_AAL5OPT_CLIENT_DESCRIPTOR;

		if((AAL5Client.ClientDescriptor <= HDL_DESCRIPTOR_MAX) || (Param6Def &&
			(IsValidVoIPSvcDesc(AAL5Client.ClientDescriptor) ||
			(AAL5Client.ClientDescriptor == ETHERNET_DESCRIPTOR) ||
			(AAL5Client.ClientDescriptor == ATMARP_DESCRIPTOR))))
		{
			AAL5Dereg.hClient = AAL5LayerHandle;
			AAL5Dereg.RefClient = NULL;

			AAL5DeregErrorCode = AAL5_Unregister((HANDLE)&AAL5Dereg);

			if(AAL5DeregErrorCode == NULL)
				return CMDDAT_CNF_OK;
			else
				return (CMDDAT_CNF_ERROR_AAL5OPT_AAL5_DEREG + ((U32)AAL5DeregErrorCode- AAL5_DEREG_SERVICE_DOWN));
		}
		else
		{// Invalid Client Descriptor
			return CMDDAT_CNF_ERROR_AAL5OPT_CLIENT_DESCRIPTOR;
		}
	}
}


/****************************************************************************/
/*	Function	:	AAL5_QueryAAL5OPT											*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles AAL5_OPT query API (for Service Descriptor 0)	*/
/*																			*/
/****************************************************************************/
U8 AAL5_QueryAAL5OPT(U16 * Payload) 
{	
	HANDLE AAL5Handle = AAL5_GetHandle(VOIP_DESCRIPTOR);
	PAAL5DESCRIPTOR pCurrentDescriptor = NULL;

	if (AAL5Handle == NULL)
		return 0;

	pCurrentDescriptor= AAL5_GetDescriptorFromAAL5(AAL5Handle);

	if (pCurrentDescriptor == NULL)
		return 0;

	*Payload++ = 0x0000 | 1 << 8 | VOIP_DESCRIPTOR | pAAL5DescCtxt->Param6Def;		
	*Payload++ = pCurrentDescriptor->VPITx;						
	*Payload++ = pCurrentDescriptor->VCITx;						

	if(pCurrentDescriptor->QoS == ATM_MAX_PCR)
		*Payload++ = 0xFFFF;							
	else
		*Payload++ = pCurrentDescriptor->QoS;					

	*Payload++ = pCurrentDescriptor->PCR;						

	if (pAAL5DescCtxt->Param6Def)
	{ // if new param6 definition is used
		if(pCurrentDescriptor->Include_FCS == TRUE)
			*Payload++ = pCurrentDescriptor->RfcEncapsulationMode | (pCurrentDescriptor->ProtocolID << 8) | MPOA_FCS_PRESERVED;
		else
			*Payload++ = pCurrentDescriptor->RfcEncapsulationMode | (pCurrentDescriptor->ProtocolID << 8);
	}
	else
	{ // we need to convert the encaps+protocolID to correct encapsulation value (old style).
		// 0x02 - Bridged LLC
		// 0x03 - Routed IP LLC
		// 0x04 - Bridged VCMux
		// 0x05 - Routed IP VCMux
		U16 Encaps = (2-pCurrentDescriptor->RfcEncapsulationMode)*2;

		if (pCurrentDescriptor->ProtocolID == PROTID_IPV4)
			Encaps += 1;
		if(pCurrentDescriptor->Include_FCS == TRUE)
			*Payload++ = Encaps | HW_IO_INCLUDE_FCS_MASK;
		else
			*Payload++ = Encaps;		
	}

	*Payload++ = pCurrentDescriptor->UTPCalendarDay;				

	return 12;
}


/****************************************************************************/
/*	Function	:	AAL5_HandleSET_AAL5_ROUTING_TAGS						*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles the SET_AAL5_ROUTING_TAGS			*/
/*					command. This is for VoIPoAAL5 only.					*/
/*																			*/
/****************************************************************************/
U16 AAL5_HandleSET_AAL5_ROUTING_TAGS(U16 *p, U16 Length)
{
	U32 ClientDescriptor;

	// Tag indices:
	// [0] == AAL5_VOIP_DESC0,  0x00 (VOIP_DESCRIPTOR)
	// [1] == AAL5_VOIP_DESC1,  0xC0
	// [2] == AAL5_VOIP_DESC2,  0xC1
	// [3] == AAL5_VOIP_DESC3,  0xC2
	// [4] == AAL5_VOIP_DESC4,  0xC3
	// [5] == AAL5_VOIP_DESC5,  0xC4
	// [6] == AAL5_VOIP_DESC6,  0xC5
	// [7] == AAL5_VOIP_DESC7,  0xC6
	// [8] == AAL5_ETH_DESC,    0x81 (ETHERNET_DESCRIPTOR)
	// [9] == AAL5_ATMARP_DESC, 0x82 (ATMARP_DESCRIPTOR
	
	if( (Length > (MAX_ROUTING_TAGS+2)) || (!pAAL5DescCtxt) )
 		return CMDDAT_CNF_ERROR_AAL5OPT_MSG_LEN;

	// Can not configure if AAL5_OPTIONS already configured
	ClientDescriptor = (U32)(*p++ & AAL5OPT_API_DESCRIPTOR);
	Length -= 2;

	if(AAL5_GetHandle(ClientDescriptor) )
 		return CMDDAT_CNF_ERROR_AAL5OPT_DESCRIPTOR;

	if (IsHDLDescriptor(ClientDescriptor))
	{	//HDL-related descriptors
		SFL_memcpy(pAAL5DescCtxt->HDL_RoutingTags[ClientDescriptor-1], (U8*)p, Length);
		pAAL5DescCtxt->HDL_NumRoutingTags[ClientDescriptor-1]= Length;
		return CMDDAT_CNF_OK;
	}
	else if (IsValidVoIPSvcDesc(ClientDescriptor) || ClientDescriptor == AAL5_ETH_DESC || ClientDescriptor == AAL5_ATMARP_DESC)
	{
		SFL_memcpy(pAAL5DescCtxt->RoutingTags[AAL5DescToTag(ClientDescriptor)], (U8*)p, Length);
		pAAL5DescCtxt->NumRoutingTags[AAL5DescToTag(ClientDescriptor)] = Length;
		return CMDDAT_CNF_OK;
	}

	return CMDDAT_CNF_ERROR_AAL5_DESCRIPTOR_UNAVAILABLE;
}


/****************************************************************************/
/*	Function	:	AAL5_QuerySET_AAL5_ROUTING_TAGS							*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles SET_AAL5_ROUTING_TAGS query.		*/
/*					This is for VoIPoAAL5 only.								*/
/*																			*/
/****************************************************************************/
U8 AAL5_QuerySET_AAL5_ROUTING_TAGS(U16 *Payload)
{
	if(!pAAL5DescCtxt)
		return 0;

	// TODO: we should be able to query routing tags for every descriptor
	// TODO: but this must be defined in NFD
	*Payload++ = VOIP_DESCRIPTOR;
	SFL_memcpy((U8*)Payload, pAAL5DescCtxt->RoutingTags[AAL5DescToTag(VOIP_DESCRIPTOR)], pAAL5DescCtxt->NumRoutingTags[AAL5DescToTag(VOIP_DESCRIPTOR)]);

	return (pAAL5DescCtxt->NumRoutingTags[AAL5DescToTag(VOIP_DESCRIPTOR)]+2);
}


/****************************************************************************/
/*	Function	:	AAL5_HandleSET_AAL5_RX_PARAMS							*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles the SET_AAL5_RX_PARAMS				*/
/*					command. This is for VoIPoAAL5 only.					*/
/*																			*/
/****************************************************************************/
U16 AAL5_HandleSET_AAL5_RX_PARAMS(U16 *p, U16 Length)
{
	U32 ClientDescriptor;

	if( (Length != 6) || (!pAAL5DescCtxt) )
 		return CMDDAT_CNF_ERROR_AAL5OPT_MSG_LEN;

	// Can not configure if AAL5_OPTIONS already configured
	ClientDescriptor = (U32)(*p++ & AAL5OPT_API_DESCRIPTOR);
	if ( (!IsValidVoIPSvcDesc(ClientDescriptor)) || AAL5_GetHandle(ClientDescriptor) )
 		return CMDDAT_CNF_ERROR_AAL5OPT_DESCRIPTOR;

	pAAL5DescCtxt->VPIRx = *p++;
	pAAL5DescCtxt->VCIRx = *p++;
	pAAL5DescCtxt->SetRxIdentifiers = TRUE;

	return CMDDAT_CNF_OK;
}


/****************************************************************************/
/*	Function	:	AAL5_QuerySET_AAL5_RX_PARAMS							*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles SET_AAL5_RX_PARAMS query.			*/
/*					This is for VoIPoAAL5 only.								*/
/*																			*/
/****************************************************************************/
U8 AAL5_QuerySET_AAL5_RX_PARAMS(U16 *Payload)
{
	if(!pAAL5DescCtxt)
		return 0;

	*Payload++ = VOIP_DESCRIPTOR;
	*Payload++ = pAAL5DescCtxt->VPIRx;
	*Payload++ = pAAL5DescCtxt->VCIRx;

	return 6;
}


/****************************************************************************/
/*	Function	:	AAL5_HandleAAL5_STATISTICS								*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles AAL5_SERVICE_CONFIG API command.	*/
/*																			*/
/****************************************************************************/
U16 AAL5_HandleAAL5_STATISTICS(U16 *AckPayload, U16 * p, U16 FuncCode)
{
	HANDLE AAL5Handle	SET_PTR_BAD(HANDLE);
	PAAL5STATS	pAAL5Stat	SET_PTR_BAD(PAAL5STATS);
	U32 * pAckPayload = (U32 *) AckPayload;
	U16 BytesCounter = 0;

	AAL5Handle = AAL5_GetHandle(*p & AAL5OPT_API_DESCRIPTOR);

	if(AAL5Handle == NULL)
		return 0;

	if( (FuncCode != STATS_READ_ONLY) && (FuncCode != STATS_READ_AND_RESET) )
		return 0;

	pAAL5Stat = AAL5_GetStatistics(AAL5Handle);

	if(pAAL5Stat == NULL)
		return 0;

	*pAckPayload++ = AAL5_STATS_REVISION;			BytesCounter += 4;
	*pAckPayload++ = *p++;						BytesCounter += 4; // Copy Sevice Descriptor from payload to ack
	*pAckPayload++ = pAAL5Stat->AALTxGood;			BytesCounter += 4;
//	*pAckPayload++ = pAAL5Stat->AALTxGoodByte;		BytesCounter += 4;
	*pAckPayload++ = pAAL5Stat->AALTxDiscarded;		BytesCounter += 4;
	*pAckPayload++ = pAAL5Stat->AALTxDiscardedByte;	BytesCounter += 4;
	*pAckPayload++ = pAAL5Stat->AALRxGood;			BytesCounter += 4;
//	*pAckPayload++ = pAAL5Stat->AALRxGoodByte;		BytesCounter += 4;
	*pAckPayload++ = pAAL5Stat->AALRxDiscarded;		BytesCounter += 4;
	*pAckPayload++ = pAAL5Stat->AALRxDiscardedByte;	BytesCounter += 4;
	*pAckPayload++ = pAAL5Stat->AALRxInvalidLen;		BytesCounter += 4;
	*pAckPayload++ = pAAL5Stat->AALRxInvalidCRC;		BytesCounter += 4;
	*pAckPayload++ = pAAL5Stat->AALRxNoBufferAvail;	BytesCounter += 4;

	if(FuncCode == STATS_READ_AND_RESET)
		AAL5_ResetStatistics(AAL5Handle);

	return BytesCounter;
}


/****************************************************************************/
/*	Function	:	AAL5_HandleAAL5PKTDETOPT								*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles the AAL5_PKT_DET_OPT command.		*/
/*																			*/
/****************************************************************************/
U16 AAL5_HandleAAL5PKTDETOPT(U16 *p, U16 Length)
{
	if (Length != 2)
 		return CMDDAT_CNF_ERROR_AAL5OPT_MSG_LEN;

	if(!pDevicedesc)
 		return CMDDAT_CNF_ERROR_AAL5OPT_DEVICEDESC;

	pDevicedesc->AAL5MonStartOpTimeOut = (U8)((*p) >> 8);
	pDevicedesc->AAL5MonNormalOpTimeOut = (U8)(*p);

	return CMDDAT_CNF_OK;
}


/****************************************************************************/
/*	Function	:	AAL5_QueryAAL5PKTDETOPT									*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine handles AAL5_PKT_DET_OPT query.			*/
/*																			*/
/****************************************************************************/
U8 AAL5_QueryAAL5PKTDETOPT(U16 *Payload)
{
	*Payload = ((U16)pDevicedesc->AAL5MonStartOpTimeOut << 8) | 
				pDevicedesc->AAL5MonNormalOpTimeOut;

	return 2;
}


/****************************************************************************/
/*	Function	:	AAL5_InitCellRxMonitorVars								*/
/*	Parms		:															*/
/*	Returns		:															*/
/*																			*/
/*	Purpose		:	This routine initializes the AAL5 packet level status	*/
/*	                monitoring variables. It is called when a new AAL5		*/
/*					client is registered.									*/
/*																			*/
/****************************************************************************/
static void AAL5_InitCellRxMonitorVars(PAAL5DESCRIPTOR pAAL5Desc)
{
	// same as ARM0 time
	pAAL5Desc->AAL5MonStartTime = KSE_gettime();

	// AAL5 packet level status monitoring variables
	if (pDevicedesc->AAL5MonStartOpTimeOut)
		pAAL5Desc->AAL5MonCheckStartOp = TRUE;
	else
		pAAL5Desc->AAL5MonCheckStartOp = FALSE;

	if (pDevicedesc->AAL5MonNormalOpTimeOut)
		pAAL5Desc->AAL5MonCheckNormalOp = TRUE;
	else
		pAAL5Desc->AAL5MonCheckNormalOp = FALSE;

	pAAL5Desc->AAL5MonLastCellCount = 0;
	pAAL5Desc->AAL5MonLastCellTime = 0;
}


/****************************************************************************/
/*	Function	:	AAL5_CellRxMonitor										*/
/*	Parms		:															*/
/*	Returns		:	Returns 0xFFFF or payload parameter. If 0xFFFF is		*/
/*					returned, don't send indicatior to host.				*/
/*																			*/
/*	Purpose		:	This function monitors AAL5 cell rx counters and		*/
/*					returns indicators for AAL5 packet level status			*/
/*					monitoring. Indicators are returned for the following	*/
/*					conditions:												*/
/*					(1) At start of operation, cell receieved within		*/
/*						time-out period AAL5MonStartOpTimeOut.				*/
/*					(2) At start of operation, no cells received within		*/
/*						time-out period AAL5MonStartOpTimeOut.				*/
/*					(3) After start of operation, no cells received within	*/
/*						time-out period AAL5MonNormalOpTimeOut.				*/
/*																			*/
/****************************************************************************/
U16 AAL5_CellRxMonitor(HANDLE AAL5Handle)
{
	PAAL5DESCRIPTOR pAAL5Desc	SET_PTR_BAD(PAAL5DESCRIPTOR);
	PAAL5STATS	pAAL5Stat	SET_PTR_BAD(PAAL5STATS);
	U16 Param1 = 0xFFFF;
	U32 EllapsedTime;
	U32 TimeMS = KSE_gettime();

	// get pointer to AAL5 descriptor
	if(!pAAL5DescCtxt)
		return Param1;

#if !_AAL5_STATS_
	// No stats available
	return Param1;
#endif

	pAAL5Desc = AAL5_GetDescriptorFromAAL5(AAL5Handle);
	if(pAAL5Desc == NULL)
		return Param1;

	// get pointer to AAL5 stats structure
	pAAL5Stat = &(pAAL5Desc->AAL5Stats);

	if (pAAL5Desc->AAL5MonCheckStartOp)
	{
		// check start of operation timeout 
		EllapsedTime = TimeMS - pAAL5Desc->AAL5MonStartTime;
		if ((pAAL5Stat->AALRxGoodCells > 0) && (EllapsedTime < pDevicedesc->AAL5MonStartOpTimeOut * 1000))
		{
			// received cell within timeout period
			Param1 = 0x0003;
			pAAL5Desc->AAL5MonLastCellCount = pAAL5Stat->AALRxGoodCells;
			pAAL5Desc->AAL5MonLastCellTime = TimeMS;
			pAAL5Desc->AAL5MonCheckStartOp = FALSE;
		}

		if (EllapsedTime >= pDevicedesc->AAL5MonStartOpTimeOut * 1000)
		{
			// timeout period expired
			Param1 = 0x0002;
			pAAL5Desc->AAL5MonCheckStartOp = FALSE;
		}
	}
	else
	{
		// check normal operation timeout
		if (pDevicedesc->AAL5MonNormalOpTimeOut)
		{
			if (pAAL5Stat->AALRxGoodCells != pAAL5Desc->AAL5MonLastCellCount)
			{
				pAAL5Desc->AAL5MonLastCellCount = pAAL5Stat->AALRxGoodCells;
				pAAL5Desc->AAL5MonLastCellTime = TimeMS;
				pAAL5Desc->AAL5MonCheckNormalOp = TRUE;
			}
			else if ((pAAL5Stat->AALRxGoodCells > 0) && (pAAL5Desc->AAL5MonCheckNormalOp))
			{
				// at least 1 cell as been received and
				// rx cell count has not changed and we have not sent an indication
				EllapsedTime = TimeMS - pAAL5Desc->AAL5MonLastCellTime;
				if (EllapsedTime > (pDevicedesc->AAL5MonNormalOpTimeOut * 1000))
				{
					// send indication to host
					Param1 = 0x0001;
					pAAL5Desc->AAL5MonCheckNormalOp = FALSE;
				}
			}
		}
	}

	return Param1;
}


/********************************************************************************************
* AAL5 layer TESTS functions
********************************************************************************************/

#if AAL5_INTERNAL_LOOPBACK
/****************************************************************************/
/*	Function	:	GhostATMLayer											*/
/****************************************************************************/
ATM_STATUS GhostATMLayer(PFDesc pFDesc, HANDLE Handle)
{
	PFDesc tempFDesc = pFDesc;
	PAAL5DESCRIPTOR pDescriptor = (PAAL5DESCRIPTOR)Handle;
	U32 ATMOffset = 6 + pDescriptor->NumRoutingTags;

	if( Handle == NULL )
		return ATM_ERROR;

	if(pDescriptor->BDescIndex != 1)
	{
		Alert (ALERT_VALUE_NE,
			ALERT_ACT_INDICATION,
			pFDesc->chanID,
			AlertId_GhostATMLayer_1,
			pDescriptor->BDescIndex,
			1);

		return ATM_ERROR;
	}

	while(tempFDesc != NULL)
	{
		// the second BPtr should have the IDMA_BCONTROL_BLAST
		if(!(tempFDesc->BDesc[1].BControl & IDMA_BCONTROL_BLAST))
		{
			int i = 0;
			while(i==0);
		}

		tempFDesc->Offset = ATMOffset + ((16 - ATMOffset)%sizeof(U32));
		tempFDesc->BDesc[0].BPtr = tempFDesc->Payload;
		tempFDesc->Length = (tempFDesc->BDesc[1].BControl & IDMA_BCONTROL_BLEN_MASK);

		if(tempFDesc->Length != ATM_CELL_PAYLOAD_SIZE)
			debugPrintf((0, DEBUG_ALWAYS, "PROBLEM\n"));

		memset(tempFDesc->Payload, 0x00, tempFDesc->Offset);
		SFL_memcpy(tempFDesc->Payload + tempFDesc->Offset,
						tempFDesc->BDesc[1].BPtr,
						tempFDesc->Length);

		tempFDesc->BDesc[1].BPtr = NULL;
		tempFDesc->BDesc[1].BControl = 0;

		if(tempFDesc->Fext == ATM_REQ_SET_PTI)
			tempFDesc->BDesc[0].BPtr[ATM_PTI_BYTE] |= ATM_PTI_BIT;

		if(tempFDesc->refFDesc != NULL)
		{
			FDESC_Free(tempFDesc->refFDesc);
			tempFDesc->refFDesc = NULL;
			tempFDesc->fpart = NULL;
		}

		tempFDesc = tempFDesc->Next;
	}

	AAL5_RxDispatch(Handle, pFDesc);

	return ATM_SUCCESS;
}
#endif

#if AAL5_TEST_CODE
static U32 Ghosttaskstk0[TASKSTKSZ/4]; 
static TASK GhostTaskID;

#define MAX_TEST_REPETITION		5000

V32 TestNumber;
V32 LoopCounter;
HANDLE GhostHandle;

U32 TxTagNumber = 0;			// This is to tag the Transmit Cells
U32 RxTagNumber = 0;			// This is to compare with the TxTag

U32 TransmittedCells = 0;		// Counter of transmitted cells (or frame)
U32 ReceivedCells = 0;			// Counter of received cells (or frame)
U32 WrongCells = 0;				// Counter of bad cells (or frame)

U32 CheckCounter = 0;			// Counter to periodically call the CheckOptiphy function

static U32 GhostVci = ATM_START_VCI_USER+3;	// Start value

#define GHOST_FRAME_SIZE		RTP_IP_UDP_HEADER + G711_20MS
U8	GhostFrame[GHOST_FRAME_SIZE];

volatile BOOL StopTx = FALSE;		// Stop the transmission

#if 1
/****************************************************************************/
/*	Function	:	GhostCallback											*/
/*	Parms		:	PFDesc : linked chain of FDesc.							*/
/*					U32 : Ref Data.											*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Test only. This function will compare the frame sent by	*/
/*					the ghost task with the one received here.				*/
/*																			*/
/****************************************************************************/
void GhostCallback(PFDesc ThisFDesc, U32 GhostData)
{	U32 i, j;
	U32 Length = ThisFDesc->Length;
	U8 *pRxGhostFrame = ThisFDesc->Payload + ThisFDesc->Offset;

	if(Length != GHOST_FRAME_SIZE)
	{
		j=0;
		while(j==0);

		FDESC_Free(ThisFDesc);
		return; // there's already a problem
	}

	for(i = 1; i < GHOST_FRAME_SIZE; i++)
	{
		if(pRxGhostFrame[i] != GhostFrame[i])
		{
			j=0;
			while(j==0);

			FDESC_Free(ThisFDesc);
			return;
		}
	}

	if(ThisFDesc->Next != NULL)
	{
		j=0;
		while(j==0);

		FDESC_Free(ThisFDesc);
		return;	// D'oh
	}

	pRxGhostFrame = ThisFDesc->Payload + ThisFDesc->Offset;

	// Test to see if we received the expected frame
	if(pRxGhostFrame[0] != RxTagNumber)
	{// The frame we received isn't the one we expected
		WrongCells++;
		debugPrintf((0, DEBUG_ALWAYS, "AAL5 Test: Wrong cell detected %d\n\n", WrongCells));
		RxTagNumber = ++pRxGhostFrame[0]; // expected next TagNumber
	}
	else
		RxTagNumber++; // expected next TagNumber

	if(RxTagNumber == 0xFF)
		RxTagNumber = 0;

	// Here it's more a ReceivedPackets than ReceivedCells....
	ReceivedCells++;

	FDESC_Free(ThisFDesc);
}


/****************************************************************************/
/*	Function	:	GhostTask												*/
/*	Parms		:	None.													*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Test only. This task wil register with the AAL5 layer	*/
/*					and then send a frame to it.							*/
/*																			*/
/****************************************************************************/
void GhostTask(void)
{
	AAL5_REG_STRUCT AAL5Client;
	PFDesc GhostFDesc	SET_PTR_BAD(PFDesc);
	U32 DefTagIdx = AAL5DescToTag(VOIP_DESCRIPTOR);

#define BURST_SIZE	45

	KS_delay(SELFTASK, 10);

	pAAL5DescCtxt->NumRoutingTags[DefTagIdx] = 0;
	pAAL5DescCtxt->RoutingTags[DefTagIdx][0] = 0x01;
	pAAL5DescCtxt->RoutingTags[DefTagIdx][1] = 0x02;

	AAL5Client.ClientDescriptor = VOIP_DESCRIPTOR;
	AAL5Client.chanID = NUM_CHANS;
	AAL5Client.VPITx = AAL5Client.VPIRx = 0;
	AAL5Client.VCITx = AAL5Client.VCIRx = GhostVci++;
	AAL5Client.UTPCalendarDay = 0;
	AAL5Client.QoS = ATM_CBR;
	AAL5Client.PCR = 20000;
	AAL5Client.ToS = ATM_AAL5_VOICE;
	AAL5Client.GPSOffset = ATM_NO_GPS_OFFSET;
	AAL5Client.RfcEncapsulationMode = HW_IO_ENCAPSULATION_NONE;
	AAL5Client.Include_FCS = FALSE;
	AAL5Client.ReceiveCallback = GhostCallback;
	AAL5Client.RefData = VOIP_DESCRIPTOR;
	AAL5Client.InFDescPart = GlobalRxETHFDescPart;
	AAL5Client.OutFDescPart = GlobalTxETHFDescPart;
	AAL5Client.NumRoutingTags = pAAL5DescCtxt->NumRoutingTags[DefTagIdx];
	if(AAL5Client.NumRoutingTags)
		SFL_memcpy(AAL5Client.RoutingTags, pAAL5DescCtxt->RoutingTags[DefTagIdx], AAL5Client.NumRoutingTags);

	if( (GhostHandle = AAL5_Register((HANDLE)&AAL5Client)) == NULL)
		Alert (ALERT_STATE_GENERAL, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, AlertId_AAL5GhostTask_1, (U32) &AAL5Client,0);

	if( GhostHandle != AAL5_GetHandle(AAL5Client.ClientDescriptor))
		Alert (ALERT_VALUE_NE, ALERT_ACT_INDICATION, ALERT_UNKNOWN_CHANNEL, AlertId_AAL5GhostTask_2, (U32) GhostHandle, (U32)AAL5Client.ClientDescriptor);

	debugPrintf((0, DEBUG_ALWAYS, "\n\n%06d: AAL5 Test Code: Start Test\n\n", KSE_gettime()));

	while(TRUE)
	{// Data loopback test
		while(TestNumber < MAX_TEST_REPETITION)
		{
			// This code creates 1 frame and then sends it before creating another one

			GhostFDesc = FDESC_Alloc(GlobalTxETHFDescPart);

			//GhostFDesc->Offset = ETH_BUFFSIZE_REAL - (MAX_AAL5_TRAILER_SIZE + GHOST_FRAME_SIZE);
			GhostFDesc->Offset = MAX_HEADER_SIZE;

			// Try to have the beginning of the data aligned on a DWORD...that will speed up the CRC32 calculation
			while( (((U32)(GhostFDesc->Payload + GhostFDesc->Offset)) & 3) != 0)
				GhostFDesc->Offset--;

			GhostFDesc->Length = GHOST_FRAME_SIZE;

			// Put a Tag number into the frame so we can check if we receive all the frame on Rx
			GhostFrame[0] = (TxTagNumber++%255);

			SFL_memcpy(GhostFDesc->Payload + GhostFDesc->Offset, GhostFrame, GHOST_FRAME_SIZE);

			// Here it's more a TransmittedPackets than TransmittedCells....
			TransmittedCells++;

			GhostFDesc->RefData = GhostHandle;
			AAL5_SendFrame(GhostFDesc);

			if(!(TransmittedCells%BURST_SIZE))
				KS_delay(SELFTASK, 5);

			TestNumber++;

			// If we need to stop the Tx, Put StopTx to 1
			while(StopTx)
				KS_delay(SELFTASK, 1000);

			// Every 300 * x ms check the status of the Optiphy
			if(++CheckCounter == 300)
			{
				CheckCounter = 0;
				CheckOptiPHY(PORT_3);
			}
		}

		LoopCounter++;
		TestNumber = 0;
	}
}

#else
/****************************************************************************/
/*	Function	:	GhostRxATMLayer											*/
/*	Parms		:	void * pThis : Pointer to context.						*/
/*					PFDesc pRxFDesc : Received cells.						*/
/*	Returns		:	BOOL.													*/
/*																			*/
/*	Purpose		:	Test only. This task wil receive cells dierctly from	*/
/*					the ATM layer and check for the pattern inserted		*/
/*					on the receive side. This also make sure that there's 	*/
/*					no data corruption along the path.						*/
/*					NOTE : This does not use AAL5.							*/
/*																			*/
/****************************************************************************/
BOOL GhostRxATMLayer(void * pThis, PFDesc pRxFDesc)
{
	PFDesc tempFDesc = pRxFDesc;
	U32 i, Client;
	U8* pRxPayload	SET_PTR_BAD(U8 *);

	if(pThis == (void *)0xaabbccdd)
		Client = 0;
	else
		Client = 1;

	while(tempFDesc != NULL)
		{ReceivedCells++;

		 pRxPayload = tempFDesc->Payload + tempFDesc->Offset;

		 // this is ATM so we should have 48 bytes payloads
		 if(tempFDesc->Length != ATM_CELL_PAYLOAD_SIZE)
			{
				Alert	(ALERT_VALUE_NE,
					ALERT_ACT_INDICATION,
					pRxFDesc->chanID,
					AlertId_GhostRxATMLayer_1,
					tempFDesc->Length ,
					ATM_CELL_PAYLOAD_SIZE );
				return FALSE;
			}

		 // Test to see if we received the expected frame
		 for(i=0; i<ATM_CELL_PAYLOAD_SIZE; i++)
		 	{if(pRxPayload[i] != RxTagNumber)
		 		{// This byte was wrong...mark the cell as wrong (corrupted or missing???)
		 		 if(i!=0)
		 		 	{// payload corruption
					 Alert (ALERT_VALUE_NE,
						ALERT_ACT_INDICATION,
						pRxFDesc->chanID,
						AlertId_GhostRxATMLayer_2,
						i ,
						0 );
					 return FALSE;
		 		 	}
		 		 RxTagNumber = ++pRxPayload[0];
		 		 WrongCells++;
		 		 break;
		 		}
		 	}

		 // Update the expected TagNmber
		 if(RxTagNumber == pRxPayload[0])
		 	RxTagNumber++;

		 if(RxTagNumber == 0xFF)
		 	RxTagNumber = 0;

		 tempFDesc = tempFDesc->Next;
		}

	FDESC_Free(pRxFDesc);
	return TRUE;
}


/****************************************************************************/
/*	Function	:	GhostTaskATMLayer										*/
/*	Parms		:	None.													*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Test only. This task wil register with the ATM layer	*/
/*					and then send multiple FDesc to it.						*/
/*					Each FDesc contains a cell with a particular pattern	*/
/*					that can be traced on the receive side.					*/
/*					NOTE : This does not use AAL5.							*/
/*																			*/
/****************************************************************************/
void GhostTaskATMLayer(void)
{
	ATM_reg_Client ATMLayerReg;
	HANDLE ATMLayerHandle;
	PFDesc	FirstGhostFDesc		SET_PTR_BAD(PFDesc), 
		CurrentGhostFDesc	SET_PTR_BAD(PFDesc), 
		PreviousGhostFDesc	SET_PTR_BAD(PFDesc);
	int Client = 0;
	U32 BDescIndex = 0;

	ATMLayerReg.pcr = 20000;
	ATMLayerReg.vpi_tx = ATMLayerReg.vpi_rx = 0;
	ATMLayerReg.vci_tx = ATMLayerReg.vci_rx = GhostVci;
	ATMLayerReg.qos = ATM_CBR;
	ATMLayerReg.tos = ATM_AAL2_VOICE;
	ATMLayerReg.TxDone = NULL;
	ATMLayerReg.TxDataCB = NULL;

	// Use a callback for the dispatch (Rx)
	ATMLayerReg.RxDispatch = GhostRxATMLayer;

	if (ATMLayerReg.vci == GhostVci)
		{Client = 0;
		 ATMLayerReg.RxDataCB = (void *)0xaabbccdd;
		 ATMLayerReg.phy = 0;	// Phy to send to
		}
	else
		{Client = 1;
		 ATMLayerReg.RxDataCB = (void *)0xddccbbaa;
		 ATMLayerReg.phy = 0;	// Phy to send to
		}

	GhostVci++;

	if ( (ATMLayerHandle = ATM_registration_request(&ATMLayerReg)) == 0 )
		{Alert (ALERT_NORSRC_GENERAL,
			ALERT_ACT_INDICATION,
			ALERT_UNKNOWN_CHANNEL,
			AlertId_GhostTaskATMLayer_1,
			(U32) &ATMLayerReg,
			ALERT_VAL_UNUSED);
		 return;
		}

	KS_delay(SELFTASK, 100);

	BDescIndex = (U32)ATM_GetBDescHeader() + 1;

	while(TRUE)
		{int i = 0;

		 // Create the FDesc Cells
		 if ((FirstGhostFDesc = CurrentGhostFDesc = FDESC_Alloc(GlobalTxATMFDescPart)) == 0)
		 	{Alert (ALERT_NO_MEMORY,
				ALERT_ACT_INDICATION,
				ALERT_UNKNOWN_CHANNEL,
				AlertId_GhostTaskATMLayer_2,
				(U32) GlobalTxATMFDescPart,
				ALERT_VAL_UNUSED);
			 return;
			}

		 CurrentGhostFDesc->Offset = ATM_BUFFCELL_SIZE - ATM_CELL_SIZE;
		 CurrentGhostFDesc->Length = ATM_CELL_SIZE;

		 // Put the ATM payload data in BDesc[1].Bptr
		 CurrentGhostFDesc->BDesc[BDescIndex].BControl = IDMA_BCONTROL_BLAST | ATM_CELL_PAYLOAD_SIZE;
		 CurrentGhostFDesc->BDesc[BDescIndex].BPtr = CurrentGhostFDesc->Payload + CurrentGhostFDesc->Offset;

		 memset(	CurrentGhostFDesc->Payload,
		 			TxTagNumber++%255,
		 			ATM_BUFFCELL_SIZE);

		 PreviousGhostFDesc = CurrentGhostFDesc;

		 while(i++<99)
		 	{CurrentGhostFDesc = FDESC_Alloc(GlobalTxATMFDescPart);

		 	 if(CurrentGhostFDesc == NULL)
				{Alert (ALERT_NO_MEMORY,
					ALERT_ACT_INDICATION,
					ALERT_UNKNOWN_CHANNEL,
					AlertId_GhostTaskATMLayer_3,
					(U32) GlobalTxATMFDescPart,
					ALERT_VAL_UNUSED);
				 return;
				}

			 CurrentGhostFDesc->Offset = ATM_BUFFCELL_SIZE - ATM_CELL_SIZE;
			 CurrentGhostFDesc->Length = ATM_CELL_SIZE;

			 // Put the ATM payload data in BDesc[1].Bptr
			 CurrentGhostFDesc->BDesc[BDescIndex].BControl = IDMA_BCONTROL_BLAST | ATM_CELL_PAYLOAD_SIZE;
			 CurrentGhostFDesc->BDesc[BDescIndex].BPtr = CurrentGhostFDesc->Payload + CurrentGhostFDesc->Offset;

			 memset(	CurrentGhostFDesc->Payload + CurrentGhostFDesc->Offset,
			 			TxTagNumber++%255,
			 			CurrentGhostFDesc->Length);

			 PreviousGhostFDesc->Next = CurrentGhostFDesc;
			 PreviousGhostFDesc = CurrentGhostFDesc;
		 	}

		 TransmittedCells += 100;

		 // Send the cells to the ATM layer
		 ATM_Tx_Request(FirstGhostFDesc, ATMLayerHandle);

		 KS_delay(SELFTASK, 5);

		 // If we need to stop the Tx, Put StopTx to 1
		 while(StopTx)
		 	{KS_delay(SELFTASK, 5000);}

		 // Every 300 * x ms check the status of the Optiphy
 		 if(++CheckCounter == 300)
		 	{CheckCounter = 0;
			 CheckOptiPHY(PORT_3);
		 	}
		}
}
#endif

/****************************************************************************/
/*	Function	:	GhostLayer												*/
/*	Parms		:	None.													*/
/*	Returns		:	None.													*/
/****************************************************************************/
void GhostLayer(void)
{
	U32 i;

	for(i = 0; i < GHOST_FRAME_SIZE; i++)
		GhostFrame[i] = i%0xFF;
	for(i = 0; i < 2; i++)
	{
		LoopCounter = 0;
		TestNumber = 0;
	}

	// Create the Ghost Tasks : GhostTaskATMLayer or GhostTask
	if ((GhostTaskID = KSE_createtask(GhostTask, Ghosttaskstk0, TASKSTKSZ, APP_HIGH_PRI, NULL, TRUE)) == 0)
	{
		Alert(ALERT_TASK_NOT_CREATED,
			ALERT_ACT_INDICATION,
			ALERT_UNKNOWN_CHANNEL,
			AlertId_GhostLayer_1,
			ALERT_VAL_UNUSED,
			ALERT_VAL_UNUSED);
		return;
	}

	// Make sure the task gets started
	KS_delay(SELFTASK, 100);
}
#endif // AAL5_TEST_CODE

#ifndef NDEBUG
/****************************************************************************/
/*	Function	:	AAL5_ConsoleGetStatus									*/
/*	Parms		:	U32: Console parameter.									*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Get AAL5 stats through the console.						*/
/*																			*/
/****************************************************************************/
static void AAL5_ConsoleGetStatus(U32 param)
{
#if _AAL5_STATS_
	PAAL5STATS pStats = NULL;
	U32 i, DiscardedCounter = 0;
#endif
	HANDLE ClientHandle	SET_PTR_BAD(HANDLE);
	PAAL5DESCRIPTOR pCurrentDescriptor	SET_PTR_BAD(PAAL5DESCRIPTOR);

	releasePrintf((0, DEBUG_ALWAYS, "\n"));
	ClientHandle = AAL5_GetHandle(param);
	if(ClientHandle)
	{
		pCurrentDescriptor = AAL5_GetDescriptorFromAAL5(ClientHandle);
		if(pCurrentDescriptor != NULL)
		{
			releasePrintf((0, DEBUG_ALWAYS, "AAL5 Client (%d): VPI %d %d\n", param, pCurrentDescriptor->VPITx, pCurrentDescriptor->VPIRx));
			releasePrintf((0, DEBUG_ALWAYS, "AAL5 Client (%d): VCI %d %d\n", param, pCurrentDescriptor->VCITx, pCurrentDescriptor->VCIRx));
			releasePrintf((0, DEBUG_ALWAYS, "AAL5 Client (%d): PCR %d cells/s\n", param, pCurrentDescriptor->PCR));
			releasePrintf((0, DEBUG_ALWAYS, "AAL5 Client (%d): UTP Cal Day %d\n", param, pCurrentDescriptor->UTPCalendarDay));
			if(pCurrentDescriptor->RfcEncapsulationMode != HW_IO_ENCAPSULATION_NONE)
				releasePrintf((0, DEBUG_ALWAYS, "AAL5 Client (%d): RFC Encap %d\n", param, pCurrentDescriptor->RfcEncapsulationMode));
			if(pCurrentDescriptor->GPSOffset != ATM_NO_GPS_OFFSET)
				releasePrintf((0, DEBUG_ALWAYS, "AAL5 Client (%d): GPS Offset %dms\n", param, pCurrentDescriptor->GPSOffset));
		}
		else
		{
			releasePrintf((0, DEBUG_ALWAYS, "AAL5 Client (%d): No such client available (2)\n\n", param));
			goto consoleend;
		}
	}
	else
	{
		releasePrintf((0, DEBUG_ALWAYS, "AAL5 Client (%d): No such client available (1)\n\n", param));
		goto consoleend;
	}

#if _AAL5_STATS_
	// Check if any client has discarded a frame.
	for(i=0; i< MAX_AAL5_CLIENTS; i++)
	{
		if(pAAL5DescCtxt->pStatsClient[i])
		{
			pStats = pAAL5DescCtxt->pStatsClient[i];
			DiscardedCounter += pStats->AALRxDiscarded;
		}
	}

	if(DiscardedCounter)
		releasePrintf((0, DEBUG_ALWAYS, "\nAAL5 Stats (All): %d frames have been discarded\n", DiscardedCounter));
	else
		releasePrintf((0, DEBUG_ALWAYS, "\nAAL5 Stats (All): No frames discarded\n"));

	if(pAAL5DescCtxt->pStatsClient[param] == NULL)
	{
		releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): No stats available\n", param));
		goto consoleend;
	}

	pStats = pAAL5DescCtxt->pStatsClient[param];

	releasePrintf((0, DEBUG_ALWAYS, "\n"));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALTxGood %d\n", param, pStats->AALTxGood));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALTxGoodCells %d\n", param, pStats->AALTxGoodCells));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALTxDiscarded %d\n", param, pStats->AALTxDiscarded));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALTxDiscardedByte %d\n", param, pStats->AALTxDiscardedByte));

	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALRxGood %d\n", param, pStats->AALRxGood));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALRxGoodCells %d\n", param, pStats->AALRxGoodCells));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALRxDiscarded %d\n", param, pStats->AALRxDiscarded));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALRxDiscardedByte %d\n", param, pStats->AALRxDiscardedByte));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALRxInvalidRFC %d\n", param, pStats->AALRxInvalidRFC));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALRxInvalidLen %d\n", param, pStats->AALRxInvalidLen));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALRxInvalidCRC %d\n", param, pStats->AALRxInvalidCRC));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): AALRxNoBufferAvail %d\n", param, pStats->AALRxNoBufferAvail));

#if _AAL5_AVG_STATS_
	releasePrintf((0, DEBUG_ALWAYS, "\n"));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): TxCellsPerBuffer %d\n", param, pStats->TxCellsPerBuffer));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): TxBytesPerAAL5Frame %d\n", param, pStats->TxBytesPerAAL5Frame));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): TxBytesPerClientFrame %d\n", param, pStats->TxBytesPerClientFrame));
#endif

#if CHECK_BUFFERS_ALIGNMENT
	releasePrintf((0, DEBUG_ALWAYS, "\n"));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): TxBuffersAligned %d\n", param, pStats->TxBuffersAligned));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): TxBuffersNotAligned %d\n", param, pStats->TxBuffersNotAligned));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): RxBuffersAligned %d\n", param, pStats->RxBuffersAligned));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Stats (%d): RxBuffersNotAligned %d\n", param, pStats->RxBuffersNotAligned));
#endif

#if AAL5_TEST_CODE
	releasePrintf((0, DEBUG_ALWAYS, "\n"));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Test Code Stats : TransmittedCells %d\n",TransmittedCells));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Test Code Stats : ReceivedCells %d\n",ReceivedCells));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Test Code Stats : WrongCells %d\n",WrongCells));
#endif
#else //_AAL5_STATS_
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Test Code Stats: No stats available in this code!\n"));
#endif

consoleend:
	releasePrintf((0, DEBUG_ALWAYS, "\n"));
	releasePrintf((0, DEBUG_ALWAYS, "ATM Layer: AAL5 frames transmitted %d\n", gPATMContext->Stat.ATM_Tx_req_AAL5_frame));
	releasePrintf((0, DEBUG_ALWAYS, "ATM Layer: Received cells %d\n", gPATMContext->Stat.ATM_Rx_cell));
	releasePrintf((0, DEBUG_ALWAYS, "ATM Layer: Discarded cells %d\n", gPATMContext->Stat.ATM_Discarded_Rx_cell));
	releasePrintf((0, DEBUG_ALWAYS, "AAL5 Layer: %d registered clients\n", pAAL5DescCtxt->CurrentEnabledDescriptor));
	if(param == 0)
		releasePrintf((0, DEBUG_ALWAYS, "VoIP Channels Running: ARM0:%d - ARM1:%d\n",
				pDevicedesc->NumberOfRunningVoIPChannels[0],
				pDevicedesc->NumberOfRunningVoIPChannels[1]));
	else
		releasePrintf((0, DEBUG_ALWAYS, "VoATM Channels Running: ARM0:%d - ARM1:%d\n",
				pDevicedesc->NumberOfRunningVoATMChannels[0],
				pDevicedesc->NumberOfRunningVoATMChannels[1]));
	releasePrintf((0, DEBUG_ALWAYS, "\n"));
}


/****************************************************************************/
/*	Function	:	AAL5_StartConsoleDebug									*/
/*	Parms		:	None.													*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Register with the consol debug module.					*/
/*																			*/
/****************************************************************************/
static void AAL5_StartConsoleDebug(void)
{
	cmd_dbg_fct_add(AAL5_ConsoleGetStatus, "AAL5 Status");
}
#endif //NDEBUG


/************************************************************************
*																		*
*      NAME:      AAL5_Query_TEST										*
*																		*
*      USAGE:       parameter(s):										*
*																		*
*                   return value:										*
*																		*
*      DESCRIPTION:														*
************************************************************************/ 
U8 AAL5_Query_TEST(U16 * Payload)
{
	U8 offset ,Length ;
	U8 tab[0x0a];
	U8 i ;
	U8 reset;

	reset = *(Payload);
	offset = sizeof(DIAG_FIFO_Hdr) / 2;
	//debugPrintf((0,0, "taille du tableau eth (10):%d \n",sizeof(tab)));//10

	Length = sizeof(tab)+sizeof(DIAG_FIFO_Hdr);
	//debugPrintf((0,0, "taille du message eth (18):%d \n",Length));//18

	for(i = 0 ; i < sizeof(tab);i++ )
	{
		if (reset)
			tab[i] = 0;
		else
			tab[i] = i;
	}

	SFL_memcpy(Payload +  offset, tab, sizeof(tab));//

	return Length;
}
#endif // AAL5_SUPPORTED

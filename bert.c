/**
*******************************************************************************
*
* @file bert.c
*
* @brief Implementation of bit error rate testing (BERT)
* @author Mindspeed Technologies
*
* COPYRIGHT&copy; 2010 Mindspeed Technologies.
* ALL RIGHTS RESERVED
*
* This is Unpublished Proprietary Source Code of Mindspeed Technologies
*
******************************************************************************/


#include "bert.h"
#include "utillib.h"	// for memset

//#define SKIPFRAMES

void BERT_SendIndication(PSPEECH_CHANNEL this);
static void BERT_CheckLock(PSPEECH_CHANNEL this, U32 NumBits);

//#define GENERATE_BER	6	// Every 6 or more bits -> no loss of lock 

/****************************************************************************/
/*	Function		:	BERT_CheckHostPattern										*/
/*	Parms		:	PSPEECH_CHANNEL: Speech Channel descriptor.				*/
/*					U8*: Buffer with data to verify.								*/
/*					U16: Number of bytes to verify.								*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Compare a buffer of data with an host-defined pattern.			*/
/*																			*/
/****************************************************************************/
void BERT_CheckHostPattern(PSPEECH_CHANNEL this, U8* pInStream, U16 frameSize)
{
	register U32 i, mask, word, SeedDecode = this->BERTSeedDecode;
	U32 *pInstream32 = (U32*)pInStream;
	PBERT_STATS pBERTStats = &this->BERTStats;

#ifdef SKIPFRAMES
	if ( (this->BERTLocked & 0x0F00) < ((this->BERTLocked & 0xF000) >> 4) ) 
	{
		// Skip this frame, increment counter
		this->BERTLocked += 0x0080;
		return;
	}
#endif

	// Update stats
	pBERTStats->TotalBitsRx += frameSize<<3;	// frameSize * 8 bits per byte

	if((this->BERTConfig & BERT_INTF_TYPE_MASK) == BERT_PKT_INTF)
		pBERTStats->PktReceived++;		// Packet BERT

	frameSize>>=2;	// Turn bytes into U32
	while(frameSize--)
	{
		word = *pInstream32++;
	
		if(word == SeedDecode)
		{// This whole word is good
			pBERTStats->TotalGoodBitsRx += sizeof(U32)<<3;	// 4 Good Bytes (4 * 8 bits per byte)

			if(this->BERTLocked & 1)
			{// Already locked
				// Update stats
				pBERTStats->LockGoodBitsRx += sizeof(U32)<<3;	// 4 Good Bytes (4 * 8 bits per byte)
			}
			else
			{// Check if we have enough good bits to lock
				pBERTStats->ConsecutiveGoodBitsRx += sizeof(U32)<<3;	// 4 Good Bytes (4 * 8 bits per byte)

				if(pBERTStats->ConsecutiveGoodBitsRx >= this->BERTBitsToLock)
				{// Locking
					this->BERTLocked |= 1;
					pBERTStats->LockTransition++;
					pBERTStats->LockGoodBitsRx = pBERTStats->ConsecutiveGoodBitsRx - this->BERTBitsToLock;
					pBERTStats->LockBadBitsRx = 0;

					// Lock check
					pBERTStats->BERTLockCheck_1secRx = 0;
					pBERTStats->BERTLockCheck_1secError = 0;
					pBERTStats->BERTLockCheck_1secErrorLast = 0;

					pBERTStats->ConsecutiveGoodBitsRx = 0;

					// Send indication to Host
					BERT_SendIndication(this);
				}
			}
		}
		else
		{// The word is incorrect. Check each and every bit of it.
			i = 32;
			while(i--)
			{
				mask = (U32)1 << i;

				if( (word&mask) == (SeedDecode&mask))
				{// This bit is good
					pBERTStats->TotalGoodBitsRx++;

					 if(this->BERTLocked & 1)
					{// Already locked
						// Update stats
						pBERTStats->LockGoodBitsRx++;
					}
					 else if(++pBERTStats->ConsecutiveGoodBitsRx >= this->BERTBitsToLock)
					{// Locking
						this->BERTLocked |= 1;
						pBERTStats->LockTransition++;
						pBERTStats->LockGoodBitsRx = 1;
						pBERTStats->LockBadBitsRx = 0;

						// Lock check
						pBERTStats->BERTLockCheck_1secRx = 0;
						pBERTStats->BERTLockCheck_1secError = 0;
						pBERTStats->BERTLockCheck_1secErrorLast = 0;

						pBERTStats->ConsecutiveGoodBitsRx = 0;

						// Send indication to Host
						BERT_SendIndication(this);
					}
				}
				else
				{// This bit is bad
					pBERTStats->TotalBadBitsRx++;

					if(this->BERTLocked & 1)
					{
						// Update stats
						pBERTStats->LockBadBitsRx++;
						pBERTStats->BERTLockCheck_1secError++;
					}
					else
						pBERTStats->ConsecutiveGoodBitsRx = 0;
				}
			}	
		}

		if(this->BERTLocked & 1)
			BERT_CheckLock(this, 32);		// Check loss of lock only every 32 bits (4 bytes)
	}
}

/****************************************************************************/
/*	Function		:	BERT_CheckPRBS											*/
/*	Parms		:	PSPEECH_CHANNEL: Speech Channel descriptor.				*/
/*					U8*: Buffer with data to verify.								*/
/*					U16: Number of bytes to verify.								*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Compare a buffer of data with an 2^15-1 pseudo-random			*/
/*					bit sequence.												*/
/*																			*/
/*	Note			:	When not locked, the code could potentially lock on u-law silence	*/
/*					(0xFF) once inverted (0x00). To prevent that, a check for the max	*/
/*					possible number of consecutive zeros has been added.			*/
/*					Also, an auto-correction code (when locked) has been added. This	*/
/*					prevents error from being propagated: a single bit error could		*/
/*					create 3 different error (when the bit is at stage01, stage14 and	*/
/*					stage15).													*/
/*																			*/
/****************************************************************************/
void BERT_CheckPRBS(PSPEECH_CHANNEL this, U8* pInStream, U16 frameSize)
{
	U32 SeedDecode = this->BERTSeedDecode;
	U32 Stage15, Stage14, Stage01, index;
	PBERT_STATS pBERTStats = &this->BERTStats;
	U32 BitsToProcess = frameSize<<3;	// frameSize * 8 bits per byte

#ifdef SKIPFRAMES
	if ( (this->BERTLocked & 0x0F00) < ((this->BERTLocked & 0xF000) >> 4) ) 
	{
		// Skip this frame, increment counter
		this->BERTLocked += 0x0080;
		return;
	}
#endif

	// Update stats
	pBERTStats->TotalBitsRx += BitsToProcess;

	if((this->BERTConfig & BERT_INTF_TYPE_MASK) == BERT_PKT_INTF)
		pBERTStats->PktReceived++;		// Packet BERT

	index = 0;
	while(BitsToProcess)
	{
		if((BitsToProcess & 0x0F) == 0)
		{// We're starting to process a new 16 bits word

			SeedDecode &= ~0xFFFF;			// Clear lower 16 bits
			SeedDecode |= (pInStream[index] << 8 | pInStream[index+1]);
			index+=2;

			if((this->BERTLocked & 1) && (index>2))
				BERT_CheckLock(this, 16);		// Check loss of lock only every 16 bits (2 bytes)
		}

		// Collect the 15th, 14th & 1st stage output
		Stage15 = (SeedDecode & DEC_STAGE_15_OUTPUT)>>DEC_STAGE_15_SHIFT;
		Stage14 = (SeedDecode & DEC_STAGE_14_OUTPUT)>>DEC_STAGE_14_SHIFT;
		Stage01 = (SeedDecode & DEC_STAGE_01_OUTPUT)>>DEC_STAGE_01_SHIFT;

		// Compare the result of the modulo-two addition (XOR) of the 14th & 15th stage outputs to the first stage
		if(Stage01 == (Stage14^Stage15))
		{// This bit is good...maybe

			if(this->BERTLocked & 1)
			{// Already locked
				// Update stats
				pBERTStats->TotalGoodBitsRx++;
				pBERTStats->LockGoodBitsRx++;
			}
			else
			{// Not locked

				// A continuous stream of zeroes can falsely trigger a lock - check against it
				if(Stage01 == 0)
				{
					if(++pBERTStats->ConsecutiveZerosWhenNotLocked > BERT_LONGEST_SEQ_OF_ZEROS)
					{// This is a bad bit...can't have more than BERT_LONGEST_SEQ_OF_ZEROS zeros
						pBERTStats->TotalBadBitsRx++;
						pBERTStats->ConsecutiveGoodBitsRx = 0;
						pBERTStats->ConsecutiveZerosWhenNotLocked = 0;
						continue;
					}
				}
				else
					pBERTStats->ConsecutiveZerosWhenNotLocked = 0;

				// This is a good bit
				pBERTStats->TotalGoodBitsRx++;

				if(++pBERTStats->ConsecutiveGoodBitsRx == this->BERTBitsToLock)
				{// Locking
					this->BERTLocked |= 1;
					pBERTStats->LockTransition++;
					pBERTStats->LockGoodBitsRx = 1;
					pBERTStats->LockBadBitsRx = 0;

					pBERTStats->ConsecutiveGoodBitsRx = 0;

					// Lock check
					pBERTStats->BERTLockCheck_1secRx = 0;
					pBERTStats->BERTLockCheck_1secError = 0;
					pBERTStats->BERTLockCheck_1secErrorLast = 0;

					// Send indication to Host
					BERT_SendIndication(this);
				}
			}
		}
		else
		{// This bit is bad
			pBERTStats->TotalBadBitsRx++;

			if(this->BERTLocked & 1)
			{
				// Update stats
				pBERTStats->LockBadBitsRx++;
				pBERTStats->BERTLockCheck_1secError++;

				// This bit is wrong but we know what it should have been so in order to prevent
				// the error from propagating, correct the input stream.
				SeedDecode &= ~DEC_STAGE_01_OUTPUT;					// Clear bit
				SeedDecode |= (Stage14^Stage15) << DEC_STAGE_01_SHIFT;	// Set correct bit
			}
			else
				pBERTStats->ConsecutiveGoodBitsRx = 0;
		}

		// Shift seed to the left
		SeedDecode <<= 1;

		// Update number of bits left to be processed
		BitsToProcess--;
	}

	// Check lock before exiting
	if(this->BERTLocked & 1)
		BERT_CheckLock(this, 16);

	// Save seed
	this->BERTSeedDecode = SeedDecode;
}

/****************************************************************************/
/*	Function		:	BERT_PacketLost											*/
/*	Parms		:	PSPEECH_CHANNEL: Speech Channel descriptor.				*/
/*					U16: Number of bytes expected.								*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Update stats when a packet is lost.							*/
/*																			*/
/****************************************************************************/
void BERT_PacketLost(PSPEECH_CHANNEL this, U16 frameSize)
{
	PBERT_STATS pBERTStats = &this->BERTStats;

	// Update Stats
	pBERTStats->PktLost++;
	pBERTStats->TotalBitsRx += frameSize<<3;		// frameSize * 8 bits per byte
	pBERTStats->TotalBadBitsRx += frameSize<<3;	// frameSize * 8 bits per byte

	pBERTStats->BERTLockCheck_1secRx = 0;
	pBERTStats->BERTLockCheck_1secError = 0;
	pBERTStats->BERTLockCheck_1secErrorLast = 0;
	pBERTStats->ConsecutiveGoodBitsRx = 0;

	if(this->BERTLocked & 1)
	{
		this->BERTLocked &= ~1;

		// Send indication to Host
		BERT_SendIndication(this);
	}
}

/****************************************************************************/
/*	Function		:	BERT_GenerateHostPattern									*/
/*	Parms		:	PSPEECH_CHANNEL: Speech Channel descriptor.				*/
/*					U8*: Buffer to write data to.									*/
/*					U16: Number of bytes to generate.							*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Generate a Host-defined bit sequence.							*/
/*																			*/
/****************************************************************************/
void BERT_GenerateHostPattern(PSPEECH_CHANNEL this, U8* pOutStream, U16 frameSize)
{
#ifdef SKIPFRAMES
	if ( (this->BERTLocked & 0x0078) < ((this->BERTLocked & 0xF000) >> 9) ) 
	{
		// Skip this frame, increment counter, fill by 0xFF
		this->BERTLocked += 0x0004;
		memset(pOutStream, 0xFF, frameSize);
		return;
	}
#endif

	// Update stats
	this->BERTStats.TotalBitsTx += frameSize<<3;	// frameSize * 8 bits per byte

	memset(pOutStream, (U8)this->BERTSeedEncode, frameSize);
}

/****************************************************************************/
/*	Function		:	BERT_GeneratePRBS										*/
/*	Parms		:	PSPEECH_CHANNEL: Speech Channel descriptor.				*/
/*					U8*: Buffer to write data to.									*/
/*					U16: Number of bytes to generate.							*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Generate a 2^15-1 pseudo-random bit sequence.				*/
/*																			*/
/****************************************************************************/
void BERT_GeneratePRBS(PSPEECH_CHANNEL this, U8* pOutStream, U16 frameSize)
{
	U32 SeedEncode = (U32)this->BERTSeedEncode;
	U32 Stage15, Stage14;
	U32 BitsToProcess = (U32)frameSize<<3;	// frameSize * 8 bits per byte

#ifdef SKIPFRAMES
	if ( (this->BERTLocked & 0x0078) < ((this->BERTLocked & 0xF000) >> 9) ) 
	{
		// Skip this frame, increment counter, fill by 0xFF
		this->BERTLocked += 0x0004;
		memset(pOutStream, 0xFF, frameSize);
		return;
	}
#endif

	// Update stats
	this->BERTStats.TotalBitsTx += BitsToProcess;

	while(BitsToProcess--)
	{
		// Collect the 15th & 14th stage output
		Stage15 = (SeedEncode & ENC_STAGE_15_OUTPUT)>>ENC_STAGE_15_SHIFT;
		Stage14 = (SeedEncode & ENC_STAGE_14_OUTPUT)>>ENC_STAGE_14_SHIFT;

		// Add in a modulo-two addition (XOR) the 14th & 15th stage outputs
		// and feed the result back to the input of the 1st stage.
		SeedEncode |= (Stage14^Stage15);

#ifdef GENERATE_BER
		if(this->BERTLocked & 1)
		{// Only start BER gen when we have lock
			if(++this->BERTStats.GenerateBER == GENERATE_BER)
			{// Need to modif Stage 15
				Stage15 = (~SeedEncode) & ENC_STAGE_15_OUTPUT;	// Reverse Stage15
				SeedEncode &= ~ENC_STAGE_15_OUTPUT;			// Clear bit
				SeedEncode |= Stage15;							// Insert new bit

				// Reset counter
				this->BERTStats.GenerateBER = 0;
			}
		}
#endif

		// Shift seed to the left
		SeedEncode <<= 1;

		if((BitsToProcess & 0x0F) == 0)
		{// We have 16 bits ready
			*pOutStream++ = (U8) ((SeedEncode & 0xFF000000)>>24);
			*pOutStream++ = (U8) ((SeedEncode & 0x00FF0000)>>16);
		}
	}

	// Save seed
	this->BERTSeedEncode = (U16)SeedEncode;
}

/****************************************************************************/
/*	Function		:	BERT_GeneratStatReport									*/
/*	Parms		:	PSPEECH_CHANNEL: Speech Channel descriptor.				*/
/*					U16 *: API Payload.										*/
/*					U16: Reset or not stats.										*/
/*	Returns		:	U32: Number of bytes written.								*/
/*																			*/
/*	Purpose		:	Generate stats payload conform with the NFD.					*/
/*																			*/
/****************************************************************************/
U32 BERT_GenerateStatReport(PSPEECH_CHANNEL this, U16*Payload, U16 ResetStat)
{
	U32*pPayload32 = (U32*)Payload;
	PBERT_STATS pBERTStats = &this->BERTStats;
	U32 Length = 0;

	if(pBERTStats)
	{
		*pPayload32++ = BERT_STATS_VERSION;			Length += sizeof(U32);
		*pPayload32++ = (U32)(this->BERTLocked & 1);	Length += sizeof(U32);
		*pPayload32++ = pBERTStats->TotalBitsRx;		Length += sizeof(U32);
		*pPayload32++ = pBERTStats->TotalGoodBitsRx;	Length += sizeof(U32);
		*pPayload32++ = pBERTStats->TotalBadBitsRx;	Length += sizeof(U32);
		*pPayload32++ = pBERTStats->LockGoodBitsRx;	Length += sizeof(U32);
		*pPayload32++ = pBERTStats->LockBadBitsRx;		Length += sizeof(U32);
		*pPayload32++ = pBERTStats->LockTransition;		Length += sizeof(U32);
		*pPayload32++ = pBERTStats->TotalBitsTx;		Length += sizeof(U32);
		*pPayload32++ = pBERTStats->PktReceived;		Length += sizeof(U32);
		*pPayload32++ = pBERTStats->PktLost;			Length += sizeof(U32);

		if(ResetStat)
			memset(pBERTStats, 0, sizeof(U32)*9);
	}

	return Length;
}

/****************************************************************************/
/*	Function		:	BERT_SendIndication										*/
/*	Parms		:	PSPEECH_CHANNEL: Speech Channel descriptor.				*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	Send an indication message to the host.						*/
/*																			*/
/****************************************************************************/
void BERT_SendIndication(PSPEECH_CHANNEL this)
{
	PCMDHEADER p_ind;

	// Check if sending an indication is needed
	if((this->BERTConfig & BERT_REPORT_LOCK_STATE_MASK) == BERT_REPORT_LOCK_STATE_OFF)
		return;

	if((p_ind = (PCMDHEADER)SFL_alloc_part(&GlobalHiINDPart)) == NULL)
	{
		pDevicedesc->NumberOfLostIndications++;
		return;
	}

	p_ind->ID = this->chanID;
	p_ind->Index = this->BERTIndex++;
	p_ind->Class = CC_CONFIG;
	p_ind->FuncCode = BERT_LOCK_IND;
	p_ind->Length = 2;
	p_ind->Payload[0] = (U16)(this->BERTLocked & 1);

	chanSendInd(p_ind);
}

/****************************************************************************/
/*	Function		:	BERT_CheckLock											*/
/*	Parms		:	PSPEECH_CHANNEL: Speech Channel descriptor.				*/
/*					U32: Number of bits verified since last Lock check.				*/
/*	Returns		:	None.													*/
/*																			*/
/*	Purpose		:	During lock only, check if a received bit cause loss of lock.		*/
/*																			*/
/*	Note			:	Sequence synchronization shall be considered to be lost and		*/
/*					resynchronization shall be started if the bit error ratio is 			*/
/*					>= 0.20 during an integration interval of 1 second.				*/
/*																			*/
/****************************************************************************/
static void BERT_CheckLock(PSPEECH_CHANNEL this, U32 NumBits)
{
	PBERT_STATS pBERTStats = &this->BERTStats;

	// Here we need to check if the BER is >= 0.20 during 1 sec. We know how many bits are generated in 1sec
	// so we need to check that: ERRORS >= 0.20 * BITGEN
	// or in other words: ERRORS*5 >= BITGEN
	if((pBERTStats->BERTLockCheck_1secError*5) >= BERT_ONESEC_BITGEN)
	{// Ok...loss of lock triggered
		this->BERTLocked &= ~1;

		pBERTStats->BERTLockCheck_1secRx = 0;
		pBERTStats->BERTLockCheck_1secError = 0;
		pBERTStats->BERTLockCheck_1secErrorLast = 0;
		pBERTStats->ConsecutiveGoodBitsRx = 0;

		// Send indication to Host
		BERT_SendIndication(this);

		return;
	}

	// Only start period check when errors are received. This make sure that error burst that would trigger a loss of lock do not get split up between
	// 2 arbitrary 1sec period resulting in 2 error bursts that do not trigger an error.
	if(pBERTStats->BERTLockCheck_1secRx || (pBERTStats->BERTLockCheck_1secErrorLast != pBERTStats->BERTLockCheck_1secError))
	{
		// Check if the 1 second testing period is over
		pBERTStats->BERTLockCheck_1secRx += NumBits;
		if(pBERTStats->BERTLockCheck_1secRx >= BERT_ONESEC_BITGEN)
		{// 1 sec down. Start again.
			pBERTStats->BERTLockCheck_1secRx = 0;
			pBERTStats->BERTLockCheck_1secError = 0;
		}

		pBERTStats->BERTLockCheck_1secErrorLast = pBERTStats->BERTLockCheck_1secError;
	}
}

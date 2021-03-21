// Taken from: https://github.com/daveake/pico-tracker/blob/main/misc.c
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#include "lora_spi.h"

char Hex(char Character)
{
    char HexTable[] = "0123456789ABCDEF";

    return HexTable[Character];
}

uint64_t get_time(void)
{
    // Returns result in us

    // Reading low latches the high value
    uint32_t lo = timer_hw->timelr;
    uint32_t hi = timer_hw->timehr;
    return ((uint64_t) hi << 32u) | lo;
}

int BuildSentence(struct SENSINFO *BMEINFO, char *TxLine, const char *PayloadID)
{
    static unsigned int SentenceCounter=0;
    int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
    char CRCString[8];

    SentenceCounter++;


    //,%.5f,%.5f,%05.5ld,%u,%.1f,%.1f,%.1f,%.0f,%.1f,%.2f,%7.5f,%7.5f,%3.1f,%d
    sprintf(TxLine,
            // SENTENCE_LENGTH-6,
            "$$%s,%d,%i,%i,%i",
            PayloadID,
            SentenceCounter,
            BMEINFO->bme280_temp, BMEINFO->bme280_pres, BMEINFO->bme280_humd
    );

    Count = strlen(TxLine);

    CRC = 0xffff;           // Seed
    xPolynomial = 0x1021;

    for (i = 2; i < Count; i++)
    {   // For speed, repeat calculation instead of looping for each bit
        CRC ^= (((unsigned int)TxLine[i]) << 8);
        for (j=0; j<8; j++)
        {
            if (CRC & 0x8000)
                CRC = (CRC << 1) ^ 0x1021;
            else
                CRC <<= 1;
        }
    }

    TxLine[Count++] = '*';
    TxLine[Count++] = Hex((CRC >> 12) & 15);
    TxLine[Count++] = Hex((CRC >> 8) & 15);
    TxLine[Count++] = Hex((CRC >> 4) & 15);
    TxLine[Count++] = Hex(CRC & 15);
    TxLine[Count++] = '\n';
    TxLine[Count++] = '\0';

    return strlen(TxLine) + 1;
}

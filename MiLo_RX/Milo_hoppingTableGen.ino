#include "Milo_hoppingTableGen.h"
#include <SHA3.h>

void ICACHE_RAM_ATTR Fhss_Init()
    {
        // fhss_freq_list = fhss_freq_list_2p4;
        // fhss_bind_channel_list = fhss_bind_channel_list_2p4;
        // freq_list_len = (uint8_t)(sizeof(fhss_freq_list_2p4) / sizeof(uint32_t));
        // fhss_bind_channel_list_len = (uint8_t)(sizeof(fhss_bind_channel_list_2p4) / sizeof(uint8_t));
        // cnt = FHSS_HOPPING_CHANNELS;
        curr_i = 0;
        is_in_binding = false;
        _seed = 0;
    }
        
void ICACHE_RAM_ATTR Fhss_generate(uint32_t seed)//
    {
        _seed = seed;
        uint8_t buf[SHA256_BLOCK_SIZE], tempHopList[SHA256_BLOCK_SIZE], tempHopListB[SHA256_BLOCK_SIZE], tempHopListG[SHA256_BLOCK_SIZE];
        
        // expand seed to 256 bits by sha3_256
        SHA3_256 sha3 /*= new SHA3_256()*/;
        sha3.update((void *)&_seed, sizeof(uint32_t));
        sha3.finalize(buf, SHA256_BLOCK_SIZE);
        
        // copy SHA256 seed to preserve full "entropy" before manipulation 
        for (uint8_t i=0; i<SHA256_BLOCK_SIZE; i++)
        {
            tempHopListB[i]=(buf[i]%17);    //create a list of 17 unique channels in a permutation depending on _seed
        }

        // eliminate duplicates
        uint8_t i, j, k, lenB=SHA256_BLOCK_SIZE, lenG=SHA256_BLOCK_SIZE;
        for(i=0;i<lenB;i++)
        {
            for(j = i+1; j < lenB; j++)
            {
                if(tempHopList[i] == tempHopList[j])
                {
                    for(k = j; k <lenB; k++)
                    {
                        tempHopList[k] = tempHopList[k+1];
                    }
                    j--;
                    lenB--;
                }
            }
        }
        

        for (uint8_t i=0; i<SHA256_BLOCK_SIZE; i++)
        {
            tempHopListG[i]=(buf[i]%8);     //create a list of 8 unique channels in a permutation depending on _seed
        }
        
        for(i=0;i<lenG;i++)
        {
            for(j = i+1; j < lenG; j++)
            {
                if(tempHopList[i] == tempHopList[j])
                {
                    for(k = j; k <lenG; k++)
                    {
                        tempHopList[k] = tempHopList[k+1];
                    }
                    j--;
                    lenG--;
                }
            }
        }

/*        if (lenB<8 || lenG<3)
        {
            //lets think what we should do if the hopping sequence gets too short
        }
        else */
        {
            uint32_t fix_permutator = buf[0];
            fix_permutator = fix_permutator<<8 || buf[1];
            fix_permutator =fix_permutator%24;
        

            hoppingTable[0]=fhss_block_fix[fix_permutator][0]; // fixed channel
            hoppingTable[1]=fhss_block_bad[1][1];
            hoppingTable[2]=fhss_block_bad[1][2];
            hoppingTable[3]=fhss_block_good[1][1];
            hoppingTable[4]=fhss_block_bad[1][3];
            hoppingTable[5]=fhss_block_bad[2][1];
            hoppingTable[6]=fhss_block_good[1][2];
            hoppingTable[7]=fhss_block_bad[2][2];
            hoppingTable[8]=fhss_block_bad[2][3];
            hoppingTable[FHSS_HOPPING_CHANNELS/4]==fhss_block_fix[fix_permutator][1]; // fixed channel
            hoppingTable[10]=fhss_block_bad[3][1];
            hoppingTable[11]=fhss_block_bad[3][2];
            hoppingTable[12]=fhss_block_good[1][3];                        
            hoppingTable[13]=fhss_block_bad[3][3];
            hoppingTable[14]=fhss_block_bad[4][1];
            hoppingTable[15]=fhss_block_good[2][1];
            hoppingTable[16]=fhss_block_bad[4][2];
            hoppingTable[17]=fhss_block_bad[4][3];
            hoppingTable[FHSS_HOPPING_CHANNELS/2]==fhss_block_fix[fix_permutator][2]; // fixed channel                                                            
            hoppingTable[19]=fhss_block_bad[5][1];
            hoppingTable[20]=fhss_block_bad[5][2];
            hoppingTable[21]=fhss_block_good[2][2];  
            hoppingTable[22]=fhss_block_bad[5][3];
            hoppingTable[23]=fhss_block_bad[6][1];
            hoppingTable[24]=fhss_block_good[2][3];
            hoppingTable[25]=fhss_block_bad[6][2];
            hoppingTable[26]=fhss_block_bad[6][3];
            hoppingTable[FHSS_HOPPING_CHANNELS/4*3]==fhss_block_fix[fix_permutator][3]; // fixed channel
            hoppingTable[28]=fhss_block_bad[7][1];
            hoppingTable[29]=fhss_block_bad[7][2];
            hoppingTable[30]=fhss_block_good[3][1];  
            hoppingTable[31]=fhss_block_bad[7][3];
            hoppingTable[32]=fhss_block_bad[8][1];
            hoppingTable[33]=fhss_block_good[3][2];
            hoppingTable[34]=fhss_block_bad[8][2];
            hoppingTable[35]=fhss_block_bad[8][3];            
        }
                    
/*        bool used_flag[FHSS_FREQ_LIST_MAX_LEN];
        
        for (uint8_t ch = 0; ch < FHSS_FREQ_LIST_MAX_LEN; ch++) 
        used_flag[ch] = false;
*/
        curr_i = 0; 
/*        //mark all channels as equally bad
        for (uint8_t k = 0; k < cnt; k++) 
        {
            fhss_last_rssi[k] = -128 ;
        }
*/
            #ifdef TEST
                //delay(1000);
                // for (uint8_t k = 0; k < FHSS_HOPPING_CHANNELS; k++) 
                // Serial.println(hoppingTable[k]);
            #endif
    }

void ICACHE_RAM_ATTR nextChannel()
    {
        ++curr_i%FHSS_HOPPING_CHANNELS;
    }

uint32_t ICACHE_RAM_ATTR GetInitialFreq()
    {
        return fhss_freq_list[fhss_bind_channel_list[0]];
    }
    
uint32_t ICACHE_RAM_ATTR GetCurrFreq(void)
    {
        return fhss_list[hoppingTable[curr_i]];
    }

#pragma once

//#define DEBUG_FHSS
//#define DEBUG_WITH_FIXED_FHSS
#define USE_FHSS_WITH_SYNCHRO

//from mLRS
//https://github.com/olliw42/mLRS/
// 2406.0 ... 2473.0  in 1 MHz steps
// = 68 channels//BW is 812Khz so band resolution of 1Mhz is suitable and no adiacent chanels so they will be randomized
//**********************************************************************************************
#define SX1280_FREQ_XTAL_MHZ               52
#define SX1280_FREQ_MHZ_TO_REG(f_mhz)     (uint32_t)(((double)(2402+f_mhz)*(double)(1 << 18))/SX1280_FREQ_XTAL_MHZ)
// this is what is suggested by Semtech
// #include <math.h>
// #define SX1280_FREQ_STEP                  ((double)(SX1280_FREQ_XTAL_HZ / pow(2.0,18.0))) // 198.3642578
// #define SX1280_FREQ_HZ_TO_REG(f_hz)       ((uint32_t)( (double)f_hz / (double)SX1280_FREQ_STEP ))
// the pow(x,y) is ugly, and using GHz units is more convenient

// channel list expanded to match list in Bluetooth core specification 5.3 page 389: f=2402+k MHz, k=0..78
const uint32_t fhss_freq_list_2p4[] = 
{
    SX1280_FREQ_MHZ_TO_REG(0),  // channel 0, 2402 MHz
    SX1280_FREQ_MHZ_TO_REG(1),  // channel 1, 2403 MHz
    SX1280_FREQ_MHZ_TO_REG(2),  // channel 2, 2404 MHz
    SX1280_FREQ_MHZ_TO_REG(3),  // channel 3, 2405 MHz
    SX1280_FREQ_MHZ_TO_REG(4),  // channel 4, 2406 MHz
    SX1280_FREQ_MHZ_TO_REG(5),  // channel 5, 2407 MHz
    SX1280_FREQ_MHZ_TO_REG(6),  // channel 6, 2408 MHz
    SX1280_FREQ_MHZ_TO_REG(7),  // channel 7, 2409 MHz
    SX1280_FREQ_MHZ_TO_REG(8),  // channel 8, 2410 MHz
    SX1280_FREQ_MHZ_TO_REG(9),  // channel 9, 2411 MHz
    SX1280_FREQ_MHZ_TO_REG(10),  // channel 10, 2412 MHz
    SX1280_FREQ_MHZ_TO_REG(11),  // channel 11, 2413 MHz
    SX1280_FREQ_MHZ_TO_REG(12),  // channel 12, 2414 MHz
    SX1280_FREQ_MHZ_TO_REG(13),  // channel 13, 2415 MHz
    SX1280_FREQ_MHZ_TO_REG(14),  // channel 14, 2416 MHz
    SX1280_FREQ_MHZ_TO_REG(15),  // channel 15, 2417 MHz
    SX1280_FREQ_MHZ_TO_REG(16),  // channel 16, 2418 MHz
    SX1280_FREQ_MHZ_TO_REG(17),  // channel 17, 2419 MHz
    SX1280_FREQ_MHZ_TO_REG(18),  // channel 18, 2420 MHz
    SX1280_FREQ_MHZ_TO_REG(19),  // channel 19, 2421 MHz
    SX1280_FREQ_MHZ_TO_REG(20),  // channel 20, 2422 MHz
    SX1280_FREQ_MHZ_TO_REG(21),  // channel 21, 2423 MHz
    SX1280_FREQ_MHZ_TO_REG(22),  // channel 22, 2424 MHz
    SX1280_FREQ_MHZ_TO_REG(23),  // channel 23, 2425 MHz
    SX1280_FREQ_MHZ_TO_REG(24),  // channel 24, 2426 MHz
    SX1280_FREQ_MHZ_TO_REG(25),  // channel 25, 2427 MHz
    SX1280_FREQ_MHZ_TO_REG(26),  // channel 26, 2428 MHz
    SX1280_FREQ_MHZ_TO_REG(27),  // channel 27, 2429 MHz
    SX1280_FREQ_MHZ_TO_REG(28),  // channel 28, 2430 MHz
    SX1280_FREQ_MHZ_TO_REG(29),  // channel 29, 2431 MHz
    SX1280_FREQ_MHZ_TO_REG(30),  // channel 30, 2432 MHz
    SX1280_FREQ_MHZ_TO_REG(31),  // channel 31, 2433 MHz
    SX1280_FREQ_MHZ_TO_REG(32),  // channel 32, 2434 MHz
    SX1280_FREQ_MHZ_TO_REG(33),  // channel 33, 2435 MHz
    SX1280_FREQ_MHZ_TO_REG(34),  // channel 34, 2436 MHz
    SX1280_FREQ_MHZ_TO_REG(35),  // channel 35, 2437 MHz
    SX1280_FREQ_MHZ_TO_REG(36),  // channel 36, 2438 MHz
    SX1280_FREQ_MHZ_TO_REG(37),  // channel 37, 2439 MHz
    SX1280_FREQ_MHZ_TO_REG(38),  // channel 38, 2440 MHz
    SX1280_FREQ_MHZ_TO_REG(39),  // channel 39, 2441 MHz
    SX1280_FREQ_MHZ_TO_REG(40),  // channel 40, 2442 MHz
    SX1280_FREQ_MHZ_TO_REG(41),  // channel 41, 2443 MHz
    SX1280_FREQ_MHZ_TO_REG(42),  // channel 42, 2444 MHz
    SX1280_FREQ_MHZ_TO_REG(43),  // channel 43, 2445 MHz
    SX1280_FREQ_MHZ_TO_REG(44),  // channel 44, 2446 MHz
    SX1280_FREQ_MHZ_TO_REG(45),  // channel 45, 2447 MHz
    SX1280_FREQ_MHZ_TO_REG(46),  // channel 46, 2448 MHz
    SX1280_FREQ_MHZ_TO_REG(47),  // channel 47, 2449 MHz
    SX1280_FREQ_MHZ_TO_REG(48),  // channel 48, 2450 MHz
    SX1280_FREQ_MHZ_TO_REG(49),  // channel 49, 2451 MHz
    SX1280_FREQ_MHZ_TO_REG(50),  // channel 50, 2452 MHz
    SX1280_FREQ_MHZ_TO_REG(51),  // channel 51, 2453 MHz
    SX1280_FREQ_MHZ_TO_REG(52),  // channel 52, 2454 MHz
    SX1280_FREQ_MHZ_TO_REG(53),  // channel 53, 2455 MHz
    SX1280_FREQ_MHZ_TO_REG(54),  // channel 54, 2456 MHz
    SX1280_FREQ_MHZ_TO_REG(55),  // channel 55, 2457 MHz
    SX1280_FREQ_MHZ_TO_REG(56),  // channel 56, 2458 MHz
    SX1280_FREQ_MHZ_TO_REG(57),  // channel 57, 2459 MHz
    SX1280_FREQ_MHZ_TO_REG(58),  // channel 58, 2460 MHz
    SX1280_FREQ_MHZ_TO_REG(59),  // channel 59, 2461 MHz
    SX1280_FREQ_MHZ_TO_REG(60),  // channel 60, 2462 MHz
    SX1280_FREQ_MHZ_TO_REG(61),  // channel 61, 2463 MHz
    SX1280_FREQ_MHZ_TO_REG(62),  // channel 62, 2464 MHz
    SX1280_FREQ_MHZ_TO_REG(63),  // channel 63, 2465 MHz
    SX1280_FREQ_MHZ_TO_REG(64),  // channel 64, 2466 MHz
    SX1280_FREQ_MHZ_TO_REG(65),  // channel 65, 2467 MHz
    SX1280_FREQ_MHZ_TO_REG(66),  // channel 66, 2468 MHz
    SX1280_FREQ_MHZ_TO_REG(67),  // channel 67, 2469 MHz
    SX1280_FREQ_MHZ_TO_REG(68),  // channel 68, 2470 MHz
    SX1280_FREQ_MHZ_TO_REG(69),  // channel 69, 2471 MHz
    SX1280_FREQ_MHZ_TO_REG(70),  // channel 70, 2472 MHz
    SX1280_FREQ_MHZ_TO_REG(71),  // channel 71, 2473 MHz
    SX1280_FREQ_MHZ_TO_REG(72),  // channel 72, 2474 MHz
    SX1280_FREQ_MHZ_TO_REG(73),  // channel 73, 2475 MHz
    SX1280_FREQ_MHZ_TO_REG(74),  // channel 74, 2476 MHz
    SX1280_FREQ_MHZ_TO_REG(75),  // channel 75, 2477 MHz
    SX1280_FREQ_MHZ_TO_REG(76),  // channel 76, 2478 MHz
    SX1280_FREQ_MHZ_TO_REG(77),  // channel 77, 2479 MHz
    SX1280_FREQ_MHZ_TO_REG(78),  // channel 78, 2480 MHz
};

const uint8_t fhss_bind_channel_list_2p4[] = 
{
    46 //,14, 33, 61 // just pick some
};

#ifdef USE_FHSS_WITH_SYNCHRO
    #define FHSS_CHANNELS_NUM  68
    #define FHSS_SYNCHRO_CHANNELS_NUM 5 // maximum number of channels reserved in channel list to be used when waiting for a connection. 
                                               // Those are the n first channels in the list and are the only one allowed when trying to (re)connect;
                                               // They have a gap of 2 at least with all others channels in the list
                                               // There are spread over the whole range fo frequency (= synchro channel one per section)
                                               // The purpose is to avoid recynchronizing Rx with Tx based on a channel that is to adjacent to the expected one.
                                               // This occurs when the Rx id not selective enough and accept a frame on an adjacent channels
     #ifdef DEBUG_WITH_FIXED_FHSS
        uint8_t debug_ch_list[] =     // 68 values
        {
            7, 22 , 37, 52, 67,  //0-4
            3, 27, 34, 56, 60,   //5-9
            0, 26, 32, 45, 71,   //10-14
            11, 16, 41, 48, 62,  //15-19
            1, 29, 42, 47, 61,   //20-24
            10, 17, 30, 55, 63,  //25-29
            14, 28, 44, 57, 74,  //30-34
            12, 18, 33, 58, 70,  //35-39
            13, 19, 31, 49, 72,  //40-44
            4, 25, 40, 59, 73,   //45-49
            2, 15, 43, 46, 64,   //50-54
            13, 19, 31, 49, 72,  //55-59
            4, 25, 40, 59, 73,   //60-64
            2, 15, 43, 46,       //65-68
            
        };
    #endif        
#endif


// https://en.wikipedia.org/wiki/Linear_congruential_generator: Microsoft Visual/Quick C/C++
// also used by ELRS;
// generates values in range [0, 0x7FFF]

// spektrum:
// appears to use val = val * 0x0019660D + 0x3C6EF35F; (val >> 8) % 73
// is numerical recipes method in the wikipedia source
// picks 24 channels out of 74, ensures it's distributed into three slots

// redpine:
// same prng as spektrum, picks 50 channels, ensures not close and not 0,1

#define FHSS_MAX_NUM    80  

uint32_t _seed;
bool is_in_binding;
uint8_t cnt ;
uint8_t curr_i= 0;


uint8_t ch_list[FHSS_CHANNELS_NUM]; // that's our list of randomly selected channels
uint32_t fhss_list[FHSS_CHANNELS_NUM]; // that's our list of randomly selected frequencies
int8_t fhss_last_rssi[FHSS_CHANNELS_NUM];

uint8_t freq_list_len;              // nimber of channels in the list of all frequencies e.g. 79
uint8_t fhss_bind_channel_list_len; // number of channels in bind list e.g. 1
uint8_t fhss_section_len;           // number of channels in a section  = fhss_freq_list / FHSS_SYNCHRO_CHANNELS_NUM ; // SECTION_NUM = 79 / 5 = 15
uint8_t fhss_max_channel ;          // max number of channels that may be used for synchro e.g. 15 *5 = 75 in order to stay in one of the section

const uint32_t* fhss_freq_list;
const uint8_t* fhss_bind_channel_list;

extern bool isConnected2Tx;

void  Fhss_Init()
{
    fhss_freq_list = fhss_freq_list_2p4;
    fhss_bind_channel_list = fhss_bind_channel_list_2p4;
    freq_list_len = (uint8_t)(sizeof(fhss_freq_list_2p4) / sizeof(uint32_t));
    fhss_bind_channel_list_len = (uint8_t)(sizeof(fhss_bind_channel_list_2p4) / sizeof(uint8_t));
    fhss_section_len = freq_list_len / FHSS_SYNCHRO_CHANNELS_NUM ; // SECTION_NUM = 79 / 5 = 15
    fhss_max_channel = fhss_section_len * FHSS_SYNCHRO_CHANNELS_NUM ; // 15 * 5 = 75

    cnt = FHSS_CHANNELS_NUM;
    curr_i = 0;
    is_in_binding = false;
    _seed = 0;
}

uint16_t prng(void)
{
    const uint32_t a = 214013;
    const uint32_t c = 2531011;
    const uint32_t m = 2147483648;
    _seed = (a * _seed + c) % m;    
    return _seed >> 16;
}

#ifdef USE_FHSS_WITH_SYNCHRO
    void  Fhss_generate(uint32_t seed)//
    {
        _seed = seed;
        bool used_flag[freq_list_len];
        bool used_synchro_flag[freq_list_len];
        for (uint8_t ch = 0; ch < freq_list_len; ch++){ 
            used_flag[ch] = false;
            used_synchro_flag[ch] = false;
        }
        uint8_t k = 0;
        uint8_t used_synchro_count = 0; // count the number of channels already used in used_synchro_flag  
        while (k < FHSS_SYNCHRO_CHANNELS_NUM) // first step = allocate 5 synchro in defferent range 
        {
            uint8_t rn = prng() % (freq_list_len - ( used_synchro_count)); // get a random number in the remaining range (note: Each syncro channel selected reserves several channels 15 or even more) 
            uint8_t i = 0;
            uint8_t ch;
            // do not take a channel that is already used 
            for (ch = 0; ch < freq_list_len; ch++) 
            {   // search the rn unused channels in used_synchro_list (where a whole section is used only once)
                if (used_synchro_flag[ch]) 
                    continue;
                if (i == rn) 
                    break; // ch is our next index
                i++;
            }
            if (ch >= freq_list_len) { // argh, must not happen !
                //Serial.println("should not happen");
                ch = 0;
            }
            // do not pick a channel that would exceed the n groups.
            if (ch >= fhss_max_channel) continue; 
            // do not pick a bind channel
            //bool is_bind_channel = false;
            //for (uint8_t bi = 0; bi < fhss_bind_channel_list_len; bi++) {
            //    if (ch == fhss_bind_channel_list[bi]) {
            //        is_bind_channel = true;
            //    }    
            //}
            //if (is_bind_channel) continue;
            
            // ensure it is not too close to the previous
            /*
            if (k > 0)     
            {
                if(ch_list[k-1]>ch)
                {
                    if( (ch_list[k-1]- ch) < 7)
                        continue;
                } else {
                    if( (ch - ch_list[k-1]) < 7)
                        continue;
                }    
            }
            */
            // we got a new ch, so register it
            ch_list[k] = ch;//ch index
            fhss_list[k] = fhss_freq_list[ch];
            used_flag[ch] = true;
            used_synchro_flag[ch] = true;
            uint8_t sectionStartAt = (ch/fhss_section_len) * fhss_section_len; // e.g. (50/15) * 15 = 45 : there are 5 sections of 15 channels so Channel section is 0...4
            // we marks the whole section as being used (so next synchro channel will be in another section)
            for (uint8_t i = 0; i < fhss_section_len ; i++){
                used_synchro_flag[sectionStartAt + i] = true;
            }
            // we mark the 7 adjacent channels as used to avoid having 2 synchro channels to close from each other
            for ( uint8_t i = 0 ; i< 7; i++){
                if (ch > i) { used_synchro_flag[ch-i-1] = true;}
                if (ch < (freq_list_len-i)) { used_synchro_flag[ch+i] = true;}
            }
            // count number of channels used in used_syncro_flag
            used_synchro_count = 0;
            for ( uint8_t i = 0 ; i < freq_list_len ; i++){
            if ( used_synchro_flag[i])  used_synchro_count++;
            }
            k++;
        }
        #ifdef DEBUG_FHSS
            Serial.println(" Synchro channels");
            for (uint8_t i = 0; i < 5; i++){
            Serial.println(ch_list[i]); 
            }
            Serial.println("------------");
        #endif
        // at this stage whe have 5 synchro channels spread in different sections
        // now we will allocate other channels taking care to avoid:
        //     - using a blocked channel
        //     - having a gap of 5 channels with channels n-1 and n-2
        while (k < cnt) 
        {
            uint8_t rn = prng() % (freq_list_len - FHSS_SYNCHRO_CHANNELS_NUM - k); // get a random number in the remaining range   
            uint8_t i = 0;
            uint8_t ch;
            for (ch = 0; ch < freq_list_len; ch++) 
            {   // search the rn unused channels
                if (used_flag[ch]) 
                    continue;
                if (i == rn) 
                    break; // ch is our next index
                i++;
            }
            if (ch >= freq_list_len) { // argh, must not happen !
                ch = 0;
            }
            // do not pick a bind channel : in fact it is not really needed
            //bool is_bind_channel = false;
            //for (uint8_t bi = 0; bi < fhss_bind_channel_list_len; bi++) {
            //    if (ch == fhss_bind_channel_list[bi]) {
            //        is_bind_channel = true;
            //    }    
            //}
            //if (is_bind_channel) continue;
            // ensure it is not too close to the 2 previous
            if (k > 0) {
                if(ch_list[k-1]>ch)
                {
                    if( (ch_list[k-1]- ch) < 5) continue;
                } else {
                    if( (ch - ch_list[k-1]) < 5) continue;
                }
            }
            if (k > 1) {
                if(ch_list[k-2]>ch)
                {
                    if( (ch_list[k-2]- ch) < 5) continue;
                } else {
                    if( (ch - ch_list[k-2]) < 5) continue;
                }    
            }
            // we got a new ch, so register it
            ch_list[k] = ch;//ch index
            fhss_list[k] = fhss_freq_list[ch];
            used_flag[ch] = true;
            k++;
        }
        #ifdef DEBUG_WITH_FIXED_FHSS
            for (uint8_t k = 0; k < cnt; k++) {
                ch_list[k] = debug_ch_list[k];
                fhss_list[k] = fhss_freq_list[ch_list[k]];
            }
        #endif
        curr_i = 0;
        //mark all channels as equally bad
        for (uint8_t k = 0; k < cnt; k++) {
            fhss_last_rssi[k] = -128 ;
        }
        #ifdef DEBUG_FHSS
            Serial.println(" All channels");
            for (uint8_t i = 0; i < cnt; i++){
            Serial.print(ch_list[i]); Serial.print(","); Serial.println(fhss_list[i]); 
            }
        #endif
    }
#endif    

void ICACHE_RAM_ATTR3 nextChannel(uint8_t skip )   // note : this version is different from Milo Tx
{
  curr_i  = (curr_i + skip)%cnt;       // curr_i is the index in the channel list 
  
  if ( !isConnected2Tx) {
    if ( curr_i >= FHSS_SYNCHRO_CHANNELS_NUM ) {
      curr_i = 0;  
    }
    //Serial.println(curr_i); // mstrens to debug
  }
  else {
    //Serial.print("."); Serial.println(curr_i); // mstrens to debug
  }
}

void ICACHE_RAM_ATTR3 setChannelIdx(uint8_t ch_idx )   // note : this version is different from Milo Tx
{
  curr_i  = ( ch_idx )%cnt;
  if ( !isConnected2Tx) {
    if ( curr_i >= FHSS_SYNCHRO_CHANNELS_NUM ) {      // only the 5 first entries in the list may be used to get a connection 
      curr_i = 0;  
    }
    
  }
}

    
uint32_t ICACHE_RAM_ATTR3 GetBindFreq()
{
    return fhss_freq_list[fhss_bind_channel_list[0]];
}

uint8_t getCurrentChannelIdx()    
{
    return curr_i; 
}

uint32_t  ICACHE_RAM_ATTR3 GetCurrFreq(void)
{
    if (is_in_binding) 
        return fhss_freq_list[fhss_bind_channel_list[0]];
    return fhss_list[curr_i];
}
    
uint32_t  bestX(void)
{
    uint8_t i_best = 0;
    for (uint8_t i = 0; i < cnt; i++) 
    {
        if (fhss_last_rssi[i] > fhss_last_rssi[i_best]) i_best = i;
    }
    curr_i = i_best;
    return fhss_list[curr_i];
}

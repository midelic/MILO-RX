//#pragma once
//from mLRS
//https://github.com/olliw42/mLRS/
// 2406.0 ... 2473.0  in 1 MHz steps
// = 68 channels//BW is 812Khz so band resolution of 1Mhz is suitable and no adiacent chanels so they will be randomized
#define SX1280_FREQ_XTAL_HZ               52000000
#define SX1280_FREQ_GHZ_TO_REG(f_ghz)     (uint32_t)((double)f_ghz*1.0E9*(double)(1 << 18)/(double)SX1280_FREQ_XTAL_HZ)
#define SX1280_REG_FIRMWARE_VERSION_MSB   0x0153 // address of the register holding firmware version MSB
#define SX1280_POWER_DBM_TO_REG(dbm)      (uint8_t)( (int8_t)dbm + 18 ) // -18 dbm = power 0, 13 dbm = power 31
// this is what is suggested by Semtech
// #include <math.h>
// #define SX1280_FREQ_STEP                  ((double)(SX1280_FREQ_XTAL_HZ / pow(2.0,18.0))) // 198.3642578
// #define SX1280_FREQ_HZ_TO_REG(f_hz)       ((uint32_t)( (double)f_hz / (double)SX1280_FREQ_STEP ))
// the pow(x,y) is ugly, and using GHz units is more convenient

const uint32_t fhss_freq_list_2p4[] = {
    SX1280_FREQ_GHZ_TO_REG(2.406), // channel 0
    SX1280_FREQ_GHZ_TO_REG(2.407),
    SX1280_FREQ_GHZ_TO_REG(2.408),
    SX1280_FREQ_GHZ_TO_REG(2.409),
    
    SX1280_FREQ_GHZ_TO_REG(2.410), // channel 4
    SX1280_FREQ_GHZ_TO_REG(2.411),
    SX1280_FREQ_GHZ_TO_REG(2.412),
    SX1280_FREQ_GHZ_TO_REG(2.413),
    SX1280_FREQ_GHZ_TO_REG(2.414),
    SX1280_FREQ_GHZ_TO_REG(2.415),
    SX1280_FREQ_GHZ_TO_REG(2.416),
    SX1280_FREQ_GHZ_TO_REG(2.417),
    SX1280_FREQ_GHZ_TO_REG(2.418),
    SX1280_FREQ_GHZ_TO_REG(2.419),
    
    SX1280_FREQ_GHZ_TO_REG(2.420), // channel 14
    SX1280_FREQ_GHZ_TO_REG(2.421),
    SX1280_FREQ_GHZ_TO_REG(2.422),
    SX1280_FREQ_GHZ_TO_REG(2.423),
    SX1280_FREQ_GHZ_TO_REG(2.424),
    SX1280_FREQ_GHZ_TO_REG(2.425),
    SX1280_FREQ_GHZ_TO_REG(2.426),
    SX1280_FREQ_GHZ_TO_REG(2.427),
    SX1280_FREQ_GHZ_TO_REG(2.428),
    SX1280_FREQ_GHZ_TO_REG(2.429),
    
    SX1280_FREQ_GHZ_TO_REG(2.430), // channel 24
    SX1280_FREQ_GHZ_TO_REG(2.431),
    SX1280_FREQ_GHZ_TO_REG(2.432),
    SX1280_FREQ_GHZ_TO_REG(2.433),
    SX1280_FREQ_GHZ_TO_REG(2.434),
    SX1280_FREQ_GHZ_TO_REG(2.435),
    SX1280_FREQ_GHZ_TO_REG(2.436),
    SX1280_FREQ_GHZ_TO_REG(2.437),
    SX1280_FREQ_GHZ_TO_REG(2.438),
    SX1280_FREQ_GHZ_TO_REG(2.439),
    
    SX1280_FREQ_GHZ_TO_REG(2.440), // channel 34
    SX1280_FREQ_GHZ_TO_REG(2.441),
    SX1280_FREQ_GHZ_TO_REG(2.442),
    SX1280_FREQ_GHZ_TO_REG(2.443),
    SX1280_FREQ_GHZ_TO_REG(2.444),
    SX1280_FREQ_GHZ_TO_REG(2.445),
    SX1280_FREQ_GHZ_TO_REG(2.446),
    SX1280_FREQ_GHZ_TO_REG(2.447),
    SX1280_FREQ_GHZ_TO_REG(2.448),
    SX1280_FREQ_GHZ_TO_REG(2.449),
    
    SX1280_FREQ_GHZ_TO_REG(2.450), // channel 44
    SX1280_FREQ_GHZ_TO_REG(2.451),
    SX1280_FREQ_GHZ_TO_REG(2.452),
    SX1280_FREQ_GHZ_TO_REG(2.453),
    SX1280_FREQ_GHZ_TO_REG(2.454),
    SX1280_FREQ_GHZ_TO_REG(2.455),
    SX1280_FREQ_GHZ_TO_REG(2.456),
    SX1280_FREQ_GHZ_TO_REG(2.457),
    SX1280_FREQ_GHZ_TO_REG(2.458),
    SX1280_FREQ_GHZ_TO_REG(2.459),
    
    SX1280_FREQ_GHZ_TO_REG(2.460), // channel 54
    SX1280_FREQ_GHZ_TO_REG(2.461),
    SX1280_FREQ_GHZ_TO_REG(2.462),
    SX1280_FREQ_GHZ_TO_REG(2.463),
    SX1280_FREQ_GHZ_TO_REG(2.464),
    SX1280_FREQ_GHZ_TO_REG(2.465),
    SX1280_FREQ_GHZ_TO_REG(2.466),
    SX1280_FREQ_GHZ_TO_REG(2.467),
    SX1280_FREQ_GHZ_TO_REG(2.468),
    SX1280_FREQ_GHZ_TO_REG(2.469),
    
    SX1280_FREQ_GHZ_TO_REG(2.470), // channel 64
    SX1280_FREQ_GHZ_TO_REG(2.471),
    SX1280_FREQ_GHZ_TO_REG(2.472),
    SX1280_FREQ_GHZ_TO_REG(2.473), // channel 67
};
const uint8_t fhss_bind_channel_list_2p4[] = {
    46 //,14, 33, 61 // just pick some
};

// https://en.wikipedia.org/wiki/Linear_congruential_generator: Microsoft Visual/Quick C/C++
// also used by ELRS
// generates values in range [0, 0x7FFF]

// spektrum:
// appears to use val = val * 0x0019660D + 0x3C6EF35F; (val >> 8) % 73
// is numerical recipes method in the wikipedia source
// picks 24 channels out of 74, ensures it's distributed into three slots

// redpine:
// same prng as spektrum, picks 50 channels, ensures not close and not 0,1

#define FHSS_FREQ_LIST_MAX_LEN  80 // 2.4 GHz is 68
#define FHSS_MAX_NUM    68
#define FHSS_CHANNELS_NUM  47
uint32_t _seed;
bool is_in_binding;
uint8_t cnt ;
uint8_t curr_i;

uint8_t ch_list[FHSS_MAX_NUM]; // that's our list of randomly selected channels
uint32_t fhss_list[FHSS_MAX_NUM]; // that's our list of randomly selected frequencies
int8_t fhss_last_rssi[FHSS_MAX_NUM];

uint8_t FREQ_LIST_LEN;
uint8_t FHSS_BIND_CHANNEL_LIST_LEN;
const uint32_t* fhss_freq_list;
const uint8_t* fhss_bind_channel_list;

#ifdef USE_HC_FHSS
    uint32_t fhss_list_hc[47] = {
        12345974,
        12421592,
        12174572,
        12164489,
        12305644,
        12325809,
        12386304,
        12204819,
        12245149,
        12179613,
        12416551,
        12356056,
        12184654,
        12169531,
        12265314,
        12330850,
        12255232,
        12235067,
        12320768,
        12300603,
        12426633,
        12159448,
        12290520,
        12129201,
        12351015,
        12214902,
        12280438,
        12446798,
        12340932,
        12381262,
        12461922,
        12371180,
        12451840,
        12366139,
        12396386,
        12315726,
        12376221,
        12230025,
        12154407,
        12406468,
        12441757,
        12456881,
        12209860,
        12219943,
        12199778,
        12295561,
        12335891,
    };
#endif


void ICACHE_RAM_ATTR Fhss_Init()
{
    fhss_freq_list = fhss_freq_list_2p4;
    fhss_bind_channel_list = fhss_bind_channel_list_2p4;
    FREQ_LIST_LEN = (uint8_t)(sizeof(fhss_freq_list_2p4) / sizeof(uint32_t));
    FHSS_BIND_CHANNEL_LIST_LEN = (uint8_t)(sizeof(fhss_bind_channel_list_2p4) / sizeof(uint8_t));
    cnt = FHSS_CHANNELS_NUM;
    curr_i = 0;
    is_in_binding = false;
    _seed = 0;
}

uint16_t ICACHE_RAM_ATTR prng(void)
{
    const uint32_t a = 214013;
    const uint32_t c = 2531011;
    const uint32_t m = 2147483648;
    
    _seed = (a * _seed + c) % m;
    
    return _seed >> 16;
}

void ICACHE_RAM_ATTR Fhss_generate(uint32_t seed)//
{
    _seed = seed;
    
    bool used_flag[FREQ_LIST_LEN];
    
    for (uint8_t ch = 0; ch < FREQ_LIST_LEN; ch++) 
        used_flag[ch] = false;
    
    uint8_t k = 0;
    while (k < cnt) 
    {
        uint8_t rn = prng() % (FREQ_LIST_LEN - k); // get a random number in the remaining range
        
        uint8_t i = 0;
        uint8_t ch;
        
        for (ch = 0; ch < FREQ_LIST_LEN; ch++) 
        {
            if (used_flag[ch]) continue;
            if (i == rn) break; // ch is our next index
            i++;
        }
        
        if (ch >= FREQ_LIST_LEN) 
        { // argh, must not happen !
            ch = 0;
        }
        
        // do not pick a bind channel
        bool is_bind_channel = false;
        for (uint8_t bi = 0; bi < FHSS_BIND_CHANNEL_LIST_LEN; bi++) {
            if (ch == fhss_bind_channel_list[bi])
                is_bind_channel = true;
        }
        if (is_bind_channel) 
            continue;
        
        // ensure it is not too close to the previous
        bool is_too_close = false;
        if (k > 0) 
        {
            int8_t last_ch = ch_list[k - 1];
            if (last_ch == 0) 
            { // special treatment for this case
                if (ch < 2) 
                    is_too_close = true;
            } 
            else 
            {
                if ((ch >= last_ch - 1) && (ch <= last_ch + 1)) is_too_close = true;
            }
        }
        if (is_too_close) 
            continue;
        
        // we got a new ch, so register it
        ch_list[k] = ch;//ch index
        fhss_list[k] = fhss_freq_list[ch];
        
        used_flag[ch] = true;
        
    k++;
    }
    
    curr_i = 0;
    
    //mark all channels as equally bad
    for (uint8_t k = 0; k < cnt; k++) 
    {
        fhss_last_rssi[k] = -128 ;
    }
    
    #ifdef DEBUG_FHSS
        delay(1000);
        for (uint8_t i = 0; i < cnt; i++) 
            Serial.println(fhss_list[i]); 
    #endif
}
    
void ICACHE_RAM_ATTR nextChannel(uint8_t skip )
{
    curr_i  = (curr_i + skip)%cnt;
}
    
uint32_t ICACHE_RAM_ATTR GetInitialFreq()
{
    return fhss_freq_list[fhss_bind_channel_list[0]];
}
    
uint32_t  ICACHE_RAM_ATTR GetCurrFreq(void)
{
if (is_in_binding) 
    return fhss_freq_list[fhss_bind_channel_list[0]];

return fhss_list[curr_i];
}
    
uint32_t ICACHE_RAM_ATTR bestX(void)
{
    uint8_t i_best = 0;
    for (uint8_t i = 0; i < cnt; i++) 
    {
        if (fhss_last_rssi[i] > fhss_last_rssi[i_best]) 
            i_best = i;
    }
    
    curr_i = i_best;
    return fhss_list[curr_i];
}

    #pragma once
    #define SX1280_FREQ_XTAL_MHZ               52
    #define SX1280_FREQ_MHZ_TO_REG(f_mhz)     (uint32_t)(((double)(f_mhz+2402)/(double)SX1280_FREQ_XTAL_MHZ)*(double)(1 << 18))
    #define SHA256_BLOCK_SIZE 32
    #define FHSS_HOPPING_CHANNELS 36

    // define bind channel
    const uint8_t fhss_bind_channel_list/*_2p4*/[] = 
    {
        0 //,14, 33, 61 // just pick some
    };

    
    // channel list expanded to match list in Bluetooth core specification 5.3 page 389: f=2402+k MHz, k=0..78
    const uint32_t fhss_freq_list/*_2p4*/[] = 
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
    
    // tripple block of "good" channels
    const uint8_t fhss_block_good[8][3] = 
    {
        {19, 44, 70},
        {20, 45, 71},
        {21, 46, 72},
        {22, 47, 73},
        {23, 48, 74},
        {24, 49, 75},
        {25, 50, 76},
        {26, 51, 77},
    };
    

    // tripple block of "bad" channels
    const uint8_t fhss_block_bad[17][3] = 
    {
        {2,  27, 52},
        {3,  28, 53},
        {4,  29, 54},
        {5,  30, 55},
        {6,  31, 56},
        {7,  32, 57},
        {8,  33, 58},
        {9,  34, 59},
        {10, 35, 60},
        {11, 36, 61},
        {12, 37, 62},
        {13, 38, 63},
        {14, 39, 64},
        {15, 40, 65},
        {16, 41, 66},
        {17, 42, 67},
        {18, 43, 68},
    };

    // randomise permutation of "fix" channels
    const uint8_t fhss_block_fix[24][4] = 
    {
        {0, 1, 69, 78},
        {0, 1, 78, 69},
        {0, 69, 1, 78},
        {0, 69, 78, 1},
        {0, 78, 1, 69},
        {0, 78, 69, 1},
        {1, 0, 69, 78},
        {1, 0, 78, 69},
        {1, 69, 0, 78},
        {1, 69, 78, 0},
        {1, 78, 69, 0},
        {1, 78, 0, 69},
        {69, 0, 1, 78},
        {69, 0, 78, 1},
        {69, 1, 0, 78},
        {69, 1, 78, 0},
        {69, 78, 0, 1},
        {69, 78, 1, 0},
        {78, 0, 1, 69},
        {78, 0, 69, 1},
        {78, 1, 0, 69},
        {78, 1, 69, 0},
        {78, 69, 0, 1},
        {78, 69, 1, 0},
    };

    uint32_t _seed;
    bool is_in_binding;
    uint8_t cnt ;
    uint8_t curr_i;

    uint8_t hoppingTable[FHSS_HOPPING_CHANNELS];
    uint8_t ch_list[FHSS_HOPPING_CHANNELS]; // that's our list of randomly selected channels
    uint32_t fhss_list[FHSS_HOPPING_CHANNELS]; // that's our list of randomly selected frequencies
//    int8_t fhss_last_rssi[FHSS_MAX_NUM];//

//    uint8_t freq_list_len;
//    uint8_t fhss_bind_channel_list_len;
//    const uint32_t* fhss_freq_list;
//    const uint8_t* fhss_bind_channel_list;
        
    void ICACHE_RAM_ATTR Fhss_Init();
                
    void ICACHE_RAM_ATTR Fhss_generate(uint32_t seed);

    void ICACHE_RAM_ATTR nextChannel();

    uint32_t ICACHE_RAM_ATTR GetInitialFreq();
    
    uint32_t ICACHE_RAM_ATTR GetCurrFreq(void);
    

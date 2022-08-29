
#ifdef EU_LBT
#include "iface_sx1280.h"
#include "SX1280.h"
#if !defined(LBT_RSSI_THRESHOLD_OFFSET_DB)
  #define LBT_RSSI_THRESHOLD_OFFSET_DB 0
#endif

static uint32_t rxStartTime;
extern uint8_t CurrentPower;

uint32_t ICACHE_RAM_ATTR SpreadingFactorToRSSIvalidDelayUs(uint8_t SF)
{
  // The necessary wait time from RX start to valid instant RSSI reading
  // changes with the spreading factor setting.
  // The worst case necessary wait time is TX->RX switch time + Lora symbol time
  // This assumes the radio switches from either TX, RX or FS (Freq Synth mode)
  // TX to RX switch time is 60us for sx1280
  // Lora symbol time for the relevant spreading factors is:
  // SF5: 39.4us
  // SF6: 78.8us
  // SF7: 157.6us
  // SF9: 630.5us
  // However, by measuring when the RSSI reading is stable and valid, it was found that
  // actual necessary wait times are:
  // SF5 ~100us (60us + SF5 symbol time)
  // SF6 ~141us (60us + SF6 symbol time)
  // SF7 ~218us (60us + SF7 symbol time)
  // SF9 ~218us (Odd one out, measured to same as SF7 wait time)

  switch(SF)
    {
      case SX1280_LORA_SF5: return 100;
      case SX1280_LORA_SF6: return 141;
      case SX1280_LORA_SF7: return 218;
      case SX1280_LORA_SF9: return 218;
      default: return 218; // Values above 100mW are not relevant, default to 100mW threshold
    }
}

int8_t ICACHE_RAM_ATTR PowerEnumToLBTLimit(uint8_t  txPower)
{
  // Calculated from EN 300 328, adjusted for 800kHz BW for sx1280
  // TL = -70 dBm/MHz + 10*log10(0.8MHz) + 10 × log10 (100 mW / Pout) (Pout in mW e.i.r.p.)
  // This threshold should be offset with a #define config for each HW that corrects for antenna gain,
  // different RF frontends.
  // TODO: Maybe individual adjustment offset for differences in
  // rssi reading between bandwidth setting is also necessary when other BW than 0.8MHz are used.

  switch(txPower)
  {
    case PWR_10mW: return -61 + LBT_RSSI_THRESHOLD_OFFSET_DB;
    case PWR_25mW: return -65 + LBT_RSSI_THRESHOLD_OFFSET_DB;
    case PWR_50mW: return -68 + LBT_RSSI_THRESHOLD_OFFSET_DB;
    case PWR_100mW: return -71 + LBT_RSSI_THRESHOLD_OFFSET_DB;
    // Values above 100mW are not relevant, default to 100mW threshold
    default: return -71 + LBT_RSSI_THRESHOLD_OFFSET_DB;
  }
}

void ICACHE_RAM_ATTR BeginClearChannelAssessment(void)
{

	#ifdef HAS_PA_LNA
	SX1280_SetTxRxMode(RX_EN);// do first to enable LNA 
	#endif
	SX1280_SetMode(SX1280_MODE_RX);//start RX mode in order to get RSSI
    rxStartTime = micros();
}

bool ICACHE_RAM_ATTR ChannelIsClear(void)
{

  // Read rssi after waiting the minimum RSSI valid delay.
  // If this function is called long enough after RX enable,
  // this will always be ok on first try as is the case for TX.

  // TODO: Better way than busypolling this for RX?
  // this loop should only run for RX, where listen before talk RX is started right after FHSS hop
  // so there is no dead-time to run RX before telemetry TX is supposed to happen.
  // if flipping the logic, so telemetry TX happens right before FHSS hop, then TX-side ends up with polling here instead?
  // Maybe it could be skipped if there was only TX of telemetry happening when FHSShop does not happen.
  // Then RX for LBT could stay enabled from last received packet, and RSSI would be valid instantly.
  // But for now, FHSShops and telemetry rates does not divide evenly, so telemetry will some times happen
  // right after FHSS and we need wait here.

  uint32_t validRSSIdelayUs = SpreadingFactorToRSSIvalidDelayUs(MiLo_currAirRate_Modparams->sf);
  uint32_t elapsed = micros() - rxStartTime;
  if(elapsed < validRSSIdelayUs)
  {
    delayMicroseconds(validRSSIdelayUs - elapsed);
  }

  int8_t rssiInst = SX1280_GetRssiInst();
   SX1280_SetMode(SX1280_MODE_FS);//SX1280_SetTxIdleMode();
  bool channelClear = rssiInst < PowerEnumToLBTLimit(CurrentPower);//CurrentPower
  return channelClear;
}
#endif



